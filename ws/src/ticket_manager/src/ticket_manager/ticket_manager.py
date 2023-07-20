#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Defines the Ticket Manager class, which maintains ticket states and
    provides their information over a service to the GUIs. It requests
    schedules from the Task Scheduler as needed.
'''
import math
import rospy

from arm_msgs.msg import Ticket, Tickets
from arm_msgs.srv import Schedule, ScheduleRequest, TicketList, TicketListResponse

from arm_utils.data_utils import create_ticket_list, convert_ticket_list_to_task_dict
from arm_utils.job_utils import get_all_children_from_task_list, get_all_parents_from_task_list


log_tag = "Ticket Manager"


class TicketManager():
    '''Ticket manager class. Responsible for maintaining ticket statuses.

    Workhorse of the ticketing system. Subscribes to:
        add_ticket
        edit_ticket
        delete_ticket
        delete_job
        start_ticket
        end_ticket
    topics. These topics are published to by the Supervisor and Operator GUIs.
    '''

    def __init__(self) -> None:
        '''.'''
        # Spawn the ROS node and announce its startup.
        rospy.init_node('ticket_manager')
        rospy.on_shutdown(self.shutdown_ticket_manager)
        rospy.loginfo(f"{log_tag}: Node started.")

        # Ticket list and subsets - waiting, ready, ongoing, done, deleted.
        self.ticket_dict = {}
        self.waiting = {}
        self.ready = {}
        self.ongoing = {}
        self.done = {}
        self.deleted = {}

        # The lowest job ID we can use when adding a new one to the list.
        self.minJobID = 0

        # Timer variables for triggering ticket starts and re-scheduling.
        self.ongoing_timer = None
        self.ongoing_timer_id = 0
        self.ongoing_timer_set_time = 0
        self.ongoing_start_times = {}
        self.lowest_time_left = 0

        # Subscribers for ticket management. Add/edit/remove ticket/job.
        self.add_ticket_sub = rospy.Subscriber(
            "add_ticket", Tickets, self.add_ticket_message_callback
        )
        self.edit_ticket_sub = rospy.Subscriber(
            "edit_ticket", Tickets, self.edit_ticket_message_callback
        )
        self.delete_ticket_sub = rospy.Subscriber(
            "delete_ticket", Tickets, self.delete_ticket_message_callback
        )
        self.delete_job_sub = rospy.Subscriber(
            "delete_job", Tickets, self.delete_job_message_callback
        )

        # Subscribers for starting and ending tasks.
        self.start_ticket_sub = rospy.Subscriber(
            "start_ticket", Ticket, self.start_ticket_message_callback
        )
        self.end_ticket_sub = rospy.Subscriber(
            "end_ticket", Ticket, self.end_ticket_message_callback
        )

        # Publisher. TODO: Why?
        self.ticket_started_pub = rospy.Publisher(
            "ticket_started", Ticket, queue_size=100
        )

        # Service for ticket lists.
        self.ticket_service = rospy.Service(
            'ticket_service', TicketList, self.send_ticket_lists
        )

        self.time_left_update_interval = 1
        self.time_left_update_timer = rospy.Timer(
            rospy.Duration(self.time_left_update_interval),
            self.update_time_left,
            oneshot=False
        )

        # Spin.
        rospy.spin()

    def shutdown_ticket_manager(self):
        '''Gracefully shutdown ticket manager.'''
        # TODO: Save the current ticket list, so we can pick up where we stopped.
        rospy.loginfo(f"{log_tag}: Node shutdown.")

    def add_ticket_message_callback(self, msg):
        '''Callback for new tickets received.

        Assigns job IDs to the received tickets, adds them to the ticket list,
        then requests a new schedule with the update list.
        '''
        rospy.loginfo(f"{log_tag}: Received new set of tickets."
                        " Adding to ticket list.")

        temp_dict = self.convert_ticket_list_to_temp_dict(msg.tickets)
        temp_dict = self.get_received_ticket_job_ids(temp_dict)
        self.add_received_tickets_to_ticket_dict(temp_dict)
        self.request_schedule()

    def edit_ticket_message_callback(self, msg):
        '''Callback when an edit_ticket message is received.

        Only parents, duration and machine type can be changed.
        TODO: is changing parents really reasonable?
        '''
        temp_dict = self.convert_ticket_list_to_temp_dict(msg.tickets)
        edited_tickets = []

        for ticket_id, ticket in temp_dict.items():
            if ticket_id in self.ticket_dict:
                if ticket_id in self.ongoing:
                    rospy.loginfo(f"{log_tag}: Editing ongoing ticket {ticket_id}.")
                    del(self.ongoing[ticket_id])
                if ticket_id in self.ready:
                    del(self.ready[ticket_id])

                # Update the ticket and put it back in waiting.
                self.ticket_dict[ticket_id]["parents"] = ticket["parents"]
                self.ticket_dict[ticket_id]["machine_type"] = ticket["machine_type"]
                self.ticket_dict[ticket_id]["duration"] = ticket["duration"]
                self.ticket_dict[ticket_id]["time_left"] = ticket["duration"]

                self.waiting[ticket_id] = self.ticket_dict[ticket_id]

                edited_tickets.append(ticket_id)
            elif ticket_id in self.done:
                # Check if it's already done. GUI won't let this happen, but
                # cross-check.
                rospy.loginfo(f"{log_tag}: Attempted to edit a done ticket "
                              f"{ticket_id}. Ignoring")
                continue
            else:
                rospy.logwarn(f"{log_tag}: Attempted to edit a non-existent "
                              "ticket. Ignoring")

        rospy.loginfo(f"{log_tag}: Edited tickets {edited_tickets}.")

        # Update the ready set.
        self.update_ready()

    def delete_ticket_message_callback(self, msg):
        '''Callback when a delete_ticket message is received.

        Deletes the tickets and all their children to avoid dangling branches.
        '''
        ticket_list = msg.tickets
        delete_ids = []
        for ticket in ticket_list:
            ticket_id = ticket.ticket_id
            if ticket_id in self.ticket_dict:
                # Find all the children of the ticket.
                linear_job = [ticket_id]
                get_all_children_from_task_list(
                    ticket_id, self.ticket_dict, linear_job
                )

                # Add all the children to the delete_ids.
                for child_id in linear_job:
                    delete_ids.append(child_id)

        # Get only the unique IDs and then delete all of them.
        # We might be given a set of tickets from the same job.
        delete_ids = set(delete_ids)
        for ticket_id in delete_ids:
            self.delete_ticket(ticket_id)

        rospy.loginfo(f"{log_tag}: Deleted tickets {delete_ids}.")

        # Update the sets.
        self.update_ready()
        self.update_ongoing_time_left()
        self.set_ongoing_timer()

    def delete_job_message_callback(self, msg):
        '''Callback when a delete_job message is received.'''
        # Only the ticket and job IDs matter here.
        job_id = 0
        deleted_ticket_ids = []
        for ticket in msg.tickets:
            self.delete_ticket(ticket.ticket_id)
            job_id = ticket.job_id
            deleted_ticket_ids.append(ticket.ticket_id)

        rospy.loginfo(f"{log_tag}: Deleted job {job_id} with tickets "
                      f"{deleted_ticket_ids}."
        )

        # Update the sets.
        self.update_ready()
        self.update_ongoing_time_left()
        self.set_ongoing_timer()

    def start_ticket_message_callback(self, msg):
        '''Called when a ticket is set to start by the Operator GUI.

        Puts the ticket in the ongoing set and updates the ongoing timer.
        '''
        # Get the ticket ID.
        ticket_id = msg.ticket_id

        # Cross-checker. If a ticket is set to start and it's not ready,
        # don't start it. This should never be necessary, as the Op GUI
        # won't give users the option if the ticket is not ready.
        if not ticket_id in self.ready:
            rospy.logwarn(f"{log_tag}: Received a start signal for ticket "
                          f"{ticket_id} which isn't ready. Ignoring.")
            return

        # Subtract elapsed time from all currently ongoing tix.
        self.update_ongoing_time_left()

        # Add the ticket ID to the ongoing set and mark when it started.
        self.ongoing[ticket_id] = self.ticket_dict[ticket_id]
        self.ongoing_start_times[ticket_id] = rospy.Time.now().to_sec()

        # Remove the ticket from the ready set.
        del(self.ready[ticket_id])

        # Set the ongoing timer.
        self.set_ongoing_timer()

        # Announce that the ticket has started?
        self.announce_ticket_start(ticket_id)

        rospy.loginfo(f"{log_tag}: Ticket {ticket_id} set to ongoing.")

        # # Add the ticket to the executed schedule.
        # self.add_started_ticket_to_schedule(ticket_id, self.ticket_dict[ticket_id])

    def end_ticket_message_callback(self, msg):
        '''Callback when a done signal is received.'''
        try:
            # Update the ongoing time left for all ongoing tix.
            self.update_ongoing_time_left()

            # End the ticket and get how much time it had left
            ticket_time_left = self.end_ticket(msg.ticket_id)

            # Update ready and set timer.
            self.update_ready()
            self.set_ongoing_timer()

            # Add the done ticket to executed schedule?
            # self.add_done_ticket_to_schedule(msg.ticket_id)

            # If the timer has more than 5 minutes left, we re-schedule.
            # Otherwise, we let it run down.
            if ticket_time_left >= 5:
                self.request_schedule()

        except KeyError as e:
            rospy.logerr(f"Error ending ticket with ID {msg.ticket_id}.")
            rospy.logerr(e)

    def send_ticket_lists(self, request):
        '''Returns the complete list of tickets and the different subsets.'''
        rospy.logdebug(f"{log_tag}: Returning current ticket information.")

        # Merge ticket_dict and done into one for the list of all tickets.
        # The two are only separate for the sake of generating schedules.
        all_tickets = {**self.ticket_dict, **self.done}
        all_tickets_list = create_ticket_list(all_tickets)

        waiting = create_ticket_list(self.waiting)
        ready = create_ticket_list(self.ready)
        ongoing = create_ticket_list(self.ongoing)
        done = create_ticket_list(self.done)

        return TicketListResponse(all_tickets_list, waiting, ready, ongoing, done)

    def request_schedule(self):
        '''Requests a new schedule from the schedule service.

        The request sends the whole ticket list and the ongoing tasks, so the
        scheduler knows which machine assignments to respect.        
        '''
        rospy.wait_for_service('schedule_service')
        try:
            request = ScheduleRequest()
            request.tickets = create_ticket_list(self.ticket_dict)
            request.ongoing = create_ticket_list(self.ongoing)

            schedule = rospy.ServiceProxy('schedule_service', Schedule)
            response = schedule(request)

            # Update the ticket list and its subsets with the received schedule.
            self.on_schedule_update(response.tickets)
        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Schedule request failed: {e}.')

    def on_schedule_update(self, tickets: Tickets):
        '''Updates the ticket list and timers based on a new schedule.'''
        updated_task_dict = convert_ticket_list_to_task_dict(tickets)

        for ticket_id, old_ticket in self.ticket_dict.items():
            self.update_tickets_from_schedule(old_ticket, updated_task_dict[ticket_id])

        self.update_ready()
        self.update_ongoing_time_left()
        self.set_ongoing_timer()

    def update_tickets_from_schedule(self, old_ticket, new_ticket):
        '''Updates an old ticket based on a new ticket from a schedule.

        The schedule assigns the task start and end times, and a machine.
        Time left is also updated because of integer requirements of the
        CP-SAT solver.
        '''
        old_ticket["start"] = new_ticket["start"]
        old_ticket["end"] = new_ticket["end"]
        old_ticket["time_left"] = new_ticket["time_left"]
        old_ticket["machine_num"] = new_ticket["machine_num"]

    def convert_ticket_list_to_temp_dict(self, ticket_list):
        '''.'''
        temp_dict = {}
        for ticket in ticket_list:
            tix = {}
            tix["machine_type"] = ticket.machine_type
            tix["duration"] = ticket.duration
            tix["parents"] = list(ticket.parents)

            # Time left in seconds.
            tix["time_left"] = ticket.duration
            temp_dict[ticket.ticket_id] = tix
        return temp_dict

    def get_received_ticket_job_ids(self, temp_dict: dict):
        '''Gets the job IDs for the tickets received.

        Figures out the job IDs for the tickets received. They are either
        related to an existing job, or brand new.
        '''
        # Iterate through the temp dictionary and search for its parents in the
        # main ticket list. If they're not in there, save the key in a list.
        # If any parent is, add the job_id. 
        # If there are no parents, create a new job_id.
        # Lastly, go through the dictionary and search for its parents in the
        # temp dictionary. Go as high in the job tree, get the top job_id,
        # then trickle it down to the descendants.

        # Step 1.
        no_found_parents = []
        for ticket_id, ticket in temp_dict.items():
            found_parent = False
            if len(ticket["parents"]) == 0:
                self.minJobID += 1
                ticket["job_id"] = self.minJobID
            for parent_id in ticket["parents"]:
                if parent_id in self.ticket_dict:
                    ticket["job_id"] = self.ticket_dict[parent_id]["job_id"]
                    found_parent = True
            if not found_parent:
                no_found_parents.append(ticket_id)
        
        # Step 2. Iterate through the tickets with no found parents.
        # For each, look for the parents in the dictionary recursively.
        # When we get to the top, set the job_id to the ticket.
        for ticket_id in no_found_parents:
            ticket = temp_dict[ticket_id]
            # start_ids = get_tree_job_start_ids(ticket_id, temp_dict)
            # linear_job = [start_ids[0]]
            # get_all_children_from_task_list(start_ids[0], temp_dict, linear_job)
            linear_job = [ticket_id]
            get_all_parents_from_task_list(ticket_id, temp_dict, linear_job)

            # Set the job_id for the ticket.
            # TODO: Set it for all descendants at once?
            top_ticket = temp_dict[linear_job[-1]]
            ticket["job_id"] = top_ticket["job_id"]

        return temp_dict

    def add_received_tickets_to_ticket_dict(self, received_tickets: dict):
        '''Adds the modified received tickets dictionary to the main ticket list.'''
        for ticket_id, ticket in received_tickets.items():
            ticket["time_left"] = ticket["duration"]

            # Add the ticket to ticket_dict and waiting set.
            self.ticket_dict[ticket_id] = ticket
            self.waiting[ticket_id] = self.ticket_dict[ticket_id]

    def delete_ticket(self, ticket_id):
        '''Removes ticket_id from all sets.'''
        # TODO: This isn't robust.
        # Delete the ticket from ticket_dict and any set it might be in.
        if ticket_id in self.ticket_dict:
            self.deleted[ticket_id] = self.ticket_dict[ticket_id]
            self.deleted[ticket_id]["time_left"] = 0
            del(self.ticket_dict[ticket_id])
            if ticket_id in self.waiting:
                del(self.waiting[ticket_id])
            if ticket_id in self.ready:
                del(self.ready[ticket_id])
            if ticket_id in self.ongoing:
                del(self.ongoing[ticket_id])
        # If the ticket is already done, remove it from there.
        elif ticket_id in self.done:
            del(self.done[ticket_id])

    def end_ticket(self, ticket_id) -> float:
        '''Moves ticket_id from ongoing to done.
        
        Returns:
            ticket_time_left: float of how much time was left.
        '''
        # Get how much time was left on the ticket.
        ticket_time_left = self.ticket_dict[ticket_id]["time_left"]

        # Add the ticket to done set.
        self.done[ticket_id] = self.ticket_dict[ticket_id]
        self.done[ticket_id]["time_left"] = 0

        # Delete the ticket from ongoing and ticket_dict.
        del(self.ongoing[ticket_id])
        del(self.ticket_dict[ticket_id])
        rospy.loginfo(f"{log_tag}: Ended ticket {ticket_id}.")

        return ticket_time_left

    def announce_ticket_start(self, ticket_id):
        '''Announces that we're starting the ticket.'''
        msg = Ticket()
        msg.ticket_id = ticket_id
        msg.job_id = self.ticket_dict[ticket_id]["job_id"]
        msg.machine_type = self.ticket_dict[ticket_id]["machine_type"]
        msg.duration = self.ticket_dict[ticket_id]["duration"]
        msg.parents = self.ticket_dict[ticket_id]["parents"]
        msg.start = self.ticket_dict[ticket_id]["start"]
        msg.end = self.ticket_dict[ticket_id]["end"]
        msg.machine_num = self.ticket_dict[ticket_id]["machine_num"]
        self.ticket_started_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Starting ticket {ticket_id}.")

    def set_ongoing_timer(self):
        '''Sets the timer for the shortest time left on an ongoing ticket.'''
        # Find the lowest time_left ticket in ongoing.
        lowest_time_left = 50000
        for ticket_id in self.ongoing.keys():
            ticket = self.ticket_dict[ticket_id]
            if ticket["time_left"] <= lowest_time_left:
                lowest_time_left = ticket["time_left"]
                self.ongoing_timer_id = ticket_id
        self.lowest_time_left = lowest_time_left

        # Start a new timer with the lowest time left.
        if self.ongoing_timer is not None:
            self.ongoing_timer.shutdown()
        self.ongoing_timer = rospy.Timer(
            rospy.Duration(lowest_time_left+0.5), # Add 0.5 because 0 timers cause error.
            self.on_ongoing_timer_trigger,
            oneshot=True
        )
        self.ongoing_timer_set_time = rospy.Time.now().to_sec()

    def on_ongoing_timer_trigger(self, event):
        '''Updates the ongoing set when ongoing_timer is triggered.

        Ongoing timer is triggered when the estimated duration for a ticket has
        elapsed, meaning the ticket is taking longer than expected.
        '''
        # Add time to triggering ticket.
        rospy.loginfo(f"{log_tag}: Adding time to ticket {self.ongoing_timer_id}.")
        rospy.loginfo(f"{log_tag}: Old time left: {self.ticket_dict[self.ongoing_timer_id]['time_left']}.")
        self.add_time_to_ticket(self.ongoing_timer_id)
        rospy.loginfo(f"{log_tag}: New time left: {self.ticket_dict[self.ongoing_timer_id]['time_left']}.")

        # Request a new schedule.
        rospy.loginfo(f"{log_tag}: Requesting new schedule.")
        self.request_schedule()

    def add_time_to_ticket(self, ticket_id):
        '''Adds 25% of original duration to time left on the given ticket.'''
        try:
            self.ticket_dict[ticket_id]["time_left"] = math.ceil(0.25*\
                                self.ticket_dict[ticket_id]["duration"])
        except KeyError as e:
            # If the ticket is not in here, it must have finished as scheduled.
            rospy.loginfo(f"{log_tag}: Ticket {ticket_id} not in the"
                          " ticket list. Might have ended on time.")
            rospy.logwarn(e)

    def update_time_left(self, event):
        '''Updates ongoing set's time left. Keeps the time up to date for the GUI.'''
        # TODO: Thread safety. Doing this means there's a possibility that the
        # function is called from multiple threads.
        self.update_ongoing_time_left()

    def update_ongoing_time_left(self):
        '''Updates the time left on all ongoing tickets.'''
        # Get the time that has passed since ongoing timer was last started.
        ongoing_time_elapsed = rospy.Time.now().to_sec() -\
                                self.ongoing_timer_set_time
        ongoing_time_left = self.lowest_time_left - ongoing_time_elapsed

        # Subtract the elapsed time from all ongoing tickets' time_left.
        # TODO: Probable source of the negative values being printed.
        for ticket_id in self.ongoing.keys():
            self.ticket_dict[ticket_id]["time_left"] -= ongoing_time_elapsed
        return ongoing_time_left

    def update_ready(self):
        '''Moves tickets whose parents are done from waiting to ready.'''
        added_ticket_ids = []
        for ticket_id in self.waiting.keys():
            ticket = self.ticket_dict[ticket_id]
            num_parents = len(ticket["parents"])
            num_done_parents = 0
            for parent in ticket["parents"]:
                if parent in self.done.keys():
                    num_done_parents += 1
            if num_done_parents == num_parents:
                self.ready[ticket_id] = ticket
                added_ticket_ids.append(ticket_id)

        # Remove the newly ready tickets from waiting.
        for ticket_id in added_ticket_ids:
            del(self.waiting[ticket_id])