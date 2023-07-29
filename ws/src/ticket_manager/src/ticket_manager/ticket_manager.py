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

from std_msgs.msg import String

from arm_msgs.msg import Ticket, Tickets
from arm_msgs.srv import Schedule, ScheduleRequest, TicketList, TicketListResponse

from arm_utils.conversion_utils import convert_task_dict_to_ticket_list, convert_ticket_list_to_task_dict
from arm_utils.job_utils import get_all_children_from_task_list, get_all_parents_from_task_list, get_job_last_ticket_status


log_tag = "Ticket Manager"


class TicketManager():
    '''Ticket manager class. Responsible for maintaining ticket statuses.

    Workhorse of the ticketing system. Subscribes to:
        add_ticket
        edit_ticket
        delete_ticket - currently disabled.
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

        # Ticket dictionary and its subsets - waiting, ready, ongoing.
        # The dictionary is what is sent to the scheduler repeatedly.
        # The subsets are lists of IDs for minimal space use.
        self.ticket_dict = {}
        self.waiting = []
        self.ready = []
        self.ongoing = []

        # When a ticket is done or deleted, it's removed from ticket_dict
        # and placed in done or deleted. It is no longer relevant for
        # scheduling.
        self.done = {}
        self.deleted = {}

        # Dictionary of {job_id: {"tickets": [ticket_ids], "status": ""}, ...}.
        # Status is either Finished or Unfinished.
        self.jobs = {}

        # The lowest job ID we can use when adding a new one to the list.
        self.minJobID = 0

        # Timer variables for triggering re-scheduling.
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
        # Currently disabling ticket deletion to avoid dealing with all
        # the complications that come with it.
        # self.delete_ticket_sub = rospy.Subscriber(
        #     "delete_ticket", Tickets, self.delete_ticket_message_callback
        # )
        # Delete job uses a Ticket, but only the job_id matters.
        self.delete_job_sub = rospy.Subscriber(
            "delete_job", Ticket, self.delete_job_message_callback
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

        # Publisher for announcing whenever the ticket list has changed.
        self.ticket_list_update_pub = rospy.Publisher(
            "ticket_list_update", String, queue_size=100
        )

        # Service for ticket lists.
        self.ticket_service = rospy.Service(
            'ticket_service', TicketList, self.send_ticket_lists
        )

        self.time_left_update_interval = 5
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
        then requests a new schedule with the updated list.
        '''
        rospy.loginfo(f"{log_tag}: Received new set of tickets."
                        " Adding to ticket list.")

        temp_dict = self.convert_ticket_list_to_temp_dict(msg.tickets)
        temp_dict = self.get_received_ticket_job_ids(temp_dict)
        self.add_received_tickets_to_ticket_dict(temp_dict)

        self.request_schedule()

        # Update ticket and job statuses.
        self.update_jobs()
        self.update_ticket_statuses()
        self.update_job_statuses()

        self.announce_ticket_list_update()

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
                    self.ongoing.remove(ticket_id)
                elif ticket_id in self.ready:
                    self.ready.remove(ticket_id)

                # Update the ticket.
                # self.ticket_dict[ticket_id]["parents"] = ticket["parents"]
                self.ticket_dict[ticket_id]["machine_type"] = ticket["machine_type"]
                self.ticket_dict[ticket_id]["duration"] = ticket["duration"]
                self.ticket_dict[ticket_id]["time_left"] = ticket["duration"]
                # self.ticket_dict[ticket_id]["num_robots"] = ticket["num_robots"]

                # Put it back in waiting it wasn't still there.
                if ticket_id not in self.waiting:
                    self.waiting.append(ticket_id)

                edited_tickets.append(ticket_id)
            elif ticket_id in self.done:
                # Check if it's already done. GUI won't let this happen, but
                # cross-check.
                rospy.loginfo(f"{log_tag}: Attempted to edit a done ticket "
                              f"{ticket_id}. Ignoring")
            else:
                rospy.logwarn(f"{log_tag}: Attempted to edit a non-existent "
                              "ticket. Ignoring")

        rospy.loginfo(f"{log_tag}: Edited tickets {edited_tickets}.")

        # Request a new schedule with the updated list.
        self.request_schedule()

        # Update ticket and job statuses.
        self.update_jobs()
        self.update_ticket_statuses()
        self.update_job_statuses()

        self.announce_ticket_list_update()

    def delete_ticket_message_callback(self, msg):
        '''Callback when a delete_ticket message is received.

        Deletes the tickets and all their children to avoid dangling branches.
        TODO: Currently unused. May never be. I just didn't want to delete it.
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

        # Request a new schedule with the updated list.
        self.request_schedule()

    def delete_job_message_callback(self, msg):
        '''Callback when a delete_job message is received.
        
        The message just contains the job ID we want to delete.
        '''
        job_id = msg.job_id

        deleted_ticket_ids = []
        for ticket_id in self.jobs[job_id]["ticket_ids"]:
            self.delete_ticket(ticket_id)
            deleted_ticket_ids.append(ticket_id)

        rospy.loginfo(f"{log_tag}: Deleted job {job_id} with tickets "
                      f"{deleted_ticket_ids}."
        )

        # Request a new schedule with the updated list.
        self.request_schedule()

        # Update ticket and job statuses.
        self.update_jobs()
        self.update_ticket_statuses()
        self.update_job_statuses()

        self.announce_ticket_list_update()

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
        self.ongoing.append(ticket_id)
        self.ongoing_start_times[ticket_id] = rospy.Time.now().to_sec()

        # Remove the ticket from the ready set.
        self.ready.remove(ticket_id)

        # Set the ongoing timer.
        self.set_ongoing_timer()

        # Announce that the ticket has started?
        self.announce_ticket_start(ticket_id)

        rospy.loginfo(f"{log_tag}: Ticket {ticket_id} set to ongoing.")

        self.request_schedule()

        # Update ticket and job statuses.
        self.update_jobs
        self.update_ticket_statuses()
        self.update_job_statuses()

        self.announce_ticket_list_update()

        # # Add the ticket to the executed schedule.
        # self.add_started_ticket_to_schedule(ticket_id, self.ticket_dict[ticket_id])

    def end_ticket_message_callback(self, msg):
        '''Callback when a done signal is received.'''
        try:
            # Update the ongoing time left for all ongoing tix.
            self.update_ongoing_time_left()

            # End the ticket and get how much time it had left.
            ticket_time_left = self.end_ticket(msg.ticket_id)

            # Update ready and set timer.
            self.update_ready()
            self.set_ongoing_timer()

            # Update ticket and job statuses.
            self.update_jobs()
            self.update_ticket_statuses()
            self.update_job_statuses()

            self.announce_ticket_list_update()

            # Add the done ticket to executed schedule?
            # self.add_done_ticket_to_schedule(msg.ticket_id)

            # If the ticket had more than 5 minutes left, we re-schedule.
            # Otherwise, we let it run down.
            if ticket_time_left >= 5:
                self.request_schedule()

        except KeyError as e:
            rospy.logerr(f"Error ending ticket with ID {msg.ticket_id}.")
            rospy.logerr(e)

    def send_ticket_lists(self, request):
        '''Returns the complete list of tickets and the different subsets.'''
        rospy.logdebug(f"{log_tag}: Returning current ticket information.")

        # Update ticket and job statuses in case anything has changed.
        self.update_jobs()
        self.update_ticket_statuses()
        self.update_job_statuses()

        # Merge ticket_dict and done into one for the list of all tickets.
        # The two are only separate for the sake of generating schedules.
        all_tickets = {**self.ticket_dict, **self.done}
        all_tickets_list = convert_task_dict_to_ticket_list(all_tickets)

        return TicketListResponse(
            all_tickets_list,
            self.waiting,
            self.ready,
            self.ongoing,
            self.done.keys()
        )

    def request_schedule(self):
        '''Requests a new schedule from the schedule service.

        The request sends the whole ticket list and the ongoing tasks, so the
        scheduler knows which machine assignments to respect.        
        '''
        rospy.wait_for_service('schedule_service')
        try:
            request = ScheduleRequest()
            request.tickets = convert_task_dict_to_ticket_list(self.ticket_dict)

            # Create ongoing dictionary from list with dictionary
            # comprehension. How cool?? Who knew - dict comprehensions!
            request.ongoing = convert_task_dict_to_ticket_list({
                id: self.ticket_dict[id] for id in self.ongoing
            })

            schedule = rospy.ServiceProxy('schedule_service', Schedule)
            response = schedule(request)

            # Update the ticket list and subsets with the received schedule.
            self.on_schedule_update(response.tickets)

        except rospy.ServiceException as e:
            rospy.logerr(f"{log_tag}: Schedule request failed: {e}.")

    def on_schedule_update(self, tickets: Tickets):
        '''Updates the ticket list based on a new schedule.'''
        updated_task_dict = convert_ticket_list_to_task_dict(tickets)

        for ticket_id, old_ticket in self.ticket_dict.items():
            self.update_tickets_from_schedule(
                old_ticket,
                updated_task_dict[ticket_id]
            )

        self.update_ready()

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

    def convert_ticket_list_to_temp_dict(self, ticket_list) -> dict:
        '''Creates a temporary dictionary of tickets using the Tickets list.

        We need this temporary dictionary before the TM adds job ID and other
        important information it is responsible for maintaining. Only used
        when the ticket list comes from the supervisor GUI.

        Args:
            ticket_list: list of Ticket messages to parse. Each ticket from the
                        Supervisor GUI will only have machine_type, duration,
                        parents, and num_robots (if it's a top-level ticket).
        Returns:
            temp_dict: dictionary of tickets without job ID information.
        '''
        temp_dict = {}
        for ticket in ticket_list:
            tix = {}
            tix["machine_type"] = ticket.machine_type
            tix["duration"] = ticket.duration
            tix["parents"] = list(ticket.parents)
            tix["ticket_id"] = ticket.ticket_id

            # TODO: Add num_robots section.
            if len(tix["parents"]) == 0:
                tix["num_robots"] = ticket.num_robots

            # Time left in seconds.
            tix["time_left"] = ticket.duration
            temp_dict[ticket.ticket_id] = tix
        return temp_dict

    def get_received_ticket_job_ids(self, temp_dict: dict) -> dict:
        '''Gets the job IDs for the tickets received.

        Figures out the job IDs for the tickets received. They are either
        related to an existing job, or brand new.

        Args:
            temp_dict: temp dictionary of tickets that need job IDs added.
        Returns:
            temp_dict: the original dictionary with job IDs.
        '''
        # Step 1. Iterate through the temp dictionary and search for each
        # ticket's parents in the main ticket list or done.
        # If any parent is in either, add that parent's job_id to the ticket.
        # If they're not in the main list, the parents are probably the temp
        # dictionary. Save the ticket in a list.
        # If there are no parents, create a new job_id for the ticket.
        # Step 2. Go through the no_found_parents list and search for each
        # ticket's parents in the temp dictionary.
        # Go as high as possible in the job tree, get the top job_id,
        # then set it to the ticket.

        # Step 1.
        no_found_parents = []
        for ticket_id, ticket in temp_dict.items():
            if len(ticket["parents"]) == 0:
                self.minJobID += 1
                ticket["job_id"] = self.minJobID
                continue
            for parent_id in ticket["parents"]:
                if parent_id in self.ticket_dict:
                    ticket["job_id"] = self.ticket_dict[parent_id]["job_id"]
                    continue
                elif parent_id in self.done:
                    ticket["job_id"] = self.done[parent_id]["job_id"]
                    continue
            no_found_parents.append(ticket_id)

        # Step 2. Iterate through the tickets with no found parents.
        # For each, look for the parents in the dictionary recursively.
        # When we get to the top, set the job_id to the ticket.
        for ticket_id in no_found_parents:
            ticket = temp_dict[ticket_id]

            linear_job = [ticket_id]
            get_all_parents_from_task_list(ticket_id, temp_dict, linear_job)

            # Set the job_id for the ticket.
            top_ticket = temp_dict[linear_job[-1]]
            ticket["job_id"] = top_ticket["job_id"]

        return temp_dict

    def add_received_tickets_to_ticket_dict(self, received_tickets: dict) -> None:
        '''Adds the received tickets dictionary to the main ticket list.

        Args:
            received_tickets: dictionary of received tickets with job IDs
                added.
        '''
        for ticket_id, ticket in received_tickets.items():
            # Add the ticket to ticket_dict and waiting set.
            self.ticket_dict[ticket_id] = ticket
            self.waiting.append(ticket_id)

    def delete_ticket(self, ticket_id):
        '''Removes ticket_id from all possible sets and puts it in deleted.'''

        # Delete the ticket from ticket_dict and any set it might be in.
        if ticket_id in self.ticket_dict:
            # Add the ticket to deleted first and set time_left to 0.
            self.deleted[ticket_id] = self.ticket_dict[ticket_id]
            self.deleted[ticket_id]["time_left"] = 0
            del(self.ticket_dict[ticket_id])

            # Remove it from the subset it is in.
            if ticket_id in self.waiting:
                self.waiting.remove(ticket_id)
            elif ticket_id in self.ready:
                self.ready.remove(ticket_id)
            elif ticket_id in self.ongoing:
                self.ongoing.remove(ticket_id)

        # If the ticket is already done, remove it from there.
        elif ticket_id in self.done:
            self.deleted[ticket_id] = self.done[ticket_id]
            self.deleted[ticket_id]["time_left"] = 0
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
        self.ongoing.remove(ticket_id)
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

    def update_time_left(self, event):
        '''Updates ongoing set's time left. Keeps the time up to date for the GUI.'''
        # TODO: Thread safety. Doing this means there's a possibility that the
        # function is called from multiple threads.
        self.update_ongoing_time_left()
        self.set_ongoing_timer()

    def update_ready(self):
        '''Moves tickets whose parents are done from waiting to ready.'''
        ready_ticket_ids = []
        for ticket_id in self.waiting:
            ticket = self.ticket_dict[ticket_id]
            num_parents = len(ticket["parents"])
            num_done_parents = 0
            for parent in ticket["parents"]:
                if parent in self.done:
                    num_done_parents += 1
            if num_done_parents == num_parents:
                self.ready.append(ticket_id)
                ready_ticket_ids.append(ticket_id)

        # Remove the newly ready tickets from waiting.
        for ticket_id in ready_ticket_ids:
            self.waiting.remove(ticket_id)

        # Update job and ticket statuses.
        self.update_jobs()
        self.update_ticket_statuses()
        self.update_job_statuses()

    def update_ongoing_time_left(self):
        '''Updates the time left on all ongoing tickets.'''
        # Get the time that has passed since ongoing timer was last started.
        ongoing_time_elapsed = rospy.Time.now().to_sec() -\
                                self.ongoing_timer_set_time
        
        # print(f"Ongoing time elapsed: {ongoing_time_elapsed}")
        # Subtract the elapsed time from all ongoing tickets' time_left.
        # TODO: Probable source of the negative values being printed.
        for ticket_id in self.ongoing:
            self.ticket_dict[ticket_id]["time_left"] =\
                  max(self.ticket_dict[ticket_id]["time_left"] -\
                       ongoing_time_elapsed, 0.0)
            # print(f"Ticket {ticket_id} time left: "
            #       f"{self.ticket_dict[ticket_id]['time_left']}")
        return ongoing_time_elapsed

    def set_ongoing_timer(self):
        '''Sets the timer for the shortest time left on an ongoing ticket.'''
        # Find the lowest time_left ticket in ongoing.
        lowest_time_left = 50000
        for ticket_id in self.ongoing:
            ticket = self.ticket_dict[ticket_id]
            if ticket["time_left"] <= lowest_time_left:
                lowest_time_left = ticket["time_left"]
                self.ongoing_timer_id = ticket_id
        self.lowest_time_left = lowest_time_left

        # print(f"Lowest time left: {self.lowest_time_left}")
        # print(f"Ongoing timer ID: {self.ongoing_timer_id}")

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
        # Update ongoing time left.        
        self.update_ongoing_time_left()

        # Add time to triggering ticket.
        rospy.loginfo(f"{log_tag}: Adding time to ticket {self.ongoing_timer_id}.")
        rospy.loginfo(f"{log_tag}: Old time left: "
                    f"{self.ticket_dict[self.ongoing_timer_id]['time_left']}.")
        self.add_time_to_ticket(self.ongoing_timer_id)
        rospy.loginfo(f"{log_tag}: New time left: "
                    f"{self.ticket_dict[self.ongoing_timer_id]['time_left']}.")

        # Request a new schedule.
        rospy.loginfo(f"{log_tag}: Requesting new schedule.")
        self.request_schedule()

        self.set_ongoing_timer()

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

    def update_jobs(self):
        '''Updates the jobs data structure.

        Called whenever tickets are added, edited, or deleted.
        '''
        # Tickets that were added or edited will be in ticket_dict.
        for ticket_id, ticket in self.ticket_dict.items():
            job_id = ticket["job_id"]
            if job_id not in self.jobs:
                self.jobs[job_id] = {}
                self.jobs[job_id]["ticket_ids"] = [ticket_id]
                self.jobs[job_id]["status"] = "Unfinished"
            elif ticket_id not in self.jobs[job_id]["ticket_ids"]:
                self.jobs[job_id]["ticket_ids"].append(ticket_id)

        # If a job was deleted, delete its ID from jobs.
        # TODO: Currently, single tickets won't be deleted, just entire jobs.
        for ticket_id, ticket in self.deleted.items():
            job_id = ticket["job_id"]
            if job_id in self.jobs:
                del(self.jobs[job_id])

    def update_job_statuses(self):
        '''Updates the statuses on the jobs.
        
        Does so by looking at the status of the job's final ticket.
        Jobs are either Unfinished or Finished.
        '''
        for _, job_dict in self.jobs.items():
            # Dictionary of all tickets including done tickets.
            all_tickets = {**self.ticket_dict, **self.done}
            job_status = get_job_last_ticket_status(
                [all_tickets[ticket_id] for ticket_id in job_dict["ticket_ids"]],
                all_tickets
            )

            job_dict["status"] = job_status

    def update_ticket_statuses(self):
        '''Updates the statuses of the tickets.

        Finds which set they are in and sets their status accordingly.
        '''
        for ticket_id, ticket in self.ticket_dict.items():
            if ticket_id in self.waiting:
                ticket["status"] = "Waiting"
            elif ticket_id in self.ready:
                ticket["status"] = "Ready"
            elif ticket_id in self.ongoing:
                ticket["status"] = "Ongoing"

        for ticket_id, ticket in self.done.items():
            ticket["status"] = "Done"

    def announce_ticket_list_update(self):
        '''Publishes the new ticket list whenever called.
        
        Currently used by the robot_assigner.
        '''
        msg = String()
        self.ticket_list_update_pub.publish(msg)