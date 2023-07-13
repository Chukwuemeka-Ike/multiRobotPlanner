#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
'''
import math
import rospy

from arm_msgs.msg import Ticket, Tickets
from arm_msgs.srv import Schedule, ScheduleRequest, TicketList, TicketListResponse

from arm_utils.job_utils import convert_ticket_list_to_task_dict,\
        create_ticket_list, get_tree_job_start_ids,\
        get_all_children_from_task_list, get_all_parents_from_task_list
from arm_utils.display_utils import display_task_list


log_tag = "Ticket Manager"

class TicketManager():
    '''Ticket manager class. Responsible for maintaining ticket statuses.'''

    def __init__(self) -> None:
        '''.'''
        rospy.init_node('ticket_manager')
        rospy.on_shutdown(self.shutdown_ticket_manager)
        rospy.loginfo(f"{log_tag}: Node started.")

        # Ticket list and subsets - waiting, ready, ongoing, done.
        self.task_list = {}
        self.waiting = {}
        self.ready = {}
        self.ongoing = {}
        self.done = {}
        self.minJobID = 0

        # Timer variables for triggering ticket starts and re-scheduling.
        self.ongoing_timer = None
        self.ongoing_timer_id = 0
        self.ongoing_timer_set_time = 0
        self.ongoing_start_times = {}
        self.lowest_time_left = 0
        self.ready_timer = None
        self.ready_timer_id = 0

        # Subscribers for ticket management. Add/edit/remove ticket/job.
        self.add_ticket_sub = rospy.Subscriber(
            "add_ticket", Tickets, self.add_ticket_message_callback
        )
        self.edit_ticket_sub = rospy.Subscriber(
            "edit_ticket", Ticket, self.edit_ticket_message_callback
        )
        self.edit_job_sub = rospy.Subscriber(
            "edit_job", Tickets, self.edit_job_message_callback
        )
        self.end_ticket_sub = rospy.Subscriber(
            "end_ticket", Ticket, self.on_done_callback
        )

        # Publisher.
        self.ticket_started_pub = rospy.Publisher(
            "ticket_started", Ticket, queue_size=100
        )

        # Service for ticket lists.
        self.ticket_service = rospy.Service(
            'ticket_service', TicketList, self.send_ticket_lists
        )
        # Spin.
        rospy.spin()

    def shutdown_ticket_manager(self):
        '''Gracefully shutdown ticket manager.'''
        # Save current task list?
        rospy.loginfo(f"{log_tag}: Node shutdown.")

    def send_ticket_lists(self, request):
        '''Returns the complete list of tickets and the different subsets.'''
        rospy.loginfo(f"{log_tag}: Returning current ticket information.")

        # Merge task_list and done into one for the list of all tickets.
        # The two are only separate for the sake of generating schedules.
        all_tickets = {**self.task_list, **self.done}
        all_tickets_list = create_ticket_list(all_tickets)

        waiting = create_ticket_list(self.waiting)
        ready = create_ticket_list(self.ready)
        ongoing = create_ticket_list(self.ongoing)
        done = create_ticket_list(self.done)
        return TicketListResponse(all_tickets_list, waiting, ready, ongoing, done)

    def request_schedule(self):
        '''Requests a new schedule from the schedule service.

        The request sends the whole task list and the ongoing tasks, so the
        scheduler knows which station assignments to respect.        
        '''
        rospy.wait_for_service('schedule_service')
        try:
            request = ScheduleRequest()
            request.tickets = create_ticket_list(self.task_list)
            request.ongoing = create_ticket_list(self.ongoing)

            schedule = rospy.ServiceProxy('schedule_service', Schedule)
            response = schedule(request)

            # Update the task list and its subsets with the received schedule.
            self.on_schedule_update(response.tickets)
        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Schedule request failed: {e}.')

    def on_schedule_update(self, tickets: Tickets):
        '''Updates the task list and timers based on a new schedule.'''
        updated_task_dict = convert_ticket_list_to_task_dict(tickets)

        for ticket_id, old_ticket in self.task_list.items():
            self.update_tickets_from_schedule(old_ticket, updated_task_dict[ticket_id])

        self.update_ready()
        self.update_ongoing_time_left()
        self.set_ongoing_timer()
        self.set_ready_timer()

    def update_tickets_from_schedule(self, old_ticket, new_ticket):
        '''Updates a ticket based on a new schedule.'''
        old_ticket["start"] = new_ticket["start"]
        old_ticket["end"] = new_ticket["end"]
        old_ticket["time_left"] = new_ticket["time_left"]
        old_ticket["station_num"] = new_ticket["station_num"]

    def add_ticket_message_callback(self, msg):
        '''Callback for new tickets received.

        Adds the received tickets to the task list and requests a new schedule.
        '''
        rospy.loginfo(f"{log_tag}: Received new set of tickets."
                        " Adding to task list.")
        # self.add_tickets_to_task_list(msg.tickets)
        temp_dict = self.convert_ticket_list_to_temp_dict(msg.tickets)
        temp_dict = self.get_received_ticket_job_ids(temp_dict)
        self.add_received_tickets_to_task_list(temp_dict)
        # display_task_list(self.task_list)
        self.request_schedule()

    def convert_ticket_list_to_temp_dict(self, ticket_list):
        '''.'''
        temp_dict = {}
        for ticket in ticket_list:
            tix = {}
            tix["station_type"] = ticket.machine_type
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
        # main task list. If they're not in there, save the key in a list.
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
                if parent_id in self.task_list:
                    ticket["job_id"] = self.task_list[parent_id]["job_id"]
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

    def add_received_tickets_to_task_list(self, ticket_dict: dict):
        '''Adds the modified received tickets dictionary to the main task list.'''
        for ticket_id, ticket in ticket_dict.items():
            ticket["time_left"] = ticket["duration"]

            # Add the ticket to task_list and waiting set.
            self.task_list[ticket_id] = ticket
            self.waiting[ticket_id] = self.task_list[ticket_id]

    def edit_ticket_message_callback(self):
        '''Callback when an edit_ticket message is received.'''

    def edit_job_message_callback(self):
        '''Callback when an edit_job message is received.'''

    def add_tickets_to_task_list(self, ticket_list):
        '''Adds the list of tickets to the task_list and waiting.'''
        for ticket in ticket_list:
            # Create the ticket dictionary.
            tix = {}
            tix["job_id"] = ticket.job_id
            tix["station_type"] = ticket.machine_type
            tix["duration"] = ticket.duration
            tix["parents"] = list(ticket.parents)

            # Time left in seconds.
            tix["time_left"] = ticket.duration

            # Add the ticket to task_list and waiting set.
            self.task_list[ticket.ticket_id] = tix
            self.waiting[ticket.ticket_id] = self.task_list[ticket.ticket_id]

    def set_ticket_ready(self, ticket_id):
        '''Moves ticket from waiting to ready.'''
        self.ready[ticket_id] = self.task_list[ticket_id]
        del(self.waiting[ticket_id])

    def start_ticket(self, ticket_id):
        '''Moves ticket_id from ready to ongoing.'''
        self.ongoing[ticket_id] = self.task_list[ticket_id]
        self.ongoing_start_times[ticket_id] = rospy.Time.now().to_sec()
        self.announce_ticket_start(ticket_id)
        # self.add_started_ticket_to_schedule(ticket_id, self.task_list[ticket_id])
        del(self.ready[ticket_id])

    def update_ready(self):
        '''Moves tickets whose parents are done from waiting to ready.'''
        added_ticket_ids = []
        for ticket_id in self.waiting.keys():
            ticket = self.task_list[ticket_id]
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

    def end_ticket(self, ticket_id):
        '''Moves ticket_id from ongoing to done.'''
        self.done[ticket_id] = self.task_list[ticket_id]
        self.done[ticket_id]["time_left"] = 0

        # Delete the ticket from ongoing and task_list.
        del(self.ongoing[ticket_id])
        del(self.task_list[ticket_id])
        rospy.loginfo(f"{log_tag}: Ended ticket {ticket_id}.")

    def announce_ticket_start(self, ticket_id):
        '''Announces that we're starting the ticket.'''
        msg = Ticket()
        msg.ticket_id = ticket_id
        msg.job_id = self.task_list[ticket_id]["job_id"]
        msg.machine_type = self.task_list[ticket_id]["station_type"]
        msg.duration = self.task_list[ticket_id]["duration"]
        msg.parents = self.task_list[ticket_id]["parents"]
        msg.start = self.task_list[ticket_id]["start"]
        msg.end = self.task_list[ticket_id]["end"]
        msg.station_num = self.task_list[ticket_id]["station_num"]
        self.ticket_started_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Starting ticket {ticket_id}.")

    def set_ready_timer(self, old_timer_id=None):
        '''Sets the timer for the earliest start in ready.'''
        # TODO: Change this to use schedule start times.
        if len(self.ready) != 0:
            # Find the earliest start time in ready.
            earliest_start = 5000
            for ticket_id in self.ready.keys():
                ticket = self.task_list[ticket_id]
                if ticket["start"] <= earliest_start:
                    earliest_start = ticket["start"]
                    self.ready_timer_id = ticket_id

            # How long since the schedule was created.
            if old_timer_id != None:
                time_to_earliest_start = self.task_list[self.ready_timer_id]["start"] \
                                            - self.task_list[old_timer_id]["start"]
            else:
                time_to_earliest_start = self.task_list[self.ready_timer_id]["start"]

            # Start a new timer with the earliest start time.
            if self.ready_timer is not None:
                self.ready_timer.shutdown()
            self.ready_timer = rospy.Timer(
                rospy.Duration(time_to_earliest_start+0.5), # Add 0.5 because 0 timers cause error.
                self.on_ready_timer_trigger, 
                oneshot=True
            )

    def on_ready_timer_trigger(self, event):
        '''When the ready_timer is triggered.'''
        # Subtract elapsed time from all ongoing tix.
        self.update_ongoing_time_left()

        # Set the ticket to ongoing.
        old_timer_id = self.ready_timer_id
        self.start_ticket(self.ready_timer_id)
        
        # Set new ready and ongoing timers.
        self.set_ready_timer(old_timer_id)
        self.set_ongoing_timer()

    def set_ongoing_timer(self):
        '''Sets the timer for the shortest time left on an ongoing ticket.'''
        # Find the lowest time_left ticket in ongoing.
        lowest_time_left = 50000
        for ticket_id in self.ongoing.keys():
            ticket = self.task_list[ticket_id]
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
        rospy.loginfo(f"{log_tag}: Old time left: {self.task_list[self.ongoing_timer_id]['time_left']}.")
        self.add_time_to_ticket(self.ongoing_timer_id)
        rospy.loginfo(f"{log_tag}: New time left: {self.task_list[self.ongoing_timer_id]['time_left']}.")

        # Request a new schedule.
        rospy.loginfo(f"{log_tag}: Requesting new schedule.")
        self.request_schedule()

    def add_time_to_ticket(self, ticket_id):
        '''Adds 25% of original duration to time left on the given ticket.'''
        try:
            self.task_list[ticket_id]["time_left"] = math.ceil(0.25*\
                                self.task_list[ticket_id]["duration"])
        except KeyError as e:
            # If the ticket is not in here, it must have finished as scheduled.
            rospy.loginfo(f"{log_tag}: Ticket {ticket_id} not in the"
                          " task list. Might have ended on time.")
            rospy.logwarn(e)

    def update_ongoing_time_left(self):
        '''Updates the time left on all ongoing tickets.'''
        # Get the time that has passed since ongoing timer was last started.
        ongoing_time_elapsed = rospy.Time.now().to_sec() -\
                                self.ongoing_timer_set_time
        ongoing_time_left = self.lowest_time_left - ongoing_time_elapsed

        # Subtract the elapsed time from all ongoing tickets' time_left.
        # TODO: Probable source of the negative values being printed.
        for ticket_id in self.ongoing.keys():
            self.task_list[ticket_id]["time_left"] -= ongoing_time_elapsed
        return ongoing_time_left

    def on_done_callback(self, msg):
        '''Callback when a done signal is received.'''
        try:
            # End the ticket, update ready and set timers.
            self.end_ticket(msg.ticket_id)
            self.update_ready()
            self.set_ready_timer()
            ongoing_time_left = self.update_ongoing_time_left()
            self.set_ongoing_timer()
            # self.add_done_ticket_to_schedule(msg.ticket_id)
            # rospy.loginfo("{log_tag}: "
            #               f"Waiting tickets: {self.waiting.keys()}. "
            #               f"Ready tickets: {self.ready.keys()}. "
            #               f"Ongoing tickets: {self.ongoing.keys()}."
            #               f"Done tickets: {self.done.keys()}.\n"
            # )

            # If the timer has more than 5 minutes left, we re-schedule.
            # Otherwise, we let it run down.
            if ongoing_time_left >= 5:
                self.request_schedule()
        except KeyError as e:
            rospy.logerr(
                "Error ending ticket with ID {}.".format(msg.ticket_id)
            )
            rospy.logerr(e)
