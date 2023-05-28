#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
'''

import math
import pandas as pd
import rospy
from ortools.sat.python import cp_model

from schedule_monitor_msgs.msg import Ticket, Tickets

from constants.stations import all_machines, station_type_names, Mj
from utils.display_utils import display_solution_stats_cpsat,\
    display_task_list
from utils.draw_utils import draw_tree_schedule
from utils.job_utils import get_task_parent_indices,\
    convert_task_list_to_job_list
from utils.sched_utils import extract_schedule_cpsat
from utils.solver_utils_cpsat import create_opt_variables, define_constraints,\
    respect_ongoing_constraints


class ScheduleMonitor():
    '''.'''


    def __init__(self):
        '''.'''
        rospy.init_node('schedule_monitor')
        rospy.on_shutdown(self.save_executed_schedule)
        rospy.loginfo("Schedule Monitor: Node started.")

        self.schedule = None
        self.schedule_times = []
        
        self.job_list = []
        self.task_list = {}
        self.waiting = {}
        self.ready = {}
        self.ongoing = {}
        self.done = {}
        self.ongoing_start_times = {}

        self.ongoing_timer = None
        self.ongoing_timer_id = 0
        self.ongoing_timer_set_time = 0
        self.lowest_time_left = 0
        self.ready_timer = None
        self.ready_timer_id = 0

        self.all_machines = all_machines
        self.station_type_names = station_type_names
        self.Mj = Mj

        self.end_ticket_sub = rospy.Subscriber(
            "end_ticket", Ticket, self.on_done_callback
        )
        self.add_ticket_sub = rospy.Subscriber(
            "add_ticket", Tickets, self.add_ticket_message_callback
        )
        self.ticket_started_pub = rospy.Publisher(
            "ticket_started", Ticket, queue_size=100
        )
        self.schedule_num = 1
        self.executed_schedule = pd.DataFrame(
            columns=[
                "job_id", "task_idx", "ticket_id","parents", "station_num",
                "station_type", "location", "start", "end", "duration",
                "time_left"
        ])

    def save_executed_schedule(self):
        '''Saves the actual executed schedule for future reference.'''
        with open('sched_times.txt', 'w') as f:
            for time in self.schedule_times:
                f.write(f"{time}\n")
        self.executed_schedule.to_csv(f"savedSched.csv")
        print(self.executed_schedule)
        rospy.loginfo("Schedule Monitor: Schedule Monitor node shutdown.")

    def generate_schedule(self):
        '''Generates a.'''
        # Ensure the time_left values are all integers for the CP-SAT solver to work.
        for _, ticket in self.task_list.items():
            ticket["time_left"] = math.ceil(ticket["time_left"])
            display_task_list(self.task_list)
        # Convert the current task_list to job_list.
        self.job_list = convert_task_list_to_job_list(self.task_list)

        # Get the indices of each task's parents in the job list.
        # This is done now to ease lookup later.
        parent_ids = get_task_parent_indices(self.job_list)

        # Maximum horizon if all jobs and tasks were done in sequence.
        horizon = sum(
            task["duration"] for job in self.job_list for task in job
        )

        # Declare the model for the problem.
        model = cp_model.CpModel()

        # Create optimization variables, and define the constraints.
        X, Y, Z, S, C, S_job, C_job, C_max = create_opt_variables(
            model, self.job_list, all_machines, Mj
        )
        define_constraints(
            model, X, Y, Z, S, C, S_job, C_job, C_max, self.job_list, parent_ids, Mj
        )
        respect_ongoing_constraints(
            model, X, S, self.job_list, self.ongoing
        )

        # Define the objective function to minimize the makespan, then
        # display some solver information.
        model.Minimize(C_max)

        # Create the solver and solve.
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = 10.0
        status = solver.Solve(model)
        
        # Display the initial solution.
        display_solution_stats_cpsat(solver, status)

        if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
            # Extract the schedule.
            self.schedule = extract_schedule_cpsat(
                solver, X, S, C, self.job_list,
                self.all_machines,
                self.station_type_names
            )
            # draw_tree_schedule(self.schedule, f"schedule{self.schedule_num}.png")
            # print(self.schedule)
            self.schedule.to_csv(f"schedule{self.schedule_num}.csv")
            self.schedule_times.append(rospy.Time.now().to_sec())
            self.schedule_num += 1

    # TODO: Name this better.
    def on_schedule_update(self):
        '''.'''
        # display_task_list(self.task_list)
        self.parse_schedule()
        # display_task_list(self.task_list)
        self.update_ready()
        self.update_ongoing_time_left()
        self.set_ongoing_timer()
        self.set_ready_timer()

    def parse_schedule(self):
        '''Goes through each set and updates their station and times.'''
        for ticket_id, ticket in self.task_list.items():
            self.update_ticket_from_schedule(ticket_id, ticket)
        # for ticket_id, ticket in self.waiting.items():
        #     self.update_ticket_from_schedule(ticket_id, ticket)

        # for ticket_id, ticket in self.ready.items():
        #     self.update_ticket_from_schedule(ticket_id, ticket)

        # for ticket_id, ticket in self.ongoing.items():
        #     self.update_ticket_from_schedule(ticket_id, ticket)

    def update_ticket_from_schedule(self, ticket_id, ticket):
        '''.'''
        try:
            row = self.schedule.loc[self.schedule["ticket_id"] == ticket_id]
            ticket["start"] = row["start"].item()
            ticket["end"] = row["end"].item()
            ticket["time_left"] = row["time_left"].item()
            ticket["station_num"] = row["station_num"].item()

            # # TODO: This currently updates task list with the schedule info. Best?
            # self.task_list[ticket_id] = ticket
        except ValueError as e:
            rospy.logerr(e)
            print(ticket_id)
            print(self.schedule)







    def add_ticket_message_callback(self, msg):
        '''Adds the tickets received to the task_list.'''
        rospy.loginfo("Schedule Monitor: Received new set of tickets."
                      " Adding to task list.")
        self.add_tickets_to_task_list(msg.tickets)
        self.generate_schedule()
        self.on_schedule_update()

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
        self.add_started_ticket_to_schedule(ticket_id, self.task_list[ticket_id])
        del(self.ready[ticket_id])

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
        rospy.loginfo(f"Schedule Monitor: Starting ticket {ticket_id}.")

    def end_ticket(self, ticket_id):
        '''Moves ticket_id from ongoing to done.'''
        self.done[ticket_id] = self.task_list[ticket_id]
        self.done[ticket_id]["time_left"] = 0

        # Delete the ticket from ongoing and task_list.
        del(self.ongoing[ticket_id])
        del(self.task_list[ticket_id])
        rospy.loginfo(f"Schedule Monitor: Ended ticket {ticket_id}.")

    def add_time_to_ticket(self, ticket_id):
        '''Adds 25% of original duration to time left on an ongoing ticket.'''
        try:
            self.task_list[ticket_id]["time_left"] = math.ceil(0.25*\
                                self.task_list[ticket_id]["duration"])
            # self.ongoing[ticket_id]["time_left"] = math.ceil(0.25*\
            #                     self.task_list[ticket_id]["duration"])
        except KeyError as e:
            # If the ticket is not in here, it must have finished as scheduled.
            rospy.loginfo(f"ScheduleMonitor: Ticket {ticket_id} not in the"
                          " task list. Might have ended on time.")
            rospy.logwarn(e)

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

    # TODO: Change this to use schedule start times.
    def set_ready_timer(self, old_timer_id=None):
        '''Sets the timer for the earliest start in ready.'''
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
        '''Updates the ongoing set when ongoing_timer is triggered.'''
        # Add time to triggering ticket.
        rospy.loginfo(f"Schedule Monitor: Adding time to ticket {self.ongoing_timer_id}.")
        rospy.loginfo(f"Schedule Monitor: Old time left: {self.task_list[self.ongoing_timer_id]['time_left']}.")
        self.add_time_to_ticket(self.ongoing_timer_id)
        rospy.loginfo(f"Schedule Monitor: New time left: {self.task_list[self.ongoing_timer_id]['time_left']}.")

        # Generate a new schedule.
        rospy.loginfo(f"Schedule Monitor: Generating new schedule.")
        self.generate_schedule()
        self.on_schedule_update()

    def on_done_callback(self, msg):
        '''Callback when a done signal is received.'''
        try:
            # End the ticket, update ready and set timers.
            self.end_ticket(msg.ticket_id)
            self.update_ready()
            self.set_ready_timer()
            print("Set ready timer.")
            ongoing_time_left = self.update_ongoing_time_left()
            self.set_ongoing_timer()
            self.add_done_ticket_to_schedule(msg.ticket_id)
            rospy.loginfo("Schedule Monitor: "
                          f"Waiting tickets: {self.waiting.keys()}. "
                          f"Ready tickets: {self.ready.keys()}. "
                          f"Ongoing tickets: {self.ongoing.keys()}."
                          f"Done tickets: {self.done.keys()}.\n"
            )

            # If the timer has more than 5 minutes left, we re-schedule.
            # Otherwise, we let it run down.
            if ongoing_time_left >= 5:
                self.generate_schedule()
                self.on_schedule_update()
        except KeyError as e:
            rospy.logerr(
                "Error ending ticket with ID {}.".format(msg.ticket_id)
            )
            rospy.logerr(e)

    def add_started_ticket_to_schedule(self, ticket_id, ticket):
        '''Adds the ticket information to the schedule.'''
        ticket_row = {}
        ticket_row["job_id"] = [ticket["job_id"]]
        ticket_row["task_idx"] = [len(
            self.executed_schedule.loc[self.executed_schedule["job_id"] == ticket["job_id"]]
        )]
        ticket_row["ticket_id"] = [ticket_id]
        ticket_row["parents"] = [(ticket["parents"])]
        ticket_row["station_num"] = [ticket["station_num"]]
        ticket_row["station_type"] = [ticket["station_type"]]
        ticket_row["location"] = [self.station_type_names[ticket["station_type"]]]
        ticket_row["start"] = rospy.Time.now().to_sec()
        ticket_row["end"] = None
        
        # self.schedule = self.schedule.append(ticket_row, ignore_index=True)
        # print(self.executed_schedule)
        # print(ticket_row)
        self.executed_schedule = pd.concat(
            [self.executed_schedule,
             pd.DataFrame(ticket_row, index=[0])
            ],
            ignore_index=True
        )

    def add_done_ticket_to_schedule(self, ticket_id):
        '''.'''
        self.executed_schedule.loc[
            self.executed_schedule["ticket_id"] == ticket_id, "end"
        ] = rospy.Time.now().to_sec()

    def update_ongoing_time_left(self):
        '''Updates the time left on all ongoing tickets.'''
        # Get the time that has passed since ongoing timer was last started.
        ongoing_time_elapsed = rospy.Time.now().to_sec() -\
                                self.ongoing_timer_set_time
        ongoing_time_left = self.lowest_time_left - ongoing_time_elapsed
        # ongoing_time_left = self.ongoing_timer.remaining()

        # Subtract the elapsed time from all ongoing tickets' time_left.
        # TODO: Probable source of the negative values being printed.
        for ticket_id in self.ongoing.keys():
            self.task_list[ticket_id]["time_left"] -= ongoing_time_elapsed
        return ongoing_time_left


if __name__ == '__main__':
    schedMon = ScheduleMonitor()
    rospy.spin()
