#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
'''

import pandas as pd
import rospy
import time
from ortools.linear_solver import pywraplp

from schedule_monitor_msgs.msg import Ticket, Tickets

from constants.stations import all_machines, station_type_names, Mj
from utils.display_utils import display_solution_stats, display_solver_information,\
    display_task_list
from utils.draw_utils import draw_tree_schedule
from utils.job_utils import get_task_parent_indices,\
    convert_task_list_to_job_list
from utils.sched_utils import extract_schedule
from utils.solver_utils import create_opt_variables, define_constraints,\
    respect_ongoing_constraints


class ScheduleMonitor():
    '''.'''


    def __init__(self):
        '''.'''
        rospy.init_node('schedule_monitor')
        rospy.on_shutdown(self.save_schedule_times)

        self.schedule = None
        self.schedule_timer = None
        self.schedule_times = []
        self.job_list = [] # job_list
        # self.job_list = job_list
        
        self.task_list = {}
        self.waiting = {}
        self.ready = {}
        self.ongoing = {}
        self.ongoing_start_times = {}
        self.done = {}

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
        self.saved_schedule = pd.DataFrame(
            columns=[
                "job_id", "task_id", "ticket_id","parents", "station_num",
                "station_type", "location", "start", "end", "duration",
                "time_left"
        ])

    def save_schedule_times(self):
        '''.'''
        with open('sched_times.txt', 'w') as f:
            # f.write('\n'.join(self.schedule_times))
            for time in self.schedule_times:
                f.write(f"{time}\n")
        self.saved_schedule.to_csv(f"savedSched.csv")

    def generate_schedule(self):
        '''Generates.'''
        # Create the mip solver with the SCIP backend.
        solver = pywraplp.Solver.CreateSolver('SCIP')
        if not solver:
            exit()

        # Convert the current task_list to job_list.
        self.job_list = convert_task_list_to_job_list(self.task_list)
        # print(self.task_list)

        # Get the indices of each task's parents in the job list.
        # This is done now to ease lookup later.
        parent_ids = get_task_parent_indices(self.job_list)

        # Maximum horizon if all jobs and tasks were done in sequence.
        horizon = sum(
            task["duration"] for job in self.job_list for task in job
        )
        # Create optimization variables, and define the constraints.
        X, Y, Z, S, C, S_job, C_job, C_max = create_opt_variables(
            solver, self.job_list, horizon, self.all_machines, self.Mj
        )
        define_constraints(
            solver, X, Y, Z, S, C, S_job, C_job, C_max,
            self.job_list, parent_ids, self.Mj
        )
        respect_ongoing_constraints(
            solver, X, S, self.job_list, self.ongoing
        )

        # Define the objective function to minimize the makespan, then
        # display some solver information.
        solver.Minimize(C_max)
        # display_solver_information(solver)
        # print()

        # Invoke the solver.
        solutionStart = time.time()
        solver.SetTimeLimit(20000)
        status = solver.Solve()
        solutionEnd = time.time()

        # Display the initial solution.
        display_solution_stats(solver, status, horizon, solutionEnd-solutionStart)

        # Extract the schedule.
        self.schedule = extract_schedule(X, S, C, self.job_list,
                    self.all_machines,
                    self.station_type_names
        )
        # draw_tree_schedule(self.schedule, f"schedule{self.schedule_num}.png")
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
        for ticket_id, ticket in self.waiting.items():
            self.update_ticket_from_schedule(ticket_id, ticket)

        for ticket_id, ticket in self.ready.items():
            self.update_ticket_from_schedule(ticket_id, ticket)

        for ticket_id, ticket in self.ongoing.items():
            self.update_ticket_from_schedule(ticket_id, ticket)

    def update_ticket_from_schedule(self, ticket_id, ticket):
        '''.'''
        row = self.schedule.loc[self.schedule["ticket_id"] == ticket_id]
        ticket["start"] = row["start"].item()
        ticket["end"] = row["end"].item()
        ticket["time_left"] = row["time_left"].item()
        ticket["station_num"] = row["station_num"].item()

        # TODO: This currently updates task list with the schedule info. Best?
        self.task_list[ticket_id] = ticket






    def add_tickets_to_task_list(self, ticket_list):
        '''Adds the list of tickets to the task_list and waiting.'''
        for ticket in ticket_list:
            # Create the ticket dictionary.
            tix = {}
            tix["job_id"] = ticket.job_id
            tix["parents"] = list(ticket.parents) #["parents"]
            tix["station_type"] = ticket.machine_type #["station_type"]
            tix["duration"] = ticket.duration #["duration"]

            # Time left in seconds.
            tix["time_left"] = ticket.duration#*60 #["duration"]*60

            # Add the ticket to task_list and waiting set.
            self.task_list[ticket.ticket_id] = tix #["ticket_id"]] = tix
            self.waiting[ticket.ticket_id] = tix #["ticket_id"]] = tix

    def add_ticket_message_callback(self, msg):
        '''Adds the tickets received to the task_list.'''
        self.add_tickets_to_task_list(msg.tickets)

        # Only update ongoing_timer if ongoing has tickets.
        if len(self.ongoing) != 0:
            self.update_ongoing_time_left()
            self.set_ongoing_timer()

        self.generate_schedule()
        self.on_schedule_update()

    def set_ticket_ready(self, ticket_id):
        '''Moves ticket from waiting to ready.'''
        self.ready[ticket_id] = self.waiting[ticket_id]
        del(self.waiting[ticket_id])

    def start_ticket(self, ticket_id):
        '''Moves ticket_id from ready to ongoing.'''
        self.ongoing[ticket_id] = self.ready[ticket_id]
        self.ongoing_start_times[ticket_id] = rospy.Time.now().to_sec()
        self.announce_ticket_start(ticket_id)
        self.add_started_ticket_to_schedule(ticket_id, self.ready[ticket_id])
        del(self.ready[ticket_id])

    def announce_ticket_start(self, ticket_id):
        '''Announces that we're starting the ticket.'''
        msg = Ticket()
        msg.ticket_id = ticket_id
        msg.job_id = self.ongoing[ticket_id]["job_id"]
        msg.machine_type = self.ongoing[ticket_id]["station_type"]
        msg.duration = self.ongoing[ticket_id]["duration"]
        msg.parents = self.ongoing[ticket_id]["parents"]
        self.ticket_started_pub.publish(msg)
        rospy.loginfo(f"Monitor: Starting ticket {ticket_id}.")

    def end_ticket(self, ticket_id):
        '''Moves ticket_id from ongoing to done.'''
        self.done[ticket_id] = self.ongoing[ticket_id]
        self.done[ticket_id]["time_left"] = 0

        # Delete the ticket from ongoing and task_list.
        del(self.ongoing[ticket_id])
        del(self.task_list[ticket_id])
        rospy.loginfo(f"Monitor: Ended ticket {ticket_id}.")

    def add_time_to_ticket(self, ticket_id):
        '''Adds 25% of original duration to time left on an ongoing ticket.'''
        self.ongoing[ticket_id]["time_left"] = 0.25*\
                            self.ongoing[ticket_id]["duration"]
        self.task_list[ticket_id]["time_left"] = \
                            self.ongoing[ticket_id]["time_left"]

    def update_ready(self):
        '''Moves tickets whose parents are done from waiting to ready.'''
        added_ticket_ids = []
        for ticket_id, ticket in self.waiting.items():
            num_parents = len(ticket["parents"])
            num_done_parents = 0
            for parent in ticket["parents"]:
                if parent in self.done.keys():
                    num_done_parents += 1
            if num_done_parents == num_parents:
                self.ready[ticket_id] = ticket
                added_ticket_ids.append(ticket_id)

        # Remove the newly ready tickets from waiting.
        for ticket in added_ticket_ids:
            del(self.waiting[ticket])

    # TODO: Change this to use schedule start times.
    def set_ready_timer(self, old_timer_id=None):
        '''Sets the timer for the earliest start in ready.'''
        if len(self.ready) != 0:
            # Find the earliest start time in ready.
            earliest_start = 5000
            for ticket_id, ticket in self.ready.items():
                if ticket["start"] <= earliest_start:
                    earliest_start = ticket["start"]
                    self.ready_timer_id = ticket_id

            # How long since the schedule was created.
            if old_timer_id != None:
                time_to_earliest_start = self.ready[self.ready_timer_id]["start"] \
                                            - self.ongoing[old_timer_id]["start"]
            else:
                time_to_earliest_start = self.ready[self.ready_timer_id]["start"]

            # Start a new timer with the earliest start time.
            if self.ready_timer is not None:
                self.ready_timer.shutdown()
            self.ready_timer = rospy.Timer(
                rospy.Duration(time_to_earliest_start+0.5),#60*
                self.on_ready_timer_trigger, 
                oneshot=True
            )

    def on_ready_timer_trigger(self, event):
        '''When the ready_timer.'''
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
        for ticket_id, ticket in self.ongoing.items():
            if ticket["time_left"] <= lowest_time_left:
                lowest_time_left = ticket["time_left"]
                self.ongoing_timer_id = ticket_id
        self.lowest_time_left = lowest_time_left

        # Start a new timer with the lowest time left.
        if self.ongoing_timer is not None:
            self.ongoing_timer.shutdown()
        self.ongoing_timer = rospy.Timer(
            rospy.Duration(lowest_time_left+0.5),
            self.on_ongoing_timer_trigger,
            oneshot=True
        )
        self.ongoing_timer_set_time = rospy.Time.now().to_sec()

    def on_ongoing_timer_trigger(self, event):
        '''Updates the ongoing set when ongoing_timer is triggered.'''
        # Add time to triggering ticket.
        rospy.loginfo(f"Monitor: Adding time to ticket {self.ongoing_timer_id}.")
        rospy.loginfo(f"Monitor: Old time left: {self.ongoing[self.ongoing_timer_id]['time_left']}.")
        self.add_time_to_ticket(self.ongoing_timer_id)
        rospy.loginfo(f"New Monitor: time left: {self.ongoing[self.ongoing_timer_id]['time_left']}.")

        # Generate a new schedule.
        rospy.loginfo(f"Monitor: Generating new schedule.")
        self.generate_schedule()
        self.on_schedule_update()

        # # Update ongoing time left and set a new ongoing timer.
        # self.update_ongoing_time_left()
        # self.set_ongoing_timer()

    def on_done_callback(self, msg):
        '''Callback when a done signal is received.'''
        try:
            # End the ticket, update ready and set timers.
            self.end_ticket(msg.ticket_id)
            self.add_done_ticket_to_schedule(msg.ticket_id)
            self.update_ready()
            self.set_ready_timer()
            ongoing_time_left = self.update_ongoing_time_left()
            self.set_ongoing_timer()
            rospy.loginfo("Monitor: "
                          f"Waiting tickets: {self.waiting.keys()}. "
                          f"Ready tickets: {self.ready.keys()}. "
                          f"Ongoing tickets: {self.ongoing.keys()}."
                          f"Done tickets: {self.done.keys()}."
            )

            # If the timer has more than 5 minutes left, we re-schedule.
            # Otherwise, we let it run down.
            # if ongoing_time_left >= 60*5:
            if ongoing_time_left >= 5:
                # Generate a new schedule.
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
        ticket_row["task_id"] = [len(
            self.saved_schedule.loc[self.saved_schedule["job_id"] == ticket["job_id"]]
        )]
        ticket_row["ticket_id"] = [ticket_id]
        ticket_row["parents"] = [(ticket["parents"])]
        ticket_row["station_num"] = [ticket["station_num"]]
        ticket_row["station_type"] = [ticket["station_type"]]
        ticket_row["location"] = [self.station_type_names[ticket["station_type"]]]
        ticket_row["start"] = rospy.Time.now().to_sec()
        ticket_row["end"] = None
        
        # self.schedule = self.schedule.append(ticket_row, ignore_index=True)
        # print(self.schedule)
        # print(ticket_row)
        self.saved_schedule = pd.concat(
            [self.saved_schedule,
             pd.DataFrame(ticket_row, index=[0])
            ],
            ignore_index=True
        )

    def add_done_ticket_to_schedule(self, ticket_id):
        '''.'''
        # ticket_row = self.saved_schedule.loc[
        #     self.saved_schedule["ticket_id"] == ticket_id
        # ]
        # ticket_row["end"] = rospy.Time.now().to_sec()
        self.saved_schedule.loc[
            self.saved_schedule["ticket_id"] == ticket_id, "end"
        ] = rospy.Time.now().to_sec()

    def update_ongoing_time_left(self):
        '''Updates the time left on all ongoing tickets.'''
        # Get the time that has passed since ongoing timer was last started.
        ongoing_time_elapsed = rospy.Time.now().to_sec() -\
                                self.ongoing_timer_set_time
        ongoing_time_left = self.lowest_time_left - ongoing_time_elapsed
        # ongoing_time_left = self.ongoing_timer.remaining()

        # Subtract the elapsed time from all ongoing tickets' time_left.
        for ticket_id, ticket in self.ongoing.items():
            ticket["time_left"] -= ongoing_time_elapsed
            self.task_list[ticket_id]["time_left"] -= ongoing_time_elapsed

        return ongoing_time_left


if __name__ == '__main__':
    schedMon = ScheduleMonitor()
    rospy.spin()