#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Tests the trajectoryPlanner node by providing a schedule over a service.
    It generates a single schedule at the beginning and ends those tickets
    based on that schedule. It generates new schedules at certain times
    to test the reaction of the planner to that scenario.
'''

import pandas as pd
import rospy
import time
from ortools.linear_solver import pywraplp

from std_msgs.msg import String
from schedule_monitor_msgs.msg import Tickets, Ticket
from schedule_monitor_msgs.srv import Schedule, ScheduleResponse

from constants.jobs import complete_ticket_list
from constants.stations import all_machines, station_type_names, Mj

from utils.display_utils import display_solution_stats, display_solver_information,\
    display_task_list
from utils.job_utils import get_task_parent_indices,\
    convert_task_list_to_job_list
from utils.sched_utils import extract_schedule
from utils.solver_utils import create_opt_variables, define_constraints,\
    respect_ongoing_constraints


class TrajectoryPlannerTester:
    '''.'''

    def __init__(self):
        '''.'''
        rospy.init_node('trajectory_planner_tester')
        
        self.job_list = []
        self.ticket_dict = complete_ticket_list.copy()
        self.waiting = complete_ticket_list.copy()
        self.ready = {}
        self.ongoing = {}
        self.ongoing_start_times = {}
        self.done = {}

        self.ready_timer = None
        self.ready_timer_id = 0
        self.ongoing_timer = None
        self.ongoing_timer_id = 0
        self.ongoing_timer_set_time = 0
        self.lowest_time_left = 0

        self.all_machines = all_machines
        self.station_type_names = station_type_names
        self.Mj = Mj

        self.sched_service = rospy.Service(
            'schedule_service', Schedule, self.send_schedule
        )
        self.end_ticket_pub = rospy.Publisher(
            "end_ticket", Ticket, queue_size=100
        )
        self.new_schedule_pub = rospy.Publisher(
            "new_schedule", String, queue_size=100
        )
        self.ticket_started_pub = rospy.Publisher(
            "ticket_started", Ticket, queue_size=100
        )

        self.schedule_timer = None
        self.schedule_times = []
        self.schedule = self.generate_schedule()

    def generate_schedule(self):
        '''.'''
        # Create the mip solver with the SCIP backend.
        solver = pywraplp.Solver.CreateSolver('SCIP')
        if not solver:
            exit()

        # Convert the current ticket_dict to job_list.
        self.job_list = convert_task_list_to_job_list(self.ticket_dict)

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

        print(self.schedule)
        
        # Update the ticket dictionaries with the schedule and update timers.
        self.parse_schedule()
        self.update_ready()
        self.update_ongoing_time_left()
        self.set_ongoing_timer()
        self.set_ready_timer()

        # Announce that a new schedule was published.
        msg = String()
        msg.data = "New"
        self.new_schedule_pub.publish(msg)

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
        self.ticket_dict[ticket_id] = ticket

    def create_ticket_list(self):
        '''Creates a list of Ticket messages .'''
        ticket_list = []
        for ticket_id, ticket in self.ticket_dict.items():
            msg = Ticket()
            msg.ticket_id = ticket_id
            msg.job_id = ticket["job_id"]
            msg.machine_type = ticket["station_type"]
            msg.duration = ticket["duration"]
            msg.parents = ticket["parents"]
            msg.start = ticket["start"]
            msg.end = ticket["end"]
            msg.station_num = ticket["station_num"]

            ticket_list.append(msg)
        return ticket_list

    def set_ticket_ready(self, ticket_id):
        '''Moves ticket from waiting to ready.'''
        self.ready[ticket_id] = self.waiting[ticket_id]
        del(self.waiting[ticket_id])

    def start_ticket(self, ticket_id):
        '''Moves ticket_id from ready to ongoing.'''
        self.ongoing[ticket_id] = self.ready[ticket_id]
        self.ongoing_start_times[ticket_id] = rospy.Time.now().to_sec()
        self.announce_ticket_start(ticket_id)
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

        # Delete the ticket from ongoing and ticket_dict.
        del(self.ongoing[ticket_id])
        del(self.ticket_dict[ticket_id])
        rospy.loginfo(f"Monitor: Ended ticket {ticket_id}.")

        msg = Ticket()
        msg.ticket_id = ticket_id
        msg.job_id = self.done[ticket_id]["job_id"]
        msg.machine_type = self.done[ticket_id]["station_type"]
        msg.duration = self.done[ticket_id]["duration"]
        msg.parents = self.done[ticket_id]["parents"]
        msg.station_num = self.done[ticket_id]["station_num"]
        self.end_ticket_pub.publish(msg)

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
        '''Ends the ticket when ongoing_timer is triggered.'''
        try:
            # End the ticket, update ready and set timers.
            self.end_ticket(self.ongoing_timer_id)
            self.update_ready()
            self.set_ready_timer()
            self.set_ongoing_timer()
            rospy.loginfo("Monitor: "
                            f"Waiting tickets: {self.waiting.keys()}. "
                            f"Ready tickets: {self.ready.keys()}. "
                            f"Ongoing tickets: {self.ongoing.keys()}."
                            f"Done tickets: {self.done.keys()}."
            )
        except KeyError as e:
            rospy.logerr(
                "Error ending ticket with ID {}.".format(self.ongoing_timer_id)
            )
            rospy.logerr(e)

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
            self.ticket_dict[ticket_id]["time_left"] -= ongoing_time_elapsed

        return ongoing_time_left



    def send_schedule(self, req):
        '''.'''
        rospy.loginfo("Returning the schedule.")
        ticket_list = self.create_ticket_list()
        return ScheduleResponse(ticket_list)

if __name__ == '__main__':
    TPT = TrajectoryPlannerTester()
    rospy.spin()