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
        
        self.ticket_dict = complete_ticket_list
        self.ongoing = {}
        self.ongoing_start_times = {}
        self.waiting = self.ticket_dict
        self.ready = {}
        self.done = {}

        self.job_list = [] # job_list
        # self.job_list = job_list

        self.ongoing_timer = None
        self.ongoing_timer_id = 0
        self.ongoing_timer_set_time = 0
        self.lowest_time_left = 0
        self.ready_timer = None
        self.ready_timer_id = 0

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
        # print(self.ticket_dict)

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
        self.parse_schedule()
        # display_task_list(self.ticket_dict)
        # display_task_list(self.waiting)
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
        '''.'''
        ticket_list = []
        for ticket_id, ticket in self.ticket_dict.items():
            msg = Ticket()
            msg.ticket_id = ticket_id
            msg.job_id = self.ticket_dict[ticket_id]["job_id"]
            msg.machine_type = self.ticket_dict[ticket_id]["station_type"]
            msg.duration = self.ticket_dict[ticket_id]["duration"]
            msg.parents = self.ticket_dict[ticket_id]["parents"]
            msg.start = self.ticket_dict[ticket_id]["start"]
            msg.end = self.ticket_dict[ticket_id]["end"]
            msg.station_num = self.ticket_dict[ticket_id]["station_num"]

            ticket_list.append(msg)
        return ticket_list


    def send_schedule(self, req):
        '''.'''
        rospy.loginfo("Returning the schedule.")
        ticket_list = self.create_ticket_list()
        return ScheduleResponse(ticket_list)

if __name__ == '__main__':
    TPT = TrajectoryPlannerTester()
    rospy.spin()