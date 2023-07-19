#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
'''
import rospy
from ortools.sat.python import cp_model

from arm_msgs.msg import Tickets, Ticket
from arm_msgs.srv import Schedule, ScheduleResponse

from arm_constants.machines import all_machines, machine_type_names, Mj
from arm_utils.display_utils import display_solution_stats_cpsat
from arm_utils.job_utils import get_task_parent_indices,\
    convert_task_list_to_job_list
from arm_utils.data_utils import create_ticket_list, convert_schedule_to_task_list, convert_ticket_list_to_task_dict
from arm_utils.sched_utils import extract_schedule_cpsat
from arm_utils.solver_utils_cpsat import create_opt_variables, define_constraints,\
    respect_ongoing_constraints


log_tag = "Task Scheduler"


class TaskScheduler():
    '''.'''

    def __init__(self) -> None:
        '''.'''
        rospy.init_node('task_scheduler')
        rospy.on_shutdown(self.shutdown_task_scheduler)
        rospy.loginfo(f"{log_tag}: Node started.")
        self.sched_service = rospy.Service(
            'schedule_service', Schedule, self.send_schedule
        )
        self.schedule_num = 0
        self.schedule_times = []
        
        rospy.spin()

    def generate_schedule(self, task_list: dict, ongoing: dict):
        '''Generates a schedule from the given task_list.'''
        # display_task_list(task_list)

        # Convert the task_list to job_list.
        job_list = convert_task_list_to_job_list(task_list)

        # Get the indices of each task's parents in the job list.
        # This is done now to ease lookup later.
        parent_ids = get_task_parent_indices(job_list)

        # Maximum horizon if all jobs and tasks were done in sequence.
        horizon = sum(
            task["duration"] for job in job_list for task in job
        )

        # Declare the model for the problem.
        model = cp_model.CpModel()

        # Create optimization variables, and define the constraints.
        X, Y, Z, S, C, S_job, C_job, C_max = create_opt_variables(
            model, job_list, all_machines, Mj
        )
        define_constraints(
            model, X, Y, Z, S, C, S_job, C_job, C_max, job_list, parent_ids, Mj
        )
        respect_ongoing_constraints(
            model, X, S, job_list, ongoing
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
                solver, X, S, C, job_list,
                all_machines,
                machine_type_names
            )
            self.schedule.to_csv(f"schedule{self.schedule_num}.csv")
            self.schedule_times.append(rospy.Time.now().to_sec())
            self.schedule_num += 1
        else:
            pass
            # display_task_list(task_list)

    def send_schedule(self, request):
        '''.'''
        rospy.loginfo(f"{log_tag}: Generating a schedule.")
        task_list = convert_ticket_list_to_task_dict(request.tickets)
        ongoing = convert_ticket_list_to_task_dict(request.ongoing)
        self.generate_schedule(task_list, ongoing)
        task_dict = convert_schedule_to_task_list(self.schedule)
        ticket_list = create_ticket_list(task_dict)
        return ScheduleResponse(ticket_list)

    def shutdown_task_scheduler(self):
        '''Gracefully shutdown task scheduler.'''
        # Save all generated schedules
        '''Saves the actual executed schedule for future reference.'''
        # with open('sched_times.txt', 'w') as f:
        #     for time in self.schedule_times:
        #         f.write(f"{time}\n")
        # self.executed_schedule.to_csv(f"savedSched.csv")
        # print(self.executed_schedule)
        rospy.loginfo(f"{log_tag}: Node shutdown.")