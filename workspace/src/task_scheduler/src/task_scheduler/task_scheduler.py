#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Defines the Task Scheduler class, which generates schedules whenever
    the Ticket Manager requests. An object is instantiated in the
    task_scheduler_node.
'''
import os
import rospy

from ortools.sat.python import cp_model

from arm_msgs.srv import MachinesOverview, MachinesOverviewRequest,\
    Schedule, ScheduleRequest, ScheduleResponse
from arm_utils.display_utils import display_solution_stats_cpsat
from arm_utils.job_utils import get_task_parent_indices
from arm_utils.conversion_utils import convert_task_dict_to_ticket_list,\
    convert_task_list_to_job_list, convert_schedule_to_task_list,\
    convert_ticket_list_to_task_dict, convert_list_of_int_lists_to_list_of_lists
from arm_utils.sched_utils import extract_schedule_cpsat
from arm_utils.solver_utils_cpsat import create_opt_variables,\
    define_constraints, respect_ongoing_constraints


log_tag = "Task Scheduler"


class TaskScheduler():
    '''Class for the Task Scheduler responsible for generating schedules
    whenever the Ticket Manager requests it.
    '''

    def __init__(self) -> None:
        '''.'''
        rospy.init_node('task_scheduler')
        rospy.on_shutdown(self.shutdown_task_scheduler)
        rospy.loginfo(f"{log_tag}: Node started.")
        rospy.loginfo(f"{log_tag}: Waiting for service from Machine Manager")

        # Path for saving schedules to.
        self.scheduler_log_dir = rospy.get_param("scheduler_log_dir", "~/.ros")

        # Service through which the Ticket Manager can request schedules.
        self.sched_service = rospy.Service(
            'schedule_service', Schedule, self.send_schedule
        )

        # Variables for tracking how many schedules have been generated
        # and the times each was generated.
        self.schedule_num = 0
        self.schedule_times = []

        rospy.spin()

    def generate_schedule(self, ticket_dict: dict, ongoing: dict):
        '''Generates a schedule from the given ticket_dict.'''
        # Convert the ticket_dict to job_list, which is the format
        # that the solver_utils functions expect it in.
        job_list = convert_task_list_to_job_list(ticket_dict)

        # Get the indices of each task's parents in the job list.
        # This is done now to ease lookup when building the constraints.
        parent_ids = get_task_parent_indices(job_list)

        # Declare the model for the problem.
        model = cp_model.CpModel()

        # Create optimization variables, and define the constraints.
        X, Y, Z, S, C, S_job, C_job, C_max = create_opt_variables(
            model, job_list, self.machine_ids, self.grouped_machine_ids
        )
        define_constraints(
            model, X, Y, Z, S, C, S_job, C_job, C_max, job_list, parent_ids,
            self.grouped_machine_ids
        )
        respect_ongoing_constraints(
            model, X, S, job_list, ongoing
        )

        # Define the objective function to minimize the makespan.
        model.Minimize(C_max)

        # Create the solver and solve the optimization problem.
        # Set a hard time limit of 10 seconds. So far, planning
        # for 12 jobs has never taken up to 10 seconds.
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = 10.0
        status = solver.Solve(model)

        # Display the solution information.
        display_solution_stats_cpsat(solver, status)

        if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
            # Extract the schedule.
            self.schedule = extract_schedule_cpsat(
                solver, X, S, C, job_list,
                self.machine_ids,
                self.machine_type_names
            )
            filename = f"schedule{self.schedule_num}.csv"
            self.schedule.to_csv(
                os.path.join(self.scheduler_log_dir, filename)
            )
            self.schedule_times.append(rospy.Time.now().to_sec())
            self.schedule_num += 1
        else:
            rospy.logwarn(f"{log_tag}: Failed to generate a schedule.")

    def send_schedule(self, request: ScheduleRequest) -> ScheduleResponse:
        '''Called when a schedule is requested by the Ticket Manager.
        
        Generates a schedule, converts it to a dictionary, then sends
        it to back to the client.
        '''
        rospy.loginfo(f"{log_tag}: Generating a schedule.")

        # Convert the ongoing and all ticket lists to dictionaries.
        ticket_dict = convert_ticket_list_to_task_dict(request.tickets)
        ongoing = convert_ticket_list_to_task_dict(request.ongoing)

        # Request machine information before trying to schedule.
        self.request_machine_overview()

        # Generate the schedule.
        self.generate_schedule(ticket_dict, ongoing)

        # Convert the generated schedule back to a dictionary then
        # ticket list, then send it back.
        task_dict = convert_schedule_to_task_list(self.schedule)
        ticket_list = convert_task_dict_to_ticket_list(task_dict)

        return ScheduleResponse(ticket_list)

    def request_machine_overview(self):
        '''.'''
        rospy.wait_for_service('machine_overview_service', timeout=10)
        try:
            request = MachinesOverviewRequest()
            machines_overview = rospy.ServiceProxy(
                'machine_overview_service', MachinesOverview
            )
            response = machines_overview(request)
            self.machine_ids = response.machine_ids
            grouped_machine_ids = response.grouped_machine_ids
            self.grouped_machine_ids = \
                convert_list_of_int_lists_to_list_of_lists(grouped_machine_ids)
            self.machine_type_names = response.machine_type_names

        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Ticket list request failed: {e}.')

    def shutdown_task_scheduler(self):
        '''Gracefully shutdown the task scheduler.'''
        rospy.loginfo(f"{log_tag}: Node shutdown.")
