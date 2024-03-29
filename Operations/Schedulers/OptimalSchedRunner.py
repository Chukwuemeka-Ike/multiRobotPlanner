'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Runs the CP-SAT solver a specified number of times. Allows us
    to quickly rerun a solver and see how it performs multiple instances.
'''
import math
import time

from ortools.sat.python import cp_model

from constants.jobs import *
from constants.stations import *
# from constants.jobs_no_overlap import *
# from constants.stations_no_overlap import *
from utils.draw_utils import draw_tree_schedule
from utils.display_utils import *
from utils.job_utils import *
from utils.sched_utils import *
from utils.solver_utils_cpsat import *


# Start timing the overall runtime.
overallTime = time.time()

# Job data.
# job_list = tree_jobs
# job_list = fifo_jobs
# job_list = physical_demo_jobs
job_list = anchor_jobs
# job_list = convert_task_list_to_job_list(complete_ticket_list)
for job in job_list:
    for task in job:
        task["time_left"] = math.ceil(task["duration"])

# # Print out the job and station information.
# display_job_data(job_list)
# display_station_numbers(station_type_names, Mj)

# Get the indices of each task's parents in the job list.
# This is done now to ease lookup later.
parent_ids = get_task_parent_indices(job_list)
# for i in range(len(parent_ids)):
#     print(parent_ids[i])

# Maximum horizon if all jobs and tasks were done in sequence.
horizon = math.ceil(sum(task["duration"] for job in job_list for task in job))
print(f"Max Schedule Length: {horizon: .2f}")


def create_and_solve_program():
    '''Creates a new model and solves the optimization problem.'''
    # Declare the model for the problem.
    model = cp_model.CpModel()

    # Create optimization variables, and define the constraints.
    X, Y, Z, S, C, S_job, C_job, C_max = create_opt_variables(model, job_list, all_machines, Mj)
    define_constraints(model, X, Y, Z, S, C, S_job, C_job, C_max, job_list, parent_ids, Mj)

    # Define the objective function to minimize the makespan, then
    # display some solver information.
    model.Minimize(C_max)
    # display_solver_information(solver)
    # print()

    # Create the solver and solve.
    # solutionStart = time.time()
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 10.0
    status = solver.Solve(model)
    # solutionEnd = time.time()

    # Display the solution.
    display_solution_stats_cpsat(solver, status)

if __name__ == '__main__':
    for i in range(30):
        create_and_solve_program()