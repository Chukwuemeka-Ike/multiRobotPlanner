'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Implementation of the flexible job shop problem using
    Google OR-Tools Linear Solver.

    Paper:
        Mathematical models for job-shop scheduling problems with routing
        and process plan flexibility - Ozguven et al (2010).
'''
import time
from ortools.linear_solver import pywraplp

from constants.jobs import *
from constants.stations import *
from utils.draw_utils import draw_tree_schedule
from utils.display_utils import *
from utils.job_utils import *
from utils.sched_utils import *
from utils.solver_utils_lp import *


# Start timing the overall runtime.
overallTime = time.time()

# Job data.
# job_list = tree_jobs
# job_list = fifo_jobs
job_list = physical_demo_jobs
# job_list = anchor_jobs
# job_list = convert_task_list_to_job_list(complete_ticket_list)
for job in job_list:
    for task in job:
        task["time_left"] = task["duration"]

# # Print out the job and station information.
# display_job_data(job_list)
# display_station_numbers(station_type_names, Mj)

# Get the indices of each task's parents in the job list.
# This is done now to ease lookup later.
parent_ids = get_task_parent_indices(job_list)
# for i in range(len(parent_ids)):
#     print(parent_ids[i])

# Maximum horizon if all jobs and tasks were done in sequence.
horizon = sum(task["duration"] for job in job_list for task in job)
print(f"Max Schedule Length: {horizon: .2f}")


# Create the mip solver with the SCIP backend.
solver = pywraplp.Solver.CreateSolver('SCIP')
if not solver:
    exit()

# Create optimization variables, and define the constraints.
X, Y, Z, S, C, S_job, C_job, C_max = create_opt_variables(solver, job_list, all_machines, Mj)
define_constraints(solver, X, Y, Z, S, C, S_job, C_job, C_max, job_list, parent_ids, Mj)

# Define the objective function to minimize the makespan, then
# display some solver information.
solver.Minimize(C_max)
display_solver_information(solver)
print()

# Invoke the solver.
solutionStart = time.time()
solver.SetTimeLimit(20000)
status = solver.Solve()
solutionEnd = time.time()

# Display the solution.
display_solution_stats_lp(solver, status, horizon, solutionEnd-solutionStart)

# Extract the schedule.
schedule = extract_schedule_lp(X, S, C, job_list, all_machines, station_type_names)
# print()
# print(schedule)
# print()

# Save the schedule and draw it.
schedule.to_csv(f"Plans/optimalScheduleLP.csv")
draw_tree_schedule(schedule, "Images/optimalScheduleLP.png")