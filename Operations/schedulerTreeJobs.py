'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Implementation of the flexible job shop problem using
    Google OR-Tools Linear Solver.

    This script handles tree jobs.

    Paper:
        Mathematical models for job-shop scheduling problems with routing
        and process plan flexibility - Ozguven et al (2010).
'''
import time
from ortools.linear_solver import pywraplp

from constants import *
from utils.draw_utils import draw_tree_schedule
from utils.display_utils import *
from utils.job_utils import *
from utils.sched_utils import *
from utils.solver_utils import *


# Start timing the overall runtime.
overallTime = time.time()

# Job data.
jobs_data = tree_jobs
# jobs_data = physical_demo_jobs

# # Print out the job and station information.
# display_job_data(jobs_data)
# display_station_numbers(station_type_names, Mj)

# Get the indices of each task's parents in the job list.
# This is done now to ease lookup later.
parent_ids = get_task_parent_indices(jobs_data)
# for i in range(len(parent_ids)):
#     print(parent_ids[i])

# Maximum horizon if all jobs and tasks were done in sequence.
horizon = sum(task["duration"] for job in jobs_data for task in job)


# Create the mip solver with the SCIP backend.
solver = pywraplp.Solver.CreateSolver('SCIP')
if not solver:
    exit()

# Create optimization variables, and define the constraints.
X, Y, Z, S, C, S_job, C_job, C_max = create_opt_variables(solver, jobs_data, horizon, all_machines, Mj)
define_constraints(solver, X, Y, Z, S, C, S_job, C_job, C_max, jobs_data, parent_ids, Mj)

# Define the objective function to minimize the makespan, then
# display some solver information.
solver.Minimize(C_max)
display_solver_information(solver)


# Invoke the solver.
solutionStart = time.time()
solver.SetTimeLimit(4000)

status = solver.Solve()
solutionEnd = time.time()

# Display the solution.
display_solution_stats(solver, status, horizon, solutionEnd-solutionStart)

# Extract the schedule.
schedule = extract_schedule(X, S, C, jobs_data, all_machines, station_type_names)
print(f"Overall runtime: {time.time() - overallTime: .3f} seconds.")
print()
# print(schedule)

for i in range(len(jobs_data)):
    print(S_job[i].solution_value())

# Save the schedule and draw it.
# schedule.to_csv(f"Plans/fjsspTree.csv")
# schedule.to_csv(f"Plans/fjsspTree1.csv")
# schedule.to_csv(f"Plans/fjsspTreePhysicalDemo1.csv")
draw_tree_schedule(schedule)

# # # Create robot timelines and save.
# robot_timelines = create_robot_timelines(schedule)
# robot_timelines.insert(0, 't', robot_timelines.index)
# robot_timelines['t'] = robot_timelines['t'].apply(lambda x: x*60)
# for m in range(11, 15):
#     robot_timelines[f"robot_{m}"] = ["WS_0_0" for _ in range(len(robot_timelines))]
# robot_timelines.to_csv("Plans/robotTimelinesPhysicalDemo1.csv")