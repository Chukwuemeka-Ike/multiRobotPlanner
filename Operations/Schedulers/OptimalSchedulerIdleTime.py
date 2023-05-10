'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Implementation of the flexible job shop problem using
    Google OR-Tools Linear Solver.

    This script runs a second optimization to minimize idle time.

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
from utils.solver_utils import *


# Start timing the overall runtime.
overallTime = time.time()

# Job data.
# jobs_data = tree_jobs
jobs_data = fifo_jobs
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
print()


# Invoke the solver.
solutionStart = time.time()
solver.SetTimeLimit(20000)
status = solver.Solve()
solutionEnd = time.time()

# Display the initial solution.
display_solution_stats(solver, status, horizon, solutionEnd-solutionStart)

# Extract the schedule.
schedule = extract_schedule(X, S, C, jobs_data, all_machines, station_type_names)
# print()
# print(schedule)

# Save the schedule and draw it.
schedule.to_csv(f"Plans/idleTimeMinInitialTree.csv")
draw_tree_schedule(schedule, "Images/idleTimeMinInitialTree.png")

# Initial total idle time.
initialIdleTimes = get_total_idle_time(schedule, jobs_data, parent_ids)
print(f"Initial total idle time: {initialIdleTimes: .2f} minutes.")
print()






# Idle time minimization.

# Create the solver.
idleSolver = pywraplp.Solver.CreateSolver('SCIP')
if not idleSolver:
    exit()

# Create optimization variables, and define the constraints.
X, Y, Z, S, C, S_job, C_job, C_max = create_opt_variables(idleSolver, jobs_data, horizon, all_machines, Mj)
define_constraints(idleSolver, X, Y, Z, S, C, S_job, C_job, C_max, jobs_data, parent_ids, Mj)
# add_no_c_inflation(idleSolver, S, C, jobs_data, Mj)

# Define objective to minimize the idle times.
optimum = solver.Objective().Value()
create_idle_time_objective(idleSolver, S, C, C_max, jobs_data, parent_ids, Mj, optimum)

display_solver_information(idleSolver)
print()

# Invoke the solver.
solutionStart = time.time()
idleSolver.SetTimeLimit(20000)
status = idleSolver.Solve()
solutionEnd = time.time()

# Display the initial solution.
print(f"New makespan: {C_max.solution_value()}")
display_solution_stats(idleSolver, status, horizon, solutionEnd-solutionStart)

# Extract the schedule.
schedule = extract_schedule(X, S, C, jobs_data, all_machines, station_type_names)
print(f"Overall runtime: {time.time() - overallTime: .2f} seconds.")

# Final total idle time.
finalIdleTimes = get_total_idle_time(schedule, jobs_data, parent_ids)
print(f"Final total idle time: {finalIdleTimes: .2f} minutes.")

# print()
# print(schedule)

# Save the schedule and draw it.
schedule.to_csv(f"Plans/idleTimeMinFinalTree.csv")
draw_tree_schedule(schedule, "Images/idleTimeMinFinalTree.png")