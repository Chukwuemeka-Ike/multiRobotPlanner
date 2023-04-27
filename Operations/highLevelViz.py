'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Creates mock robot timelines for a previously generated schedule.
    Useful for the high level visualization.
'''
import json
import pandas as pd
import time

from constants import *
from utils.display_utils import *
from utils.draw_utils import draw_tree_schedule
from utils.job_utils import *
from utils.sched_utils import *
from utils.solver_utils import *



# # Load schedule from file instead of optimizing.
# schedule = pd.read_csv("Plans/fjsspTreeIdleTimeMinFinalPresentation.csv", index_col=0)

# # Lists get converted to strings when saved. This converts the strings back.
# schedule["Parents"] = schedule["Parents"].apply(lambda x:json.loads(x))

# # Ensure start and end are integers.
# schedule["Start"] = schedule["Start"].apply(lambda x:int(x))
# schedule["End"] = schedule["End"].apply(lambda x:int(x))


# Run optimization.

# Job data.
# jobs_data = tree_jobs
jobs_data = physical_demo_jobs

# # Print out the job and station information.
# display_job_data(jobs_data)
# display_station_numbers(station_type_names, Mj)

# Get the indices of each task's parents in the job list.
parent_ids = get_task_parent_indices(jobs_data)

# Maximum horizon if all jobs and tasks were done in sequence.
horizon = sum(task["duration"] for job in jobs_data for task in job)



# Initial schedule optimization.
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
schedule.to_csv(f"Plans/highLevelVizScheduleInitial15.csv")
# timelines.to_csv("Plans/highLevelVizRobotTimelines.csv")
draw_tree_schedule(schedule, "Images/highLevelVizScheduleInitial15.png")

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

# Invoke the solver.
solutionStart = time.time()
# idleSolver.SetTimeLimit(2000)
status = idleSolver.Solve()
solutionEnd = time.time()

# Display the initial solution.
print(f"New makespan: {C_max.solution_value()}")
display_solution_stats(idleSolver, status, horizon, solutionEnd-solutionStart)

# Extract the schedule.
schedule = extract_schedule(X, S, C, jobs_data, all_machines, station_type_names)

# Final total idle time.
finalIdleTimes = get_total_idle_time(schedule, jobs_data, parent_ids)
print(f"Final total idle time: {finalIdleTimes: .2f} minutes.")



# Create the timelines for the robots.
timelines = create_robot_timelines(schedule)

# Add time column for Burak's parser and convert to seconds.
timelines.insert(0, 't', timelines.index)
timelines['t'] = timelines['t'].apply(lambda x: x*60)

# Save the generated timelines and the schedule, and draw the schedule.
schedule.to_csv(f"Plans/highLevelVizSchedule15.csv")
timelines.to_csv("Plans/highLevelVizRobotTimelines15.csv")
draw_tree_schedule(schedule, "Images/highLevelVizSchedule15.png")