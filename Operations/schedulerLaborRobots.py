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
# jobs_data = tree_jobs
jobs_data = physical_demo_jobs
num_jobs = len(jobs_data)

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

# Constraints for assignment.
numRobots = 10
robotReqs = [4, 3, 5, 3]
all_robots = range(numRobots)
print(robotReqs[0])

# Create variables.
R = {}
Sr = {}
Cr = {}
V = {}
L = 1000

# [210, 120, , 210]

for job in range(num_jobs):
    for robot in all_robots:
        R[job, robot] = solver.IntVar(0, 1, f'R{job}{robot}')
        Sr[job, robot] = solver.IntVar(0, horizon, '')
        Cr[job, robot] = solver.IntVar(0, horizon, '')

for job_b in range(1, num_jobs):
    for job_a in range(job_b):
        for robot in all_robots:
            V[job_a, job_b, robot] = solver.IntVar(0, 1, '')


# Each job gets as many robots as it needs.
for job_id in range(num_jobs):
    solver.Add(
        sum(R[job_id, robot] for robot in all_robots) == robotReqs[job_id]
    )

for job_id in range(num_jobs):
    for robot in all_robots:
        solver.Add(
            Sr[job_id, robot] + Cr[job_id, robot] <=
            R[job_id, robot]*L
        )
        # solver.Add(
        #     Cr[job_id, robot] >=
        #     Sr[job_id, robot] + 
        # )

for job_b in range(1, num_jobs):
    for job_a in range(job_b):
        for robot in all_robots:
            solver.Add(
                Sr[job_a, robot] >=
                Cr[job_b, robot] -
                V[job_a, job_b, robot]*L
            )
            solver.Add(
                Sr[job_b, robot] >=
                Cr[job_a, robot] -
                (1-V[job_a, job_b, robot])*L
            )

for job_id in range(num_jobs):
    solver.Add(
        (1/robotReqs[job_id])*sum(Sr[job_id, robot] for robot in all_robots) ==
        S_job[job_id]
    )
    solver.Add(
        (1/robotReqs[job_id])*sum(Cr[job_id, robot] for robot in all_robots) ==
        C_job[job_id]
    )





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
    print(f"S_{i}: {S_job[i].solution_value()}\t"
    f"Sr_{i}: {(1/robotReqs[i])*sum(Sr[i, robot].solution_value() for robot in all_robots)}\t"
    f"Cr_{i}: {(1/robotReqs[i])*sum(Sr[i, robot].solution_value() for robot in all_robots)}")

# Save the schedule and draw it.
# schedule.to_csv(f"Plans/fjsspTree.csv")
# schedule.to_csv(f"Plans/fjsspTree1.csv")
# schedule.to_csv(f"Plans/fjsspTreePhysicalDemo1.csv")
draw_tree_schedule(schedule)