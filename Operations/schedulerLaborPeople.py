import time
from ortools.linear_solver import pywraplp

from constants import *
from utils.draw_utils import draw_tree_schedule, draw_labor_schedule
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
numRobots = 5
robotReqs = [4, 3, 5, 3]
all_robots = range(numRobots)

# Create variables.
R = {}
Sr = {}
Cr = {}
V = {}
U = {}
L = 10000

for job in range(num_jobs):
    for task in range(len(jobs_data[job])):
        for robot in all_robots:
            R[job, task, robot] = solver.IntVar(0, 1, f'R{job}{task}{robot}')
            Sr[job, task, robot] = solver.Var(0, horizon, False, '')
            Cr[job, task, robot] = solver.Var(0, horizon, False, '')

for job_b in range(1, num_jobs):
    for job_a in range(job_b):
        for task_b in range(len(jobs_data[job_b])):
            for task_a in range(len(jobs_data[job_a])):
                for robot in all_robots:
                    V[job_a, task_a, job_b, task_b, robot] = solver.IntVar(0, 1, '')

for job_id, job in enumerate(jobs_data):
    combos = combinations(range(len(job)), 2)
    for combo in combos:
        for robot in all_robots:
            # Binary variable - 1 if (job_id, combo[0]) precedes
            # (job_id, combo[1]) on robot. 0, otherwise.
            U[job_id, combo[0], combo[1], robot] = solver.IntVar(0, 1, '')




# Constraints.
# Each task gets as much labor as it needs.
for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        solver.Add(
            sum(R[job_id, task_id, robot] for robot in all_robots) == task["num_robots"]
        )

# Task completion.
for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        for robot in all_robots:
            solver.Add(
                Sr[job_id, task_id, robot] + Cr[job_id, task_id, robot] <=
                R[job_id, task_id, robot]*L
            )

            solver.Add(
                Cr[job_id, task_id, robot] >=
                Sr[job_id, task_id, robot] + task["duration"] -
                (1 - R[job_id, task_id, robot])*L
            )

            solver.Add(
                Sr[job_id, task_id, robot] + task["duration"] >=
                Cr[job_id, task_id, robot]
            )

# Job-task job-task precedence on the same robot.
for job_b in range(1, num_jobs):
    for job_a in range(job_b):
        for task_b in range(len(jobs_data[job_b])):
            for task_a in range(len(jobs_data[job_a])):
                for robot in all_robots:
                    solver.Add(
                        Sr[job_a, task_a, robot] >=
                        Cr[job_b, task_b, robot] -
                        V[job_a, task_a, job_b, task_b, robot]*L
                    )
                    solver.Add(
                        Sr[job_b, task_b, robot] >=
                        Cr[job_a, task_a, robot] -
                        (1 - V[job_a, task_a, job_b, task_b, robot])*L
                    )

# Job tasks can't overlap on same robot.
for job_id, job in enumerate(jobs_data):
    combos = combinations(range(len(job)), 2)
    for combo in combos:
        for robot in all_robots:
            solver.Add(
                Sr[job_id, combo[0], robot] >=
                Cr[job_id, combo[1], robot] -
                    U[job_id, combo[0], combo[1], robot]*L
            )
            solver.Add(
                Sr[job_id, combo[1], robot] >=
                Cr[job_id, combo[0], robot] -
                    (1-U[job_id, combo[0], combo[1], robot])*L
            )

# All robots assigned to a task must start at the same time.
for job_id, job in enumerate(jobs_data):
    combos = combinations(all_robots, 2)
    for task_id, task in enumerate(job):
        for combo in combos:
            solver.Add(
                Sr[job_id, task_id, combo[0]] <=
                Sr[job_id, task_id, combo[1]] +
                (1 - R[job_id, task_id, combo[1]])*L
            )
            solver.Add(
                Sr[job_id, task_id, combo[1]] <=
                Sr[job_id, task_id, combo[0]] +
                (1 - R[job_id, task_id, combo[0]])*L
            )


# # Couple scheduling with the assignment.
# for job_id, job in enumerate(jobs_data):
#     for task_id, task in enumerate(job):
#         solver.Add(
#             (1/task["num_robots"])*sum(Sr[job_id, task_id, robot] for robot in all_robots) ==
#             sum(S[job_id, task_id, machine] for machine in all_machines)
#         )
#         solver.Add(
#             (1/task["num_robots"])*sum(Cr[job_id, task_id, robot] for robot in all_robots) ==
#             sum(C[job_id, task_id, machine] for machine in all_machines)
#         )







# Define the objective function to minimize the makespan, then
# display some solver information.
solver.Minimize(C_max)
display_solver_information(solver)


# # Invoke the solver.
# solutionStart = time.time()
# solver.SetTimeLimit(15000)

# status = solver.Solve()
# solutionEnd = time.time()

# # Display the solution.
# display_solution_stats(solver, status, horizon, solutionEnd-solutionStart)

# # Extract the schedule.
# schedule = extract_schedule(X, S, C, jobs_data, all_machines, station_type_names)
# print(f"Overall runtime: {time.time() - overallTime: .3f} seconds.")
# print()
# # print(schedule)

# job_id = 0
# task_id = 6
# robot = 1
# print(f"R_{job_id}_{task_id}_{robot}: {R[job_id, task_id, robot].solution_value()}\t"
#     f"S_{job_id}_{task_id}_{robot}: {Sr[job_id, task_id, robot].solution_value()}\t"
#     f"C_{job_id}_{task_id}_{robot}: {Cr[job_id, task_id, robot].solution_value()}\t"
#     f"")

# labor_schedule = extract_labor_schedule(R, Sr, Cr, jobs_data, all_robots)
# # # print(labor_schedule)
# labor_schedule.to_csv(f"Plans/laborSched.csv")





# # # for i in range(len(jobs_data)):
# # #     print(f"S_{i}: {S_job[i].solution_value()}\t"
# # #     f"Sr_{i}: {(1/robotReqs[i])*sum(Sr[i, robot].solution_value() for robot in all_robots)}\t"
# # #     f"Cr_{i}: {(1/robotReqs[i])*sum(Sr[i, robot].solution_value() for robot in all_robots)}")

# # # Save the schedule and draw it.
# # schedule.to_csv(f"Plans/fjsspTree.csv")
# # # schedule.to_csv(f"Plans/fjsspTree1.csv")
# # # schedule.to_csv(f"Plans/fjsspTreePhysicalDemo1.csv")
# # # draw_tree_schedule(schedule)
# # draw_labor_schedule(labor_schedule, list(all_robots))