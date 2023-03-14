'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Implementation of the flexible job shop problem using
    Google OR-Tools Linear Solver. 

    This script handles only linear jobs.

    This version assumes we have multiple machines for some operation types:
        Loading Area - 0
        Mega Stitch - [1, 2]
        RF - 3
        Perimeter - [4, 5, 6]
        Inspection - 7
    Paper:
        Mathematical models for job-shop scheduling problems with routing
        and process plan flexibility - Ozguven et al (2010).
'''
import pandas as pd
import time

from constants import *
from ortools.linear_solver import pywraplp
from utils.draw_utils import draw_schedule


def intersection(lst1, lst2):
    '''Returns the common items between two lists.'''
    return list(set(lst1) & set(lst2))


overallTime = time.time()
# Job data. Every job starts at the loading area.
jobs_data = linear_jobs
num_jobs = len(jobs_data)
print()
print("Jobs:")
print("[")
for job in jobs_data:
    print(f"    {job}")
print("]")

# Maximum horizon if all jobs were sequential.
horizon = sum(task[1] for job in jobs_data for task in job)



# Create the mip solver with the SCIP backend.
solver = pywraplp.Solver.CreateSolver('SCIP')

if not solver:
    exit()

# Variable creation.
X = {}
Y = {}
S = {}
C = {}
C_job = {}
t = {}
L = 1000

for job in range(len(jobs_data)):
    for task in range(len(jobs_data[job])):
        for machine in all_machines:
            X[job, task, machine] = solver.IntVar(0, 1, f'X{job}{task}{machine}')
            
            S[job, task, machine] = solver.IntVar(0, horizon, f'S{job}{task}{machine}')
            C[job, task, machine] = solver.IntVar(0, horizon, f'C{job}{task}{machine}')
            # t[job, task, machine] = solver.IntVar(0, horizon, f't{job}{task}{machine}')

for i in range(num_jobs):
    C_job[i] = solver.IntVar(0, horizon, 'Ci')


for job_b in range(1, len(jobs_data)):
    for job_a in range(job_b):
        for task_b in range(len(jobs_data[job_b])):
            for task_a in range(len(jobs_data[job_a])):
                M_intersection = intersection(
                    Mj[jobs_data[job_b][task_b][0]],
                    Mj[jobs_data[job_a][task_a][0]]
                )
                # print(M_intersection)
                for machine in M_intersection:
                    Y[job_a, task_a, job_b, task_b, machine] = solver.IntVar(
                        0, 1, f'Y{job_a}{task_a}{job_b}{task_b}{machine}'
                    )


# Constraint construction.
# Job-specific constraints.
for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        # Each operation can only be assigned to one machine.
        solver.Add( 
            sum(X[job_id, task_id, machine] for machine in Mj[task[0]]) == 1
        )

        # Within a job, each task must start after the previous task ends.
        if task_id > 0:
            solver.Add( 
                sum(S[job_id, task_id, machine] for machine in Mj[task[0]]) >=
                sum(C[job_id, task_id-1, machine] for machine in Mj[job[task_id-1][0]])
            )

# Task completion constraints.
for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        # Set constraints for all machines that match the task requirement.
        for machine in Mj[task[0]]:
            # The start and end time must equal zero if the task is not
            # assigned to that machine.
            solver.Add(
                S[job_id, task_id, machine] + C[job_id, task_id, machine] <=
                  X[job_id, task_id, machine]*L
            )

            # Task completion must happen after task duration + start time.
            solver.Add(
                C[job_id, task_id, machine] >=
                S[job_id, task_id, machine] + task[1] - 
                (1 - X[job_id, task_id, machine])*L
            )

# Precedence constraints. Iterate between every job-task-job-task pair to
# make sure that no two tasks are assigned to the same machine at the
# same time.
for job_b in range(1, len(jobs_data)):
    for job_a in range(job_b):
        for task_b in range(len(jobs_data[job_b])):
            for task_a in range(len(jobs_data[job_a])):
                # Check if the task-task pair have overlapping machines.
                M_intersection = intersection(
                    Mj[jobs_data[job_b][task_b][0]],
                    Mj[jobs_data[job_a][task_a][0]]
                )
                for machine in M_intersection:
                    solver.Add(
                        S[job_a, task_a, machine] >=
                            C[job_b, task_b, machine] - 
                            Y[job_a, task_a, job_b, task_b, machine]*L
                    )
                    solver.Add(
                        S[job_b, task_b, machine] >=
                            C[job_a, task_a, machine] - 
                            (1-Y[job_a, task_a, job_b, task_b, machine])*L
                    )


# Define objective.
C_max = solver.IntVar(0, horizon, 'makespan')
# solver.Minimize(C_max)

objective_terms = []
for job_id, job in enumerate(jobs_data):
    objective_terms.append(C_job[job_id])
    # Job's completion must be after the last task's completion time.
    solver.Add(
        C_job[job_id] >= 
            sum(C[job_id, len(job)-1, machine] for machine in Mj[job[-1][0]])
    )
    # # The overall makespan must be after the last job's completion. Not
    # # sure the usefulness.
    # solver.Add(
    #     C_max >= C_job[i]
    # )

# Minimize the sum of completion times.
solver.Minimize(solver.Sum(objective_terms))

# # Minimize the idle time between loading and the next operation.
# solver.Add(solver.Sum(objective_terms) <= 1615*1.1)
# idle_times = []
# for job_id, job in enumerate(jobs_data):
#     idle_times.append(
#         sum(S[job_id, 1, machine] for machine in Mj[job[1][0]])
#             - sum(C[job_id, 0, machine] for machine in Mj[job[0][0]])
#     )
# solver.Minimize(solver.Sum(idle_times))

# # Minimize all idle times between operations in a job.
# solver.Add(solver.Sum(objective_terms) <= 1615*1.1)
# idle_times = []
# for job_id, job in enumerate(jobs_data):
#     idle_times.append(
#         sum(
#             sum(S[job_id, task_id, machine] for machine in Mj[job[task_id][0]])
#             - sum(C[job_id, task_id, machine] for machine in Mj[job[task_id][0]])
#             for task_id in range(1, len(job))
#         )
#     )
# solver.Minimize(solver.Sum(idle_times))
    


print(f"Number of variables: {solver.NumVariables()}")
print(f"Number of constraints: {solver.NumConstraints()}")


# Invoke the solver.
solutionStart = time.time()
status = solver.Solve()
print("\n\n")
if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
    print(f"Max Schedule Length: {horizon: .1f}")
    print(f"Objective value: {solver.Objective().Value()}")
    print(
        "Optimal Schedule Length:",
        f"{max(obj_term.solution_value() for obj_term in objective_terms)}."
    )
    print(f"Solution Runtime: {time.time() - solutionStart: .3f} seconds.")
else:
    print("Infeasible program. Exiting.\n")
    exit()

# # Display the solution found.
# for job_id, job in enumerate(jobs_data):
#     print(f"Job {job_id} completion time: {C_job[job_id].solution_value()}")
#     for task_id, task in enumerate(job):
#         for machine in all_machines:
#             if X[job_id, task_id, machine].solution_value() > 0.5:
#                 print(f"Job {job_id} Task {task_id} assigned to machine {machine}.",
#                     f"Start time: {S[job_id, task_id, machine].solution_value()}.",
#                     f"Completion time: {C[job_id, task_id, machine].solution_value()}.")
#     print()


jobs, tasks = [], []
locations, station_nums, station_type_nums = [], [], []
starts, ends, durations = [], [], []

for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        for machine in all_machines:
            if X[job_id, task_id, machine].solution_value() > 0.5:
                jobs.append(job_id)
                tasks.append(task_id)
                locations.append(station_names[task[0]])
                station_nums.append(machine)
                station_type_nums.append(task[0])
                starts.append(S[job_id, task_id, machine].solution_value())
                ends.append(C[job_id, task_id, machine].solution_value())
                durations.append(task[1])

schedule = pd.DataFrame()
schedule["Job #"] = jobs
schedule["Task #"] = tasks
schedule["Location"] = locations
schedule["Station #"] = station_nums
schedule["Station Type #"] = station_type_nums
schedule["Start"] = starts
schedule["End"] = ends
schedule["Duration"] = durations

print(f"Overall runtime: {time.time() - overallTime: .3f} seconds.")
print()

print(schedule)
# schedule.to_csv(f"Plans/fjssp.csv")
# draw_schedule(schedule)