'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Implementation of the flexible job shop problem using
    Google OR-Tools Linear Solver.

    This script handles tree jobs.

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
from itertools import combinations
from ortools.linear_solver import pywraplp
from utils.draw_utils import draw_schedule


def intersection(lst1, lst2):
    '''Returns the common items between two lists.'''
    return list(set(lst1) & set(lst2))

overallTime = time.time()
# Job data. Every job starts at the loading area.
# task_template = {"ticket_id": , "station_type": , "duration": , "parents": []}
jobs_data = tree_jobs
num_jobs = len(jobs_data)
# print()
# print("Jobs:")
# print("[")
# for job in jobs_data:
#     for task in job:
#         print("    ", [f"{k}: {v}" for k, v in task.items()])
# print("]")
print(f"Number of Jobs: {num_jobs}.")

# Get the positions of the task parents in the job list. This is done now to
# ease lookup later.
# TODO: Streamline.
parent_ids = []
for job in jobs_data:
    for task_id in range(len(job)):
        parents = []
        for parent in job[task_id]["parents"]:
            for task_id in range(len(job)):
                if job[task_id]["ticket_id"] == parent:
                    parents.append(task_id)
        parent_ids.append(parents)

# for i in range(len(parent_ids)):
#     print(parent_ids[i])


# Maximum horizon if all jobs were sequential.
horizon = sum(task["duration"] for job in jobs_data for task in job)
# print(horizon)



# Create the mip solver with the SCIP backend.
solver = pywraplp.Solver.CreateSolver('SCIP')

if not solver:
    exit()


# Variable creation.
X = {}
Y = {}
Z = {}
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

for i in range(num_jobs):
    C_job[i] = solver.IntVar(0, horizon, 'Ci')

for job_b in range(1, len(jobs_data)):
    for job_a in range(job_b):
        for task_b in range(len(jobs_data[job_b])):
            for task_a in range(len(jobs_data[job_a])):
                M_intersection = intersection(
                    Mj[jobs_data[job_b][task_b]["station_type"]],
                    Mj[jobs_data[job_a][task_a]["station_type"]]
                )
                # print(M_intersection)
                for machine in M_intersection:
                    Y[job_a, task_a, job_b, task_b, machine] = solver.IntVar(
                        0, 1, f'Y{job_a}{task_a}{job_b}{task_b}{machine}'
                    )

for job_id, job in enumerate(jobs_data):
    combos = combinations(range(len(job)), 2)
    for combo in combos:
        M_intersection = intersection(
            Mj[job[combo[0]]["station_type"]],
            Mj[job[combo[1]]["station_type"]]
        )
        for machine in M_intersection:
            Z[job_id, combo[0], combo[1], machine] = solver.IntVar(
                0, 1, f'Z{job_id}{combo[0]}{combo[1]}{machine}'
            )

# Constraint construction.
# Job-specific constraints.
j = 0
for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        # Each operation can only be assigned to one machine.
        solver.Add( 
            sum(X[job_id, task_id, machine] for machine in Mj[task["station_type"]]) == 1
        )

        # Within a job, each task must start after the previous parent task ends.
        if task_id > 0:
            for parent in parent_ids[j]:
                solver.Add(
                    sum(S[job_id, task_id, machine] for machine in Mj[task["station_type"]]) >=
                    sum(C[job_id, parent, machine] for machine in Mj[job[parent]["station_type"]])
                )
                # print(parent, Mj[job[parent]["station_type"]])
        j += 1

for job_id, job in enumerate(jobs_data):
    combos = combinations(range(len(job)), 2)
    for combo in combos:
        M_intersection = intersection(
            Mj[job[combo[0]]["station_type"]],
            Mj[job[combo[1]]["station_type"]]
        )
        for machine in M_intersection:
            solver.Add(
                S[job_id, combo[0], machine] >=
                C[job_id, combo[1], machine] -
                    Z[job_id, combo[0], combo[1], machine]*L
            )
            solver.Add(
                S[job_id, combo[1], machine] >=
                C[job_id, combo[0], machine] -
                    (1-Z[job_id, combo[0], combo[1], machine])*L
            )
        # if len(M_intersection):
        #     print(M_intersection)
# print(Z)

# Task completion constraints.
for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        # Set constraints for all machines that match the task requirement.
        for machine in Mj[task["station_type"]]:
            # The start and end time must equal zero if the task is not
            # assigned to that machine.
            solver.Add(
                S[job_id, task_id, machine] + C[job_id, task_id, machine] <=
                  X[job_id, task_id, machine]*L
            )

            # Task completion must happen after task duration + start time.
            solver.Add(
                C[job_id, task_id, machine] >=
                S[job_id, task_id, machine] + task["duration"] - 
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
                    Mj[jobs_data[job_b][task_b]["station_type"]],
                    Mj[jobs_data[job_a][task_a]["station_type"]]
                )   
                # print(M_intersection)
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
            sum(C[job_id, len(job)-1, machine] for machine in Mj[job[-1]["station_type"]])
    )
    # # The overall makespan must be after the last job's completion. Not
    # # sure the usefulness.
    # solver.Add(
    #     C_max >= C_job[i]
    # )

# Minimize the sum of completion times.
solver.Minimize(solver.Sum(objective_terms))

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
ticket_ids, parentses = [], []
starts, ends, durations = [], [], []

for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        for machine in all_machines:
            if X[job_id, task_id, machine].solution_value() > 0.5:
                jobs.append(job_id)
                tasks.append(task_id)
                ticket_ids.append(task["ticket_id"])
                parentses.append(task["parents"])
                station_nums.append(machine)
                station_type_nums.append(task["station_type"])
                locations.append(station_names[task["station_type"]])
                starts.append(S[job_id, task_id, machine].solution_value())
                ends.append(C[job_id, task_id, machine].solution_value())
                durations.append(task["duration"])

schedule = pd.DataFrame()
schedule["Job #"] = jobs
schedule["Task #"] = tasks
schedule["Ticket ID"] = ticket_ids
schedule["Parents"] = parentses
schedule["Station #"] = station_nums
schedule["Station Type #"] = station_type_nums
schedule["Location"] = locations
schedule["Start"] = starts
schedule["End"] = ends
schedule["Duration"] = durations

print(f"Overall runtime: {time.time() - overallTime: .3f} seconds.")
print()

print(schedule)
# schedule.to_csv(f"Plans/fjssp.csv")
# draw_schedule(schedule)