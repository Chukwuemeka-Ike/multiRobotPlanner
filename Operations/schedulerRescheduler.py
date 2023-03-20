'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Implementation of the flexible job shop problem using
    Google OR-Tools Linear Solver.

    This script handles tree jobs. We re-run the optimization after the initial
    to reduce the amount of idle-time between operations of a job.

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
from utils.draw_utils import draw_tree_schedule
from utils.display_utils import *
from utils.sched_utils import extract_schedule
from utils.solver_utils import *


def intersection(lst1, lst2):
    '''Returns the common items between two lists.'''
    return list(set(lst1) & set(lst2))

# Start timing the overall runtime.
overallTime = time.time()

# Job data. Every job starts at the loading area.
# task_template = {"ticket_id": , "station_type": , "duration": , "parents": []}
jobs_data = tree_jobs
num_jobs = len(jobs_data)
print(f"Number of Jobs: {num_jobs}.")

# Print out the job information.
display_all_tickets(jobs_data)
display_station_numbers(station_names, Mj)

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
X, Y, Z, S, C, C_job = create_opt_variables(solver, jobs_data, horizon, all_machines, Mj)
L = 1000




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
solver.SetTimeLimit(4000)
status = solver.Solve()

# Display the solution.
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



























solver1 = pywraplp.Solver.CreateSolver('SCIP')

if not solver1:
    exit()


# Variable creation.
X, Y, Z, S, C, C_job = create_opt_variables(solver1, jobs_data, horizon, all_machines, Mj)
L = 1000




# Constraint construction.
# Job-specific constraints.
j = 0
for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        # Each operation can only be assigned to one machine.
        solver1.Add( 
            sum(X[job_id, task_id, machine] for machine in Mj[task["station_type"]]) == 1
        )

        # Within a job, each task must start after the previous parent task ends.
        if task_id > 0:
            for parent in parent_ids[j]:
                solver1.Add(
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
            solver1.Add(
                S[job_id, combo[0], machine] >=
                C[job_id, combo[1], machine] -
                    Z[job_id, combo[0], combo[1], machine]*L
            )
            solver1.Add(
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
            solver1.Add(
                S[job_id, task_id, machine] + C[job_id, task_id, machine] <=
                  X[job_id, task_id, machine]*L
            )

            # Task completion must happen after task duration + start time.
            solver1.Add(
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
                    solver1.Add(
                        S[job_a, task_a, machine] >=
                            C[job_b, task_b, machine] - 
                            Y[job_a, task_a, job_b, task_b, machine]*L
                    )
                    solver1.Add(
                        S[job_b, task_b, machine] >=
                            C[job_a, task_a, machine] - 
                            (1-Y[job_a, task_a, job_b, task_b, machine])*L
                    )



# Get the objective value.
optimum = solver.Objective().Value()

# # Minimize the idle time between loading and the next operation.
# solver1.Add(0 <= solver1.Sum(objective_terms) <= optimum*1.1)
# idle_times = []
# for job_id, job in enumerate(jobs_data):
#     idle_times.append(
#         sum(S[job_id, 1, machine] for machine in Mj[job[1]["station_type"]])
#             - sum(C[job_id, 0, machine] for machine in Mj[job[0]["station_type"]])
#     )
# solver1.Minimize(solver1.Sum(idle_times))

# Minimize all idle times between operations in a job.
solver1.Add(0 <= solver1.Sum(objective_terms) <= optimum*1.1)
idle_times = []
for job_id, job in enumerate(jobs_data):
    idle_times.append(
        sum(
            sum(S[job_id, task_id, machine] for machine in Mj[job[task_id]["station_type"]])
            - sum(C[job_id, task_id, machine] for machine in Mj[job[task_id]["station_type"]])
            for task_id in range(1, len(job))
        )
    )
solver1.Minimize(solver1.Sum(idle_times))


# Invoke the solver1.
solutionStart = time.time()
# solver1.SetTimeLimit(4000)
status = solver1.Solve()

# Display the solution.
print("\n\n")
if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
    print(f"Max Schedule Length: {horizon: .1f}")
    print(f"Objective value: {solver1.Objective().Value()}")
    print(
        "Optimal Schedule Length:",
        f"{max(obj_term.solution_value() for obj_term in objective_terms)}."
    )
    print(f"Solution Runtime: {time.time() - solutionStart: .3f} seconds.")
else:
    print("Infeasible program. Exiting.\n")
    exit()

# Extract the schedule.
schedule = extract_schedule(X, S, C, jobs_data, all_machines, station_names)
print(f"Overall runtime: {time.time() - overallTime: .3f} seconds.")
print()
print(schedule)

# Save the schedule and draw it.
schedule.to_csv(f"Plans/fjsspTreeRescheduler.csv")
draw_tree_schedule(schedule)