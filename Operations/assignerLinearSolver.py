'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Implementation of the robot-job assignment problem.
    We have a pool of robots which can only tackle one job until it's done.
    Each job
'''
import time

from ortools.linear_solver import pywraplp


# Costs.
costs = [
    [90, 80, 75, 70],
    [90, 80, 75, 70],
    [35, 85, 55, 65],
    [125, 95, 90, 95],
    [125, 95, 90, 95],
    [45, 110, 95, 115],
    [50, 100, 90, 100],
    [50, 100, 90, 100],
]
num_workers = len(costs)
num_jobs = len(costs[0])
job_reqs = [1, 3, 2, 2]

print(f"Number of available robots: {num_workers}")

# Solver.
solver = pywraplp.Solver.CreateSolver('SCIP')

if not solver:
    exit()

# Variables.
# x[i, j] is an array of 0-1 variables, which will be 1 if worker i is assigned
# to job j.
x = {}
for i in range(num_workers):
    for j in range(num_jobs):
        x[i, j] = solver.IntVar(0,1, '')


# Constraints.
# Each worker is assigned to at most 1 job.
for i in range(num_workers):
    solver.Add(solver.Sum([x[i, j] for j in range(num_jobs)]) <= 1)

# Each job is assigned to the number of workers that it requires.
for j in range(num_jobs):
    solver.Add(solver.Sum([x[i, j] for i in range(num_workers)]) == job_reqs[j])
    # solver.Add(solver.Sum([x[i, j] for i in range(num_workers)]) >= 1)


# Objective.
objective_terms = []
for i in range(num_workers):
    for j in range(num_jobs):
        objective_terms.append(costs[i][j]*x[i, j])
solver.Minimize(solver.Sum(objective_terms))

# Invoke the solver.
status = solver.Solve()

# Print solution.
if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
    print(f"Total cost = {solver.Objective().Value()}\n")

    for j in range(num_jobs):
        print(f"Job {j}:")
        for i in range(num_workers):
            if x[i, j].solution_value() > 0.5:
                print(f"Robot {i} is assigned. Cost: {costs[i][j]}.")
        print()
else:
    print("No solution found.")