'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Implementation of the vanilla assignment problem.
    We have a pool of robots which can only tackle one job until it's done.
    Source:
        https://developers.google.com/optimization/assignment/assignment_example.
'''
import time

from ortools.sat.python import cp_model
from random import randint, seed

# Overall runtime.
overallStart = time.time()

# Costs are currently random, and there are enough workers for .
num_jobs = 5
num_workers = 20

seed(10)
costs = [[randint(0, 15) for _ in range(num_jobs)] for _ in range(num_workers)]
job_nums = [3, 5, 2, 6, 2]
# print(costs)

# Declare the CP-SAT model.
model = cp_model.CpModel()

# Create the variables.
x = []
for i in range(num_workers):
    t = []
    for j in range(num_jobs):
        t.append(model.NewBoolVar(f'x[{i},{j}]'))
    x.append(t)
# print(x)

# Constraints.
# Each worker is assigned to at most one task.
for i in range(num_workers):
    model.AddAtMostOne(x[i][j] for j in range(num_jobs))

# Each task is assigned to exactly one worker.
for j in range(num_jobs):
    # model.AddExactlyOne(x[i][j] for i in range(num_workers))
    model.Add(sum(x[i][j] for i in range(num_workers)) == job_nums[j])


# Create the objective function.
objective_terms = []
for i in range(num_workers):
    for j in range(num_jobs):
        objective_terms.append(costs[i][j] * x[i][j])
model.Minimize(sum(objective_terms))

# Invoke the solver.
solutionStart = time.time()
solver = cp_model.CpSolver()
status = solver.Solve(model)
print(f"Solution runtime: {time.time() - solutionStart: .3f} seconds.\n")

if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    print(f'Total cost = {solver.ObjectiveValue()}')
    print()
    for i in range(num_workers):
        for j in range(num_jobs):
            if solver.BooleanValue(x[i][j]):
                print(
                    f'Worker {i} assigned to task {j}.\tCost = {costs[i][j]}.')
else:
    print('No solution found.')

print()
print(f"Overall runtime: {time.time() - overallStart: .3f} seconds.")
print()