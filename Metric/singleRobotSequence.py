'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Sequences visiting multiple locations with a single robot.
'''
from z3 import *
import matplotlib.pyplot as plt
import numpy as np
import time

from utils.grid_utils import *
from utils.draw_utils import *
from locations import *

# Distance in ft. Time in seconds.
bounds = [0, 120, 0, 40]
spaceStep = 5
timeStep = 10

# Robot characteristics.
robSize = 2     # ft.
maxSpeed = 2    # ft/s.

# Horizon in steps. In time, timeStep*planHorizon.
planHorizon = 25
visit_1_deadline = 10
visit_2_deadline = planHorizon - visit_1_deadline

# Set up the grid.
grid = create_grid(bounds, spaceStep)
numStates = grid.size
print(f"Number of possible states (grid size): {numStates}")
print(f"Grid shape: {grid.shape}")

# Generate the set of reachable locations lookup table.
# Each row corresponds to an integer location.
# i.e. Reach(position x) is a list at index x.
startTime = time.time()
reach = []
distances = []
previous = []
for i in range(numStates):
    reachable_set, dist, prev = get_reach(i, grid, obs, stations, timeStep,
                                        spaceStep, maxSpeed)
    reach.append(reachable_set)
    distances.append(dist)
    previous.append(prev)
del reachable_set, dist, prev
print(f"Reachable set generation runtime: {time.time() - startTime} seconds")
# x = 15
# print(reach[x])
# print(distances[x])


# Position variables for the solver.
X = [Int('x%s' % i) for i in range(planHorizon)]
# print(X[-1])

# Set up the optimizer object
solver = Solver()

# Initial position in cell num.
solver.add(X[0] == initPose)

# Final position in cell num.
solver.add(X[-1] == finalPose)

# Constraints over the total horizon.
for k in range(planHorizon-1):
    # solver.minimize(X[k])

    # Keep x in workspace.
    solver.add(X[k] < numStates)
    solver.add(X[k] >= 0)

    # Reachable set. Have to add a set of or statements for
    # every possible state.
    for x in range(numStates):
        cons = []
        for x_next in reach[x]:
            cons.append( (X[k+1] == int(x_next)) )
        solver.add( Implies(X[k] == int(x), Or(cons)) )

# Visit workstation at visit_1 at least once.
cons = []
for k in range(visit_1_deadline):
    cons.append(
        X[k] == int(get_neighbors(visit_1, grid, obs, stations)[0])
    )
solver.add(Or(cons))

# Visit workstation at visit_2 at least once.
cons = []
for k in range(visit_1_deadline, planHorizon):
    cons.append(
        X[k] == int(get_neighbors(visit_2, grid, obs, stations)[0])
    )
solver.add(Or(cons))


# Check for sat and print the model if it exists.
startTime = time.time()
model_sat = solver.check()
if model_sat == unsat:
    print(f"The model was unsatisfiable. Exiting.\n")
    exit()
m = solver.model()
print(f"Solution runtime: {time.time() - startTime} seconds")

# Extract the path.
path = []
for k in range(planHorizon):
    path.append(m[X[k]].as_long())
print(f"Path:\n{path}")


# Draw the map, obstacles, stations, and the generated path.
ax = draw_map(bounds, spaceStep)

for x in obs:
    draw_obstacle(x, grid, spaceStep, ax)

# for k in range(planHorizon):
#     draw_rectangle(path[k], grid, spaceStep, ax)
for k in range(planHorizon):
    if k < planHorizon-1:
        connector = find_shortest_path(path[k], path[k+1], reach[path[k]], previous[path[k]])
        for entry in connector:
            draw_rectangle(entry, grid, spaceStep, ax)
    draw_rectangle(path[k], grid, spaceStep, ax)

for x in stations:
    draw_station(x, grid, spaceStep, ax)

plt.show()