'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Solve the mission planning problem using Microsoft Z3.
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
timeStep = 1

# Horizon in steps.
planHorizon = 40
visit_1_deadline = 15
visit_2_deadline = planHorizon - visit_1_deadline

# Robot characteristics. Currently unused.
robSize = 2     # ft.
maxSpeed = 5    # ft/s.

# Set up the grid.
grid = create_grid(bounds, spaceStep)
numStates = grid.size
print(f"Number of possible states (grid size): {numStates}")
print(f"Grid shape: {grid.shape}")

# Generate the set of reachable locations lookup table.
# Each row corresponds to an integer location.
# i.e. Reach(position x) is a list at index x.
reach = []
for i in range(numStates):
    neighbors = get_neighbors(i, grid, obs, stations)
    reach.append(neighbors)
# print(reach[15])

# Position variables for the solver.
X = [Ints('x%s' % i) for i in range(planHorizon)]
# print(X[-1])

# Set up the optimizer object
solver = Solver()

# Initial position in cell num.
solver.add(X[0][0] == initPose)

# Final position in cell num.
solver.add(X[-1][0] == finalPose)

# Constraints over the total horizon.
for k in range(planHorizon-1):
    # Keep x in workspace.
    solver.add(X[k][0] < numStates)
    solver.add(X[k][0] >= 0)
    
    # Reachable set. Have to add a set of or statements for
    # every possible state.
    for x in range(numStates):
        cons = []
        for x_next in reach[x]:
            cons.append( (X[k+1][0] == int(x_next)) )
        solver.add( Implies(X[k][0] == int(x), Or(cons)) )

# Visit workstation at visit_1 at least once.
cons = []
for k in range(visit_1_deadline):
    cons.append(
        X[k][0] == int(get_neighbors(RF_Welder[0], grid, obs, stations)[0])
    )
solver.add(Or(cons))

# Visit workstation at visit_2 at least once.
cons = []
for k in range(visit_1_deadline, planHorizon):
    cons.append(
        X[k][0] == int(get_neighbors(Grommet, grid, obs, stations)[0])
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
numRobots = 1
path = extract_path(m, X, planHorizon, numRobots)
print(path)

animate_path(numRobots, planHorizon, path, grid, stations, obs, bounds, spaceStep, "singleRobPathAdj.mp4")