'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Solve the mission planning problem using Microsoft Z3 using
    all the utils scripts.
'''
from z3 import *
import time

from locations import *
from utils.constraint_utils import *
from utils.draw_utils import *
from utils.grid_utils import *

# Distance in ft. Time in seconds.
bounds = [0, 120, 0, 40]
spaceStep = 5
timeStep = 10

# Horizon in steps.
planHorizon = 60
visit_1_deadline = 30
visit_1_duration = 5
visit_2_duration = 10

# Robot characteristics. Currently unused.
robSize = 2     # ft.
maxSpeed = 2    # ft/s.

# Set up the grid.
grid = create_grid(bounds, spaceStep)
numStates = grid.size
print(f"Number of possible states (grid size): {numStates}")
print(f"Grid shape: {grid.shape}")

# Generate the reach set.
startTime = time.time()
reach, distances, previous = create_reach_set(grid, obs, stations, timeStep,
                                                        spaceStep, maxSpeed)
# reach = create_adjacent_reach_set(grid, obs, stations)
print(f"Reachable set generation runtime: {time.time() - startTime} seconds")


# Set up the Z3 solver.
solver = Solver()

# Position variables for the robot.
numRobots = 1
X = [Ints('x%s' % i) for i in range(planHorizon)]
print(f"Final position variable(s): {X[-1]}")

# Set up all the constraints.
startTime = time.time()

# Initial and final position constraints.
terminal_position_constraints(solver, X, [144], [144])

# Constraints for reachable locations from any location.
stay_in_workspace_constraints(solver, X, grid, planHorizon)

startTime = time.time()
reachable_set_constraints(solver, X, grid, planHorizon, reach)
print(f"Reachable constraint setup runtime: {time.time() - startTime} seconds")


# print(f"Constraint setup runtime: {time.time() - startTime} seconds")


# # Check for sat and print the model if it exists.
# startTime = time.time()
# model_sat = solver.check()
# if model_sat == unsat:
#     print(f"The model was unsatisfiable. Exiting.\n")
#     exit()
# m = solver.model()
# print(f"Solution runtime: {time.time() - startTime} seconds")

# # Extract the path.
# path = extract_path(m, X, planHorizon, numRobots)
# print(path)

# animate_path(numRobots, planHorizon, path, grid, stations, obs, bounds, spaceStep, "Videos/singleRobPath.mp4")