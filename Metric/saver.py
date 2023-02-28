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

# Robot characteristics.
robSize = 2     # ft.
maxSpeed = 2    # ft/s.

# Horizon in steps. In time, timeStep*planHorizon.
planHorizon = 60
visit_1_deadline = int(planHorizon/2)
visit_1_duration = 10
visit_2_duration = 5

# Set up the grid.
grid = create_grid(bounds, spaceStep)
numStates = grid.size
print(f"Number of possible states (grid size): {numStates}")
print(f"Grid shape: {grid.shape}")

# Generate the reach set.
startTime = time.time()
reach, distances, previous = create_reach_set(grid, obs, stations, timeStep,
                                                        spaceStep, maxSpeed)
print(f"Reachable set generation runtime: {time.time() - startTime} seconds")


filename = "Constraints/z3save2.smt"


# # Set up the Z3 solver.
# solver = Solver()
numRobots = 3
X, Oc = create_robot_occupied_variables(numRobots, planHorizon)

# startTime = time.time()
# reachable_set_constraints(solver, X, grid, planHorizon, reach)
# print(f"Reachable constraint setup runtime: {time.time() - startTime} seconds")


# startTime = time.time()
# constraintsText = solver.sexpr()
# with open(filename, mode='w', encoding='ascii') as f:
#     f.write(constraintsText)
#     f.close()
# print(f"Save constraint runtime: {time.time() - startTime} seconds")

startTime = time.time()
solver = Solver()
solver.from_file(filename)
print(f"Load constraint runtime: {time.time() - startTime} seconds")

# pp(solver)

# animate_path(numRobots, planHorizon, path, grid, stations, obs, bounds, spaceStep, "Videos/multiRobotSeqSaver.mp4")