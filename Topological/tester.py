from z3 import *
import time

from locations import *
from constraint_utils import *
from draw_utils import *
from grid_utils import *
from location_utils import *

# Horizon in steps. Step length is defined separately.
planHorizon = 60
numRobots = 8

# Print current setup.
print(f"Plan horizon: {planHorizon}.")
print(f"Number of nodes: {numStates}.")
print(f"Number of robots: {numRobots}.")


# Set up the Z3 solver.
solver = Solver()
# set_option("parallel.enable", True)

overallTime = time.time()

# Robot position and occupied variables.
X, Oc = create_robot_occupied_variables(numRobots, planHorizon)

# Constraints for initial and final positions, and for staying in workspace.
startTime = time.time()
initialPositions = [Base]*numRobots
finalPositions = [Base]*numRobots
terminal_position_constraints(solver, X, initialPositions, finalPositions)
stay_in_workspace_constraints(solver, X, workspace, planHorizon)
print(f"Position constraint setup runtime: {time.time() - startTime: .3f} seconds.")

# Task-specific constraints.
startTime = time.time()

numVisitors = 4

# 1-visit.
singleTime = time.time()
n_robots_visit_station_for_duration(
    solver, X, Oc, 0, 60, numVisitors, RF_Welder, 15
)
print(f"1-visit setup runtime: {time.time() - singleTime: .3f} seconds.")

# # 2-visit.
# singleTime = time.time()
# n_robots_sequence_two_visits(
#     solver, X, Oc, 0, 60, numVisitors, RF_Welder, Sewing_Machine_2, 15, 10
# )
# print(f"2-visit setup runtime: {time.time() - singleTime: .3f} seconds.")


# # 3-visit.
# singleTime = time.time()
# n_robots_sequence_three_visits(
#     solver, X, Oc, 0, 60, numVisitors, RF_Welder, Sewing_Machine_2, Grommet, 15, 10, 10
# )
# print(f"3-visit setup runtime: {time.time() - singleTime: .3f} seconds.")

