'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Multi robot planner using the topological map.
'''
from z3 import *
import time

from locations import *
from constraint_utils import *
from grid_utils import *
from draw_utils import *

# Horizon in steps. Step length is defined separately.
planHorizon = 60

# Check map setup.
print(f"Number of nodes: {numStates}")


# Set up the Z3 solver.
solver = Solver()
# set_option("parallel.enable", True)

# Robot position and occupied variables.
numRobots = 6
X, Oc = create_robot_occupied_variables(numRobots, planHorizon)

# # Constraints for initial and final positions, and for staying in workspace.
# startTime = time.time()
# initialPositions = [Base]*numRobots
# finalPositions = [Base]*numRobots
# terminal_position_constraints(solver, X, initialPositions, finalPositions)
# stay_in_workspace_constraints(solver, X, workspace, planHorizon)
# print(f"Position constraint setup runtime: {time.time() - startTime} seconds.")

# # Task-specific constraints.
# startTime = time.time()

# # Visit RF_Welder for 10 timesteps.
# singleTime = time.time()
# numVisitors = 2
# n_robots_visit_station_for_duration(solver, X, Oc, 45, planHorizon,
#                                     numVisitors, RF_Welder, 10)
# print(f"Single visit setup runtime:{time.time() - singleTime} seconds.")

# # Visit Machine 2 for 10 timesteps.
# numVisitors = 1
# n_robots_visit_station_for_duration(solver, X, Oc, 30, planHorizon,
#                 numVisitors, Sewing_Machine_2, 10)

# # Visit Grommet for 10 timesteps.
# numVisitors = 4
# n_robots_visit_station_for_duration(solver, X, Oc, 0, 40, 
#                 numVisitors, Grommet, 10)

# Visit Mega_Stitch for 10 then Long_Arm for 5.
singleTime = time.time()
numVisitors = 2
n_robots_sequence_two_visits(solver, X, Oc, 0, planHorizon,
                numVisitors, Mega_Stitch, Long_Arm, 10, 5)
print(f"Two visit setup runtime:{time.time() - singleTime} seconds.")


filename = "Constraints/z3save3.smt"

startTime = time.time()
constraintsText = solver.sexpr()
with open(filename, mode='w', encoding='ascii') as f:
    f.write(constraintsText)
    f.close()
print(f"Save constraint runtime: {time.time() - startTime} seconds")

# # Visit Mega_Stitch for 10, Machine 3 for 10, and Grommet for 5.
# singleTime = time.time()
# numVisitors = 3
# n_robots_sequence_three_visits(solver, X, Oc, 20, planHorizon,
#             numVisitors, Mega_Stitch, Sewing_Machine_3, Grommet, 10, 10, 5)
# print(f"Three visit setup runtime:{time.time() - singleTime} seconds.")

# print(f"Task constraint setup runtime: {time.time() - startTime} seconds.")

# # pp(solver)

# Check for sat and print the model if it exists.
startTime = time.time()
model_sat = solver.check()
if model_sat == unsat:
    print(f"The model was unsatisfiable. Exiting.\n")
    exit()
m = solver.model()
print(f"Solution runtime: {time.time() - startTime} seconds.")


# Extract the path and occupied flags.
path = extract_path(m, X, planHorizon, numRobots)
occupied = extract_occupied(m, Oc, planHorizon, numRobots)

import pandas as pd

df = pd.DataFrame()
df[[f"Path{i}" for i in range(numRobots)]] = path.T
df[[f"Occupied{i}" for i in range(numRobots)]] = occupied.T
print(df)
df.to_csv(f"Plans/complex{numRobots}RobotPlan.csv")

animate_path(numRobots, planHorizon, path, grid, stations, obs, bounds, spaceStep, "Videos/multiRobotAnim.mp4")