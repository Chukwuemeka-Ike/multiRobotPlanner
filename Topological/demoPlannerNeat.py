'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Demo plan where all jobs fit within the 60 timestep horizon.
'''
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
print(f"Position constraint setup runtime: {time.time() - startTime} seconds.")

# Task-specific constraints.
startTime = time.time()


# Job 1.
singleTime = time.time()
numVisitors = 3

# Cutter -> Mega Stitch for 10.
n_robots_sequence_two_visits(solver, X, Oc, 0, 15, numVisitors, Cutter, Mega_Stitch, 1, 10)

# Mega Stitch -> Long Arm for 15.
n_robots_sequence_two_visits(solver, X, Oc, 15, 35, numVisitors, Mega_Stitch, Long_Arm, 1, 15)

# Long Arm -> Grommet for 10.
n_robots_sequence_two_visits(solver, X, Oc, 35, 50, numVisitors, Long_Arm, Grommet, 1, 10)

# Grommet -> Mega Stitch for 10.
n_robots_sequence_two_visits(solver, X, Oc, 45, 60, numVisitors, Grommet, Mega_Stitch, 1, 10)

print(f"Job 1 setup runtime: {time.time() - singleTime} seconds.")


# Job 2.
singleTime = time.time()
numVisitors = 2

# Cutter -> RF Welder for 15.
n_robots_sequence_two_visits(solver, X, Oc, 5, 25, numVisitors, Cutter, RF_Welder, 1, 15)

# RF Welder -> Mega Stitch for 10.
n_robots_sequence_two_visits(solver, X, Oc, 25, 40, numVisitors, RF_Welder, Mega_Stitch, 1, 10)

# Mega Stitch -> Grommet for 5.
n_robots_sequence_two_visits(solver, X, Oc, 40, 50, numVisitors, Mega_Stitch, Grommet, 1, 5)

# Grommet -> Machine 2 for 15.
n_robots_sequence_two_visits(solver, X, Oc, 47, 60, numVisitors, Grommet, Mega_Stitch, 1, 10)

print(f"Job 2 setup runtime: {time.time() - singleTime} seconds.")


# Currently makes the model unsat. Will add later.
# # Job 3.
# singleTime = time.time()
# numVisitors = 3

# # Cutter -> RF Welder for 15.
# n_robots_sequence_two_visits(solver, X, Oc, 25, 45, numVisitors, Cutter, RF_Welder, 1, 15)

# # RF Welder -> Long Arm for 10.
# n_robots_sequence_two_visits(solver, X, Oc, 40, 55, numVisitors, RF_Welder, Mega_Stitch, 1, 10)

# # Long Arm -> Machine 4 for 4.
# n_robots_sequence_two_visits(solver, X, Oc, 50, 60, numVisitors, Mega_Stitch, Grommet, 1, 4)

# print(f"Job 3 setup runtime: {time.time() - singleTime} seconds.")

# Check for sat and print the model if it exists.
startTime = time.time()
model_sat = solver.check()
if model_sat == unsat:
    print(f"The model was unsatisfiable. Exiting.\n")
    exit()
m = solver.model()
print(f"Solution runtime: {time.time() - startTime} seconds.")
print(f"Overall runtime: {time.time() - overallTime} seconds.")


# Extract the path and occupied flags.
path = extract_path(m, X, planHorizon, numRobots)
occupied = extract_occupied(m, Oc, planHorizon, numRobots)
locations = extract_location_names(path, loc_names)

import pandas as pd

df = pd.DataFrame()
df[[f"Location {i+1}" for i in range(numRobots)]] = locations.T
df[[f"Occupied {i+1}" for i in range(numRobots)]] = occupied.T
# print(df)

df.to_csv(f"Plans/neatDemo{numRobots}Bots.csv")

animate_path(numRobots, planHorizon, path, grid, stations, obs, bounds, spaceStep, "Videos/neatDemo.mp4")