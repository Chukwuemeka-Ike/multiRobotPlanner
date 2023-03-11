'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Uses Z3 to schedule tasks and assign robots on the metric map.
    This script tests out sequencing 1, 2, and 3 visits in a single task.
    It runs longer than other scripts because of the large number of
    disjunctions that come with sequencing 2 and 3 visits.
'''
from z3 import *
import time
import pandas as pd

from locations import *
from utils.constraint_utils import *
from utils.draw_utils import animate_path
from utils.grid_utils import create_grid, create_reach_set, \
    extract_path, extract_occupied

# Distance in ft. Time in seconds.
bounds = [0, 120, 0, 40]
spaceStep = 5
timeStep = 10

# Robot speed for reachability.
maxSpeed = 2    # ft/s.

# Horizon in steps. In time, timeStep*planHorizon.
planHorizon = 60

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


# Set up the Z3 solver.
solver = Solver()

# Position variables for the robots.
numRobots = 6
X, Oc = create_robot_occupied_variables(numRobots, planHorizon)

# Set up all the constraints.
startTime = time.time()

# Initial and final position constraints.
initialPositions = [120, 119, 23, 45, 21, 20]
finalPositions = [20, 21, 22, 23, 47, 46]
terminal_position_constraints(solver, X, initialPositions, finalPositions)

# Allowable position constraints.
stay_in_workspace_constraints(solver, X, grid, planHorizon)
avoid_one_another(solver, X, planHorizon)
print(f"Position constraint setup runtime: {time.time() - startTime} seconds")

# Reachability constraints.
startTime = time.time()
reachable_set_constraints(solver, X, grid, planHorizon, reach)
print(f"Reachable constraint setup runtime: {time.time() - startTime} seconds")

# Task-specific constraints.
startTime = time.time()

# Visit RF_Welder for 10 timesteps.
taskTime = time.time()
numVisitors = 2
n_robots_visit_station_for_duration(solver, X, Oc, grid, obs, stations,
                45, planHorizon, numVisitors, RF_Welder, 10)
print(f"Single visit setup runtime:{time.time() - taskTime} seconds.")

# Visit Machine 2 for 10 timesteps.
numVisitors = 1
n_robots_visit_station_for_duration(solver, X, Oc, grid, obs, stations,
                30, planHorizon, numVisitors, Sewing_Machine_2, 10)

# Visit Grommet for 10 timesteps.
numVisitors = 3
n_robots_visit_station_for_duration(solver, X, Oc, grid, obs, stations,
                0, 40, numVisitors, Grommet, 10)

# Visit Mega_Stitch for 10 then Long_Arm for 5.
taskTime = time.time()
numVisitors = 3
n_robots_sequence_two_visits(solver, X, Oc, grid, obs, stations,
                20, planHorizon, numVisitors, Mega_Stitch, Long_Arm, 10, 5)
print(f"Two visit setup runtime:{time.time() - taskTime} seconds.")

# Visit Mega_Stitch for 10, Machine 3 for 10, and Grommet for 5.
taskTime = time.time()
numVisitors = 3
n_robots_sequence_three_visits(solver, X, Oc, grid, obs, stations, 20,
    planHorizon, numVisitors, Mega_Stitch, Sewing_Machine_3, Grommet, 10, 10, 5)
print(f"Three visit setup runtime:{time.time() - taskTime} seconds.")

print(f"Task constraint setup runtime: {time.time() - startTime} seconds")



# Check for sat and print the model if it exists.
startTime = time.time()
model_sat = solver.check()
if model_sat == unsat:
    print(f"The model was unsatisfiable. Exiting.\n")
    exit()
m = solver.model()
print(f"Solution runtime: {time.time() - startTime} seconds")


# Extract the path and occupied flags.
path = extract_path(m, X, planHorizon, numRobots)
occupied = extract_occupied(m, Oc, planHorizon, numRobots)


# Save the schedule and assignments, and the animated path.
df = pd.DataFrame()
df[[f"Path{i}" for i in range(numRobots)]] = path.T
df[[f"Occupied{i}" for i in range(numRobots)]] = occupied.T
print(df)
df.to_csv(f"Plans/complex{numRobots}RobotPlan.csv")

animate_path(numRobots, planHorizon, path, grid, stations, obs, bounds, spaceStep, "Videos/multiRobotSeqUtils.mp4")