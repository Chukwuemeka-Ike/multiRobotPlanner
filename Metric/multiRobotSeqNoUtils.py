'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    First cut at using Z3 to assign robot to tasks.
    We specify that we want:
        2 robots to visit RF Welder,
        2 robots to visit the Grommet then machine 2, and
        1 robot to the Mega Stitch
    given 3 robots. This is an example of both robot assignment and task
    scheduling.
'''
from z3 import *
import matplotlib.pyplot as plt
import numpy as np
import time

from utils.draw_utils import *
from utils.grid_utils import *
from locations import *

# Distance in ft. Time in seconds.
bounds = [0, 120, 0, 40]
spaceStep = 5
timeStep = 10

# Robot characteristics.
robSize = 2     # ft.
maxSpeed = 2    # ft/s.

# Horizon in steps. In time, timeStep*planHorizon.
planHorizon = 30
visit_1_deadline = int(planHorizon/2)
visitDuration_1 = 5
visitDuration_2 = 10

# Set up the grid.
grid = create_grid(bounds, spaceStep)
numStates = grid.size
print(f"Number of possible states (grid size): {numStates}")
print(f"Grid shape: {grid.shape}")

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


# Time the constraint setup process.
startTime = time.time()

# Set up the constraint solver.
solver = Solver()

# Position variables for the solver.
# There are 3 robots, so we have x1-3 for each timestep.
X = [Ints('x1%s x2%s x3%s' % (i,i,i)) for i in range(planHorizon)]
# print(X[0][0])

# Initial locations for the robots.
initialPositions = [120, 119, 23]
numBots = len(initialPositions)
solver.add([X[0][i] == initialPositions[i] for i in range(numBots)])
solver.add([X[-1][i] == initialPositions[i] for i in range(numBots)])

# Stations of interest.
station_1 = [176, 177] # RF_Welder
station_2 = [191] # Grommet
station_3 = [12] # Sewing_Machine_2
station_4 = [27, 28, 51, 52] # Mega_Stitch

# Constraints over the total horizon.
for k in range(planHorizon-1):
    # Keep x in workspace.
    solver.add([X[k][i] < numStates for i in range(numBots)])
    solver.add([X[k][i] >= 0 for i in range(numBots)])

    # Reachable set. Have to add a set of or statements for
    # every possible state.
    for i in range(numBots):
        for x in range(numStates):
            cons = [(X[k+1][i] == int(x_next)) for x_next in reach[x]]
            solver.add( Implies(X[k][i] == int(x), Or(cons)) )

# Two robots to visit station_1 and stay for visitDuration_1.
station_1_neighbors = get_neighbors(station_1[0], grid, obs, stations)
sta1_1 = station_1_neighbors[0]
sta1_2 = station_1_neighbors[1]
print(f"sta1_1 desired spot: {sta1_1}")
print(f"sta1_2 desired spot: {sta1_2}")
cons = [And(X[k][i] == int(sta1_1), X[k][j] == int(sta1_2)) for k in range(planHorizon-visitDuration_1) for i in range(numBots) for j in range(i+1, numBots)]
solver.add(Or(cons))

for i in range(numBots):
    for k in range(1, planHorizon-visitDuration_1):
        cons = [X[k+j][i] == int(sta1_1) for j in range(1, visitDuration_1)]
        solver.add( Implies(
            And(X[k-1][i] != int(sta1_1),
                X[k][i] == int(sta1_1)),
            And(cons))
        )

for i in range(numBots):
    for k in range(1, planHorizon-visitDuration_1):
        cons = [X[k+j][i] == int(sta1_2) for j in range(1, visitDuration_1)]
        solver.add( Implies(
            And(X[k-1][i] != int(sta1_2),
                X[k][i] == int(sta1_2)),
            And(cons))
        )

# Two robots to visit station_2, then station_3.
station_2_neighbors = get_neighbors(station_2[0], grid, obs, stations)
sta2_1 = station_2_neighbors[0]
sta2_2 = station_2_neighbors[1]
print(f"Sta2_1 desired spot: {sta2_1}")
print(f"Sta2_2 desired spot: {sta2_2}")
cons = [And(X[k][i] == int(sta2_1), X[k][j] == int(sta2_2)) for k in range(visit_1_deadline) for i in range(numBots) for j in range(i+1, numBots)]
solver.add(Or(cons))

station_3_neighbors = get_neighbors(station_3[0], grid, obs, stations)
sta3_1 = station_3_neighbors[0]
sta3_2 = station_3_neighbors[1]
print(f"Sta3_1 desired spot: {sta3_1}")
print(f"Sta3_2 desired spot: {sta3_2}")
cons = [And(X[k][i] == int(sta3_1), X[k][j] == int(sta3_2)) for k in range(visit_1_deadline,planHorizon) for i in range(numBots) for j in range(i+1, numBots)]
solver.add(Or(cons))

station_4_neighbors = get_neighbors(station_4[0], grid, obs, stations)
sta4_1 = station_4_neighbors[0]
sta4_2 = station_4_neighbors[1]
print(f"Sta4_1 desired spot: {sta4_1}")
print(f"Sta4_2 desired spot: {sta4_2}")
cons = [And(X[k][i] == int(sta4_1), X[k][j] == int(sta4_2)) for k in range(planHorizon-visitDuration_2) for i in range(numBots) for j in range(i+1, numBots)]
solver.add(Or(cons))

for i in range(numBots):
    for k in range(1, planHorizon-visitDuration_2):
        cons = [X[k+j][i] == int(sta4_1) for j in range(1, visitDuration_2)]
        solver.add( Implies(
            And(X[k-1][i] != int(sta4_1),
                X[k][i] == int(sta4_1)),
            And(cons))
        )

for i in range(numBots):
    for k in range(1, planHorizon-visitDuration_2):
        cons = [X[k+j][i] == int(sta4_2) for j in range(1, visitDuration_2)]
        solver.add( Implies(
            And(X[k-1][i] != int(sta4_2),
                X[k][i] == int(sta4_2)),
            And(cons))
        )


print(f"Constraint setup runtime: {time.time() - startTime} seconds")
# print(solver)


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
for i in range(numBots):
    a = [m[X[k][i]].as_long() for k in range(planHorizon)]
    path.append(a)
print(f"Paths:\n")
print(path[0])
print(path[1])
print(path[2])

fig, ax = plt.subplots(1,3)
fig.suptitle('Robot Planned Paths')
plt.setp(ax, 
        xticks=np.arange(bounds[0], bounds[1]+1, spaceStep),
        yticks=np.arange(bounds[2], bounds[3]+1, spaceStep)
)
cc = ["m", "c", "g", "y"]

robot = 0


for j in range(3):
    ax[j].axis(bounds)
    ax[j].grid()

    # Draw obstacles and stations.
    for x in obs:
        draw_obstacle(x, grid, spaceStep, ax[j])
    for x in stations:
        draw_station(x, grid, spaceStep, ax[j])
    
    # Draw the path for this robot.
    for k in range(planHorizon):
        draw_rectangle(path[robot][k], grid, spaceStep, ax[j], cc[robot])

    robot = robot + 1
plt.show()


# for j in range(3):
#     ax[j].axis(bounds)
#     ax[j].grid()

#     # Draw obstacles and stations.
#     for x in obs:
#         draw_obstacle(x, grid, spaceStep, ax[j])
#     for x in stations:
#         draw_station(x, grid, spaceStep, ax[j])
    
#     # Draw the path for this robot.
#     for k in range(planHorizon):
#         if k < planHorizon-1:
#             connector = find_shortest_path(path[robot][k], path[robot][k+1], reach[path[robot][k]], previous[path[robot][k]])
#             for entry in connector:
#                 draw_rectangle(entry, grid, spaceStep, ax[j], cc[robot])        
#         draw_rectangle(path[robot][k], grid, spaceStep, ax[j], cc[robot])

#     robot = robot + 1
# plt.show()