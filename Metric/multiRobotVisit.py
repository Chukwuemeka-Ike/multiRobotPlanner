'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    First cut at using Z3 to assign robot to tasks. We specify that we want
    2 robots to visit station 1, and 1 robot to visit station 3 out of the
    4 robots we have available.
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
planHorizon = 15
visitDuration = 10

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


startTime = time.time()
# Set up the constraint solver.
solver = Solver()

# Position variables for the solver.
# There are 4 robots, so we have x1-4 for each timestep.
X = [Ints('x1%s x2%s x3%s x4%s' % (i,i,i,i)) for i in range(planHorizon)]
# print(X[0][0])

# Initial locations for the robots.
initialPositions = [120, 145, 119, 23]
numBots = len(initialPositions)
solver.add([X[0][i] == initialPositions[i] for i in range(numBots)])

#
station_1 = [176, 177]
station_2 = [27, 28, 51, 52]
station_3 = 12

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
    
# Constraint for two robots to visit station_1.
sta1 = get_neighbors(station_1[0], grid, obs, stations)[0]
sta2 = get_neighbors(station_1[0], grid, obs, stations)[1]
print(f"Sta1 desired spot: {sta1}")
print(f"Sta2 desired spot: {sta2}")
# for k in range(planHorizon-visitDuration):
cons = [And(X[k][i] == int(sta1), X[k][j] == int(sta2)) for k in range(planHorizon-visitDuration) for i in range(numBots) for j in range(i, numBots)]
solver.add(Or(cons))

# Constraint to visit station_3.
sta3 = get_neighbors(station_3, grid, obs, stations)[0]
print(f"Sta3 desired spot: {sta3}")
cons = [X[k][i] == int(sta3) for k in range(planHorizon-visitDuration) for i in range(numBots)]
solver.add(Or(cons))

# Constraint to stay at a station for 10 timesteps.
for i in range(numBots):
    for k in range(1, planHorizon-visitDuration):
        cons = [X[k+j][i] == int(sta1) for j in range(1, visitDuration)]
        solver.add( Implies(
            And(X[k-1][i] != int(sta1),
                X[k][i] == int(sta1)), 
            And(cons)) )

for i in range(numBots):
    for k in range(1, planHorizon-visitDuration):
        cons = [X[k+j][i] == int(sta2) for j in range(1, visitDuration)]
        solver.add( Implies(
            And(X[k-1][i] != int(sta2),
                X[k][i] == int(sta2)), 
            And(cons)) )

for i in range(numBots):
    for k in range(1, planHorizon-visitDuration):
        cons = [X[k+j][i] == int(sta3) for j in range(1, visitDuration)]
        solver.add( Implies(
            And(X[k-1][i] != int(sta3),
                X[k][i] == int(sta3)), 
            And(cons)) )

# Add a constraint that whichever robot is unused stays where it is.
cons = []
for i in range(numBots):
    cons.append(And([ X[k][i] == initialPositions[i] for k in range(planHorizon) ]))
solver.add(Or(cons))

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
print(path[3])


fig, ax = plt.subplots(2,2)
fig.suptitle('Robot Planned Paths')
plt.setp(ax, 
        xticks=np.arange(bounds[0], bounds[1]+1, spaceStep),
        yticks=np.arange(bounds[2], bounds[3]+1, spaceStep)
)

robot = 0
cc = ["m", "c", "g", "y"]
for i in range(2):
    for j in range(2):
        ax[i,j].axis(bounds)
        ax[i,j].grid()

        # Draw obstacles and stations.
        for x in obs:
            draw_obstacle(x, grid, spaceStep, ax[i,j])
        for x in stations:
            draw_station(x, grid, spaceStep, ax[i,j])

        # Draw the path for this robot.
        for k in range(planHorizon):
            if k < planHorizon-1:
                connector = find_shortest_path(path[robot][k], path[robot][k+1], reach[path[robot][k]], previous[path[robot][k]])
                for entry in connector:
                    draw_rectangle(entry, grid, spaceStep, ax[i,j], cc[robot])        
            draw_rectangle(path[robot][k], grid, spaceStep, ax[i,j], cc[robot])

        robot = robot + 1
plt.show()

fig, ax = plt.subplots(2,2)
fig.suptitle('Robot Planned Paths')
plt.setp(ax, 
        xticks=np.arange(bounds[0], bounds[1]+1, spaceStep),
        yticks=np.arange(bounds[2], bounds[3]+1, spaceStep)
)

robot = 0
cc = ["m", "c", "g", "y"]
for i in range(2):
    for j in range(2):
        ax[i,j].axis(bounds)
        ax[i,j].grid()

        # Draw obstacles and stations.
        for x in obs:
            draw_obstacle(x, grid, spaceStep, ax[i,j])
        for x in stations:
            draw_station(x, grid, spaceStep, ax[i,j])
        
        # Draw the path for this robot.
        for k in range(planHorizon):
            draw_rectangle(path[robot][k], grid, spaceStep, ax[i,j], cc[robot])

        robot = robot + 1
plt.show()