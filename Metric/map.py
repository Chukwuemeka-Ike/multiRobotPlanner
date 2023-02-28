'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for graphing and mapping a grid environment.
'''
import matplotlib.pyplot as plt
import numpy as np
from utils.draw_utils import *
from locations import *


# Distance in ft. Time in seconds.
bounds = [0, 120, 0, 40]
spaceStep = 5
timeStep = 1

# Horizon in steps.
planHorizon = 50
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

# Draw the map, obstacles, and stations.
ax = draw_map(bounds, spaceStep)

# target = 20
# center_point = get_rectangle_center(target, grid, spaceStep)
# print(f"Target spot: {target}")
# print(f"Center point: {center_point}")
# draw_rectangle(target, grid, spaceStep, ax)
# ax.plot(center_point[0], center_point[1], 'o')

for x in obs:
    draw_obstacle(x, grid, spaceStep, ax)

for x in stations:
    draw_station(x, grid, spaceStep, ax)

plt.show()
# plt.savefig('tt.svg')