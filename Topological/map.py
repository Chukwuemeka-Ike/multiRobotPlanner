'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Draws the metric map without any additional parameters.
'''
import matplotlib.pyplot as plt
from utils.draw_utils import *
from utils.grid_utils import create_grid
from utils.locations import *


# Distance in ft. Time in seconds.
bounds = [0, 120, 0, 40]
spaceStep = 5

# Robot characteristics. Currently unused.
robSize = 2     # ft.
maxSpeed = 5    # ft/s.

# Set up the grid.
grid = create_grid(bounds, spaceStep)
numStates = grid.size
print(f"Number of possible states (grid size): {numStates}")
print(f"Grid shape: {grid.shape}")


# Draw the map, obstacles, and stations.
fig, ax = plt.subplots(1, 1, figsize=(17,7))
fontSize = 20
draw_env(grid, stations, obs, bounds, spaceStep, ax, fontSize)
plt.show()