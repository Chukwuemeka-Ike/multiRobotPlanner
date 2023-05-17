'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Draws the metric map without any additional parameters.
'''
import matplotlib.pyplot as plt

from constants.locations import stations, station_locations
from utils.draw_utils import draw_env
from utils.map_utils import *

# Distance in ft. Time in seconds.
bounds = [0, 150, 0, 42]
spaceStep = 3

# Robot characteristics. Currently unused.
robSize = 2     # ft.
maxSpeed = 5    # ft/s.

# Set up the grid.
grid = create_grid_map(bounds, spaceStep)
numStates = grid.size
print(f"Number of possible states (grid size): {numStates}")
print(f"Grid shape: {grid.shape}")

station_num = 1
station_neighbors = get_station_neighbors(station_locations[station_num], grid, stations)
print(station_neighbors)
print(get_coordinates(station_neighbors[0], grid, spaceStep))

# Draw the map, obstacles, and stations.
fig, ax = plt.subplots(1, 1, figsize=(17,7))
fontSize = 20
draw_env(grid, stations, [], bounds, spaceStep, ax, fontSize)
plt.show()