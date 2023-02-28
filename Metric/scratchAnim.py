import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import itertools
import time

import pandas as pd

from utils.grid_utils import *
from utils.draw_utils import *
from locations import *


# Distance in ft. Time in seconds.
bounds = [0, 120, 0, 40]
spaceStep = 5
timeStep = 10

# Set up the grid.
xLength = int(bounds[1]/spaceStep)
yLength = int(bounds[3]/spaceStep)
numStates = xLength*yLength
grid = np.array(
    [xLength*i + [j for j in np.arange(xLength)] for i in np.arange(yLength) ]
)

# Cross check the grid setup.
print(f"Number of possible states (grid size): {numStates}")
print(f"Grid shape: {grid.shape}")



# fig, ax = plt.subplots(2,2)
# fig.suptitle('Robot Planned Paths')
# plt.setp(ax, 
#         xticks=np.arange(bounds[0], bounds[1]+1, spaceStep),
#         yticks=np.arange(bounds[2], bounds[3]+1, spaceStep)
# )


# path = np.array([0, 4, 55, 106, 110, 183, 186, 190, 189, 143, 190, 186, 183, 179, 152, 125, 128, 178, 181, 178, 105, 179, 109, 39, 62, 58, 103, 148, 74, 0])

# # 15 step horizon. 4-robot, 2 visit RF Welder, 1 visit Sewing Machine, 1 stay put.
# path = [[120, 75, 146, 150, 152, 152, 152, 152, 152, 152, 152, 152, 152, 152, 105],
#                 [145, 121, 122, 148, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 103],
#                 [119, 119, 119, 119, 119, 119, 119, 119, 119, 119, 119, 119, 119, 119, 119],
#                 [23, 19, 40, 110, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 10]]

# 30 step horizon. 3-robot, 2 visit RF Welder.
path = [[120, 74, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 48, 0, 96, 169, 148, 152, 152, 152, 152, 152, 150, 124, 120],
        [119, 167, 167, 167, 116, 162, 163, 167, 164, 113, 86, 182, 156, 60, 14, 37, 58, 79, 175, 175, 175, 175, 175, 104, 106, 36, 107, 111, 115, 119],
        [23, 119, 190, 139, 88, 133, 178, 127, 76, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 25, 48, 75, 30, 57, 11, 61, 65, 19, 23]]

path = [[120, 74, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 48, 0, 96, 169, 148, 152, 152, 152, 152, 152, 150, 124, 120],
        [119, 167, 167, 167, 116, 162, 163, 167, 164, 113, 86, 182, 156, 60, 14, 37, 58, 79, 175, 175, 175, 175, 175, 104, 106, 36, 107, 111, 115, 119],
        [23, 119, 190, 139, 88, 133, 178, 127, 76, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 25, 48, 75, 30, 57, 11, 61, 65, 19, 23]]


path = np.array(path)
# print(pd.DataFrame(path.T))

# Distance in ft. Time in seconds.
bounds = [0, 120, 0, 40]
spaceStep = 5
timeStep = 10

numRobots = path.shape[0]
planHorizon = path.shape[1]
grid = create_grid(bounds, spaceStep)

animate_path(numRobots, planHorizon, path, grid, stations, obs, bounds, spaceStep)