'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for graphing and decorating the grid environment.
'''

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

from utils.map_utils import get_indices


def get_rectangle_center(x: int, grid: np.ndarray, spaceStep: int):
    '''Gets the center of the rectangle in the map.'''
    yLength, _ = grid.shape
    m, n = get_indices(x, grid)
    halfStep = spaceStep/2
    return (n*spaceStep)+halfStep, ((yLength-m-1)*spaceStep)+halfStep

def draw_rectangle(
    x: int, grid: np.ndarray, spaceStep: int, ax, color: str='green'
):
    '''Draws a rectangle at the x point in the grid.'''
    yLength, _ = grid.shape
    m, n = get_indices(x, grid)

    # Draw the Rectangle patch and add it to the axis object.
    # x and y positions are flipped when drawing, and
    # y is mirrored around the x-axis.
    ax.add_patch(
        patches.Rectangle(
            ((n)*spaceStep, (yLength-m-1)*spaceStep),
            spaceStep, spaceStep, linewidth=1,
            edgecolor='none', facecolor=color)
    )

def draw_obstacles(obstacles: list, grid: np.ndarray, spaceStep: int, ax):
    '''Draws obstacles in red at the x points in the grid.'''
    for x in obstacles:
        draw_rectangle(x, grid, spaceStep, ax, 'red')

def draw_stations(stations: list, grid: np.ndarray, spaceStep: int, ax):
    '''Draws stations in blue at the x points in the grid.'''
    for x in stations:
        draw_rectangle(x, grid, spaceStep, ax, 'blue')

def draw_env(
    grid: np.ndarray, stations: list, obs: list, bounds: list,
    spaceStep: int, ax, fontSize: int
):
    '''Draws the grid specified by bounds and spaceStep.'''

    plt.axis(bounds)

    plt.xticks(np.arange(bounds[0], bounds[1]+1, spaceStep), fontsize=fontSize)
    plt.yticks(np.arange(bounds[2], bounds[3]+1, spaceStep), fontsize=fontSize)

    plt.xlabel('(feet)', fontsize=fontSize)
    plt.ylabel('(feet)', fontsize=fontSize)

    draw_obstacles(obs, grid, spaceStep, ax)
    draw_stations(stations, grid, spaceStep, ax)

    plt.grid()
