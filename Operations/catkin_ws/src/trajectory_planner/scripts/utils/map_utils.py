#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
'''

import numpy as np


def create_grid_map(bounds, spaceStep):
    '''Sets up a grid based on the space discretization and bounds given.'''
    # Check if the workspace divides nicely by the chosen space discretization.
    if (bounds[1] % spaceStep + bounds[3] % spaceStep) != 0:
        raise ArithmeticError(f"The chosen bounds {bounds[1], bounds[3]}"
            f"are not both divisible by the spaceStep {spaceStep}.\n")

    xLength = int(bounds[1]/spaceStep)
    yLength = int(bounds[3]/spaceStep)

    grid = np.array(
        [xLength*i + [j for j in np.arange(xLength)] for i in np.arange(yLength, -1, -1)]
    )
    return grid

def get_indices(x: int, grid: np.ndarray):
    '''Gets the indices of integer x in the grid.'''
    idx = np.where(grid == x)
    try:
        return [idx[0][0], idx[1][0]]
    except:
        raise ValueError(f"The value {x} is not in the provided grid.")

def get_coordinates(x: int, grid: np.ndarray, spaceStep: int):
    '''Gets the (x, y) coordinates of the integer x in the grid.'''
    yLength, _ = grid.shape
    m, n = get_indices(x, grid)
    return ((n*spaceStep)+spaceStep/2,
            ((yLength-m-1)*spaceStep)+spaceStep/2)

def get_neighbors(x: int, grid: np.ndarray, stations: list):
    '''Gets the 8-point neighbors (diagonal included) of x in the grid.'''
    # TODO: Optimize this.
    m, n = grid.shape
    i, j = get_indices(x, grid)

    neighbors = []
    if i > 0:
        neighbors.append(grid[i-1, j])
    if i+1 < m:
        neighbors.append(grid[i+1, j])
    if j > 0:
        neighbors.append(grid[i, j-1])
    if j+1 < n:
        neighbors.append(grid[i, j+1])
    if j+1 < n and i+1 < m:
        neighbors.append(grid[i+1, j+1])
    if j > 0 and i > 0:
        neighbors.append(grid[i-1, j-1])
    if j > 0 and i+1 < m:
        neighbors.append(grid[i+1, j-1])
    if j+1 < n and i > 0:
        neighbors.append(grid[i-1, j+1])

    for x in stations:
        if x in neighbors:
            neighbors.remove(x)
    return neighbors

def get_station_neighbors(station: list, grid: np.ndarray, stations: list):
    '''Gets the integer neighbors to the given station.

    Returns:
        Neighbors - list of integers representing 8-point neighbors of the
                    station in the grid.
    '''
    neighbors = []
    for x in station:
        neighbors.append(get_neighbors(x, grid, stations))
    neighbors = [i for j in neighbors for i in j]   # Flatten the list.
    return neighbors