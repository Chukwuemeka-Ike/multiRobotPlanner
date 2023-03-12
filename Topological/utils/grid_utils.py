'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for working with the grid.
'''
import numpy as np
from z3 import ModelRef, ArithRef, BoolRef


def create_grid(bounds, spaceStep):
    '''Sets up the grid based on the space discretization and bounds given.'''
    # Check if the workspace divides nicely by the chosen space discretization.
    if (bounds[1] % spaceStep + bounds[3] % spaceStep) != 0:
        raise ArithmeticError(f"The chosen bounds {bounds[1], bounds[3]}"
            f"are not both divisible by the spaceStep {spaceStep}.\n")

    xLength = int(bounds[1]/spaceStep)
    yLength = int(bounds[3]/spaceStep)

    grid = np.array(
        [xLength*i + [j for j in np.arange(xLength)] for i in np.arange(yLength)]
    )
    return grid

def extract_path(model: ModelRef, robotPos: ArithRef, planHorizon: int, numBots: int):
    '''Extracts the paths from the solved model.'''
    return np.array([[
        model[robotPos[k][i]].as_long() for k in range(planHorizon)
        ] for i in range(numBots)])

def extract_occupied(model: ModelRef, occupied: BoolRef, planHorizon: int, numBots: int):
    '''Extracts the occupied flags from the solved model.'''
    return np.array([[
        model[occupied[k][i]] for k in range(planHorizon)
        ] for i in range(numBots)])

def get_indices(x: int, grid: np.ndarray):
    '''Gets the indices of x in grid.'''
    idx = np.where(grid == x)
    try:
        return [idx[0][0], idx[1][0]]
    except:
        raise ValueError(f"The value {x} is not in the provided grid.")
