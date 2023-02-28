'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for getting distance, locations, neighbors in the grid.
'''
import numpy as np
import z3


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

def create_reach_set(grid, obs, stations, timeStep, spaceStep, robotSpeed):
    '''Generates the set of reachable locations lookup table.

    Each row corresponds to an integer location.
    i.e. Reach(position x) is a list at index x.
    '''
    numStates = grid.size
    reach, distances, previous = [], [], []

    for i in range(numStates):
        reachable_set, dist, prev = get_reach(i, grid, obs, stations, timeStep,
                                            spaceStep, robotSpeed)
        reach.append(reachable_set)
        distances.append(dist)
        previous.append(prev)

    return reach, distances, previous

def create_adjacent_reach_set(grid, obstacles, stations):
    '''Generates the set of reachable locations lookup table.
    The only reachable points here are adjacent (8-point neighbors) cells.
    '''
    reach = []
    for i in range(grid.size):
        neighbors = get_neighbors(i, grid, obstacles, stations)
        reach.append(neighbors)
    return reach

def extract_path(model: z3.ModelRef, robotPos: z3.ArithRef, planHorizon: int, numBots: int):
    '''D.'''
    return np.array([[
        model[robotPos[k][i]].as_long() for k in range(planHorizon)
        ] for i in range(numBots)])

def extract_occupied(model: z3.ModelRef, occupied: z3.BoolRef, planHorizon: int, numBots: int):
    '''D.'''
    return np.array([[
        model[occupied[k][i]] for k in range(planHorizon)
        ] for i in range(numBots)])

# TODO: Optimize this.
def get_indices(x: int, grid: np.ndarray):
    '''Gets the indices of x in grid.'''
    idx = np.where(grid == x)
    try:
        return [idx[0][0], idx[1][0]]
    except:
    # m, n = grid.shape
    # for i in range(m):
    #     for j in range(n):
    #         if grid[i, j] == x:
    #             return [i, j]
        raise ValueError(f"The value {x} is not in the provided grid.")

def get_distance(x: int, y: int, grid: np.ndarray, ds: int):
    '''Gets the distance between two adjacent cells.
        Side-by-side cells are 1*ds apart, while diagonal
        neighbors are 1.4*ds apart.
    '''
    xx, xy = get_indices(x, grid)
    yx, yy = get_indices(y, grid)

    xDiff = abs(xx - yx)
    yDiff = abs(xy - yy)

    if xDiff + yDiff == 2:
        return float(1.4*ds)
    elif xDiff + yDiff == 1:
        return float(1*ds)
    else:
        raise ValueError(f"The values {x} and {y} are not adjacent "
                          "in the provided grid.")

# TODO: Optimize this.
def get_neighbors(x, grid, obs, stations):
    '''Gets the 8-point neighbors (diagonal included) of x in the grid.'''
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

    # # Add the current position as reachable.
    # neighbors.append(grid[i,j])

    for x in stations:
        if x in neighbors:
            neighbors.remove(x)
    for x in obs:
        if x in neighbors:
            neighbors.remove(x)
    return neighbors

def get_reach(x: int, grid: np.ndarray, obs: list,
        stations: list, dt: int, ds: int, speed: int):
    '''Gets the cells within reach of x based on dt and robot speed.

    Uses a truncated version of Dijkstra's algorithm to find all points
    in the grid that are within reach of the robot if it is in position x.
    '''
    # How far the robot can reach in one timestep.
    reachable = dt*speed
    # print(f"Reachable distance: {reachable}")

    # Max distance assigned to all grid points before their real distance
    # is calculated.
    maxDist = 20000
    dist = maxDist*np.ones((grid.size,1), dtype=float)
    prev = np.empty((grid.size,1))
    unvisited = [i for i in range(grid.size)]

    # Set the distance from the source to itself as 0.
    dist[x] = 0

    i = 0
    while np.amin(dist[unvisited]) <= reachable:
        u_idx = np.argmin(dist[unvisited])
        u = unvisited[u_idx]
        del unvisited[u_idx]

        neighbors = get_neighbors(u, grid, obs, stations)

        for neighbor in neighbors:
            alt = dist[u] + get_distance(neighbor, u, grid, ds)

            if alt < dist[neighbor]:
                dist[neighbor] = alt
                prev[neighbor] = u
        i = i + 1

    # Select only indices that are reachable.
    reachable_set = np.where(dist <= reachable)[0]
    dist = dist[reachable_set].reshape(-1)
    prev = prev[reachable_set].reshape(-1)

    return reachable_set, dist, prev

def find_shortest_path(x, y, reach, prev):
    '''Finds the shortest path from x to y given the reach and prev.'''
    path = []
    a = y

    while a != x:
        idx = np.where(reach == a)[0][0]
        a = prev[idx]
        path.append(a)
    return path


