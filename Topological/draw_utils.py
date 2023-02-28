'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for graphing and decorating the grid environment.
'''
from matplotlib.animation import FFMpegWriter
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from locations import *
from grid_utils import get_indices


def get_rectangle_corner(x, grid, spaceStep):
    ''''''
    yLength, _ = grid.shape
    m, n = get_indices(x, grid)
    return (n*spaceStep), ((yLength-m-1)*spaceStep)

def draw_rectangle(x, grid, spaceStep, ax, color='green'):
    '''Draws a rectangle at the x point in the grid.'''
    yLength, _ = grid.shape
    m, n = get_indices(x, grid)

    # Draw the Rectangle patch.
    # x and y positions are flipped when drawing, and
    # y is mirrored around the x-axis.
    rect = patches.Rectangle(
        ((n)*spaceStep, (yLength-m-1)*spaceStep),
        spaceStep, spaceStep, linewidth=1,
        edgecolor='none', facecolor=color)

    # Add the patch to the axes object.
    ax.add_patch(rect)


def draw_obstacle(x, grid, spaceStep, ax):
    '''Draws obstacles in red at the x points in the grid.'''
    draw_rectangle(x, grid, spaceStep, ax, 'red')

def draw_station(x, grid, spaceStep, ax):
    '''Draws stations in blue at the x points in the grid.
    '''
    draw_rectangle(x, grid, spaceStep, ax, 'blue')

def draw_obstacles(obstacles, grid, spaceStep, ax):
    for x in obstacles:
        draw_obstacle(x, grid, spaceStep, ax)

def draw_stations(stations, grid, spaceStep, ax):
    for x in stations:
        draw_station(x, grid, spaceStep, ax)








'''
    Functions for animating the path for n >= 1 robots.
    TODO: Clean up the workflow.
'''
def draw_env(grid, stations, obs, bounds, spaceStep, ax, fontSize):
    '''Draws the grid specified by bounds and spaceStep.'''

    plt.axis([bounds[0]-spaceStep, bounds[1], bounds[2], bounds[3]+(2*spaceStep)])
    
    plt.xticks(np.arange(bounds[0], bounds[1]+1, spaceStep), fontsize=fontSize)
    plt.yticks(np.arange(bounds[2], bounds[3]+1, spaceStep), fontsize=fontSize)
    
    plt.xlabel('(feet)', fontsize=fontSize)
    plt.ylabel('(feet)', fontsize=fontSize)

    draw_obstacles(obs, grid, spaceStep, ax)
    draw_stations(stations, grid, spaceStep, ax)

    plt.grid()

def draw_robot_positions(positions, offsets, grid, colors, numRobots, spaceStep, ax):
    ''''''
    for robot in range(numRobots):
        # m, n = get_indices(, grid)
        x, y = get_rectangle_corner(positions[robot], grid, spaceStep)
        ax.plot(
            x - 1, 
            y + offsets[robot],
            color=f"C{robot}",#colors[robot],
            marker='*',
            markersize=12
        )

def draw_robot_trails(numRobots, timeIdx, ax, line_x, line_y, colors):
    '''Draw the trails behind each robot in the grid.'''
    trail_len = 3

    for robot in range(numRobots):
        if timeIdx >= trail_len:
            startIdx = timeIdx+1-trail_len
            ax.plot(
                line_x[robot][startIdx:timeIdx+1],
                line_y[robot][startIdx:timeIdx+1],
                f"C{robot}"#colors[robot]
            )
        else:
            ax.plot(
                line_x[robot][:timeIdx+1],
                line_y[robot][:timeIdx+1],
                f"C{robot}"#colors[robot]
            )

def translate_path_to_locations(path, planHorizon, numRobots):
    ''''''
    locations = np.zeros((path.shape), dtype=float)

    for k in range(planHorizon):
        for i in range(numRobots):
            locations[i, k] = grid_locations[loc_names[path[i, k]-1]][0]
    return locations

def animate_path(
    numRobots: int, planHorizon: int, path: np.ndarray, grid: np.ndarray,
    stations: list, obs: list, bounds: list, spaceStep: int,
    saveFilename: str="Videos/animatedPathSparse.mp4"
):
    '''D.'''
    fig, ax = plt.subplots(1, 1, figsize=(17,7))
    fontSize = 20
    colors = ["m", "k", "g", "y", "b", "r", "c"]
    offsets = [1.5*(i+1) for i in range(numRobots)]

    # Generate the center points for all path locations for all robots.
    line_x = [[] for _ in range(numRobots)]
    line_y = [[] for _ in range(numRobots)]

    locations = translate_path_to_locations(path, planHorizon, numRobots)

    for i in range(numRobots):
        for k in range(planHorizon):
            x, y = get_rectangle_corner(locations[i, k], grid, spaceStep)
            line_x[i].append(x - 1)
            line_y[i].append(y + offsets[i])

    metadata = dict(title=f"Robot paths for {numRobots} robots over {planHorizon}.")
    writer = FFMpegWriter(fps=3, metadata=metadata)

    with writer.saving(fig, saveFilename, 100):
        for k in range(planHorizon):
            draw_env(grid, stations, obs, bounds, spaceStep, ax, fontSize)
            draw_robot_positions(locations[:, k], offsets, grid, colors, numRobots, spaceStep, ax)
            draw_robot_trails(numRobots, k, ax, line_x, line_y, colors)

            plt.title(f"Timestep {k+1}.", fontsize = fontSize)
            plt.draw()
            # plt.pause(0.1)
            writer.grab_frame()

            if k < planHorizon-1:
                ax.clear()
    # plt.show()
