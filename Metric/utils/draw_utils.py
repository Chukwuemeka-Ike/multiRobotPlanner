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


from utils.grid_utils import get_indices, find_shortest_path


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

def get_rectangle_center(x, grid, spaceStep):
    ''''''
    yLength, _ = grid.shape
    m, n = get_indices(x, grid)
    halfStep = spaceStep/2
    return (n*spaceStep)+halfStep, ((yLength-m-1)*spaceStep)+halfStep

def draw_obstacle(x, grid, spaceStep, ax):
    '''Draws obstacles in red at the x points in the grid.'''
    draw_rectangle(x, grid, spaceStep, ax, 'red')

def draw_station(x, grid, spaceStep, ax):
    '''Draws stations in blue at the x points in the grid.
    '''
    draw_rectangle(x, grid, spaceStep, ax, 'blue')

def draw_map(bounds, spaceStep):
    '''Draws the grid specified by bounds and spaceStep.'''
    fontSize = 20

    plt.figure(figsize=(17,7)) 
    plt.axis(bounds)
    
    plt.xticks(np.arange(bounds[0], bounds[1]+1, spaceStep), fontsize=fontSize)
    plt.yticks(np.arange(bounds[2], bounds[3]+1, spaceStep), fontsize=fontSize)
    
    plt.xlabel('(feet)', fontsize=fontSize)
    plt.ylabel('(feet)', fontsize=fontSize)
    plt.grid()
    
    return plt.gca()

def draw_obstacles(obstacles, grid, spaceStep, ax):
    for x in obstacles:
        draw_obstacle(x, grid, spaceStep, ax)

def draw_stations(stations, grid, spaceStep, ax):
    for x in stations:
        draw_station(x, grid, spaceStep, ax)

def draw_unconnected_path(path, grid, spaceStep, ax):
    '''Draws the unconnected path for a single robot on the grid provided.'''
    for k in range(path.size):
        draw_rectangle(path[k], grid, spaceStep, ax)

def draw_connected_path(path, grid, reach, previous, spaceStep, ax):
    '''Draws the connected path for a single robot on the grid provided.'''
    for k in range(path.size):
        if k < path.size-1:
            # Find the shortest connected path between path[k] and path[k+1].
            connector = find_shortest_path(
                    path[k], 
                    path[k+1], 
                    reach[path[k]], 
                    previous[path[k]]
                )
            # Draw the 
            for entry in connector:
                draw_rectangle(entry, grid, spaceStep, ax)
        draw_rectangle(path[k], grid, spaceStep, ax)





'''
    Functions for animating the path for n >= 1 robots.
    TODO: Clean up the workflow.
'''
def draw_env(grid, stations, obs, bounds, spaceStep, ax, fontSize):
    '''Draws the grid specified by bounds and spaceStep.'''

    plt.axis(bounds)
    
    plt.xticks(np.arange(bounds[0], bounds[1]+1, spaceStep), fontsize=fontSize)
    plt.yticks(np.arange(bounds[2], bounds[3]+1, spaceStep), fontsize=fontSize)
    
    plt.xlabel('(feet)', fontsize=fontSize)
    plt.ylabel('(feet)', fontsize=fontSize)

    draw_obstacles(obs, grid, spaceStep, ax)
    draw_stations(stations, grid, spaceStep, ax)

    plt.grid()

def draw_robot_positions(positions, grid, colors, numRobots, spaceStep, ax):
    '''Draw the robots' current positions in the grid.'''
    for robot in range(numRobots):
        draw_rectangle(positions[robot], grid, spaceStep, ax, colors[robot])

def draw_robot_trails(numRobots, timeIdx, ax, line_x, line_y, colors):
    '''Draw the trails behind each robot in the grid.'''
    trail_len = 3

    for robot in range(numRobots):
        if timeIdx >= trail_len:
            startIdx = timeIdx+1-trail_len
            ax.plot(
                line_x[robot][startIdx:timeIdx+1],
                line_y[robot][startIdx:timeIdx+1],
                colors[robot]
            )
        else:
            ax.plot(
                line_x[robot][:timeIdx+1],
                line_y[robot][:timeIdx+1],
                colors[robot]
            )


def animate_path(numRobots, planHorizon, path, grid, stations, obs, bounds, spaceStep, saveFilename="Videos/animatedPath.mp4"):
    ''''''
    fig, ax = plt.subplots(1, 1, figsize=(17,7))
    fontSize = 20
    colors = ["m", "k", "g", "y", "b", "r"]

    # Generate the center points for all path locations for all robots.
    line_x = [[] for _ in range(numRobots)]
    line_y = [[] for _ in range(numRobots)]

    for i in range(numRobots):
        for k in range(planHorizon):
            center_point = get_rectangle_center(path[i][k], grid, spaceStep)
            line_x[i].append(center_point[0])
            line_y[i].append(center_point[1])
    
    metadata = dict(title=f"Robot paths for {numRobots} robots over {planHorizon}.")
    writer = FFMpegWriter(fps=5, metadata=metadata)
    
    with writer.saving(fig, saveFilename, 100):
        for k in range(planHorizon):
            draw_env(grid, stations, obs, bounds, spaceStep, ax, fontSize)
            draw_robot_positions(path[:, k], grid, colors, numRobots, spaceStep, ax)
            draw_robot_trails(numRobots, k, ax, line_x, line_y, colors)

            plt.title(f"Timestep {k+1}.", fontsize = fontSize)
            plt.draw()
            # plt.pause(0.1)
            writer.grab_frame()

            if k < planHorizon-1:
                ax.clear()
    
        writer.grab_frame()
    # plt.show()



