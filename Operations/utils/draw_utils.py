'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for graphing and decorating the assignment and schedule.
'''
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import pandas as pd


def draw_rectangle(job_num: int, station_type_num: int, start: int, duration: int, ax: plt.axes):
    '''Draw a rectangle at the x-y location given by the start-job numbers.'''
    # Create the rectangle and add it to the axes object.
    ax.add_patch(
        patches.Rectangle(
            (start, job_num),
            duration, 1, linewidth=1,
            edgecolor="none", facecolor=f"C{station_type_num}"
        )
    )

def draw_env(bounds: list, fontSize: int):
    '''Draw the base environment for the schedule to be graphed.'''
    plt.axis([bounds[0], bounds[1], bounds[2], bounds[3]])
    plt.xticks(np.arange(bounds[0], bounds[1], int(bounds[1]/10)), fontsize=fontSize)
    plt.yticks(np.arange(bounds[3], bounds[2], 1), fontsize=fontSize)
    plt.xlabel('Time Index (k)', fontsize=fontSize)
    plt.ylabel('Job #', fontsize=fontSize)
    plt.grid()

def draw_schedule(schedule: pd.DataFrame):
    '''Draw the schedule.'''
    # Bounds are jobs on y-axis, time index on x-axis. y-axis is flipped.
    bounds = [0, schedule["End"].max()+1, schedule["Job #"].max()+1, 0]
    fontSize = 20

    # Create the axes and draw the base graph.
    _, ax = plt.subplots(1, 1, figsize=(17,7))
    draw_env(bounds, fontSize)

    # Draw rectangles for every task in the schedule.
    for i in range(len(schedule.index)):
        row = schedule.iloc[i]
        draw_rectangle(
            row["Job #"], row["Station Type #"], 
            row["Start"], row["Duration"], 
            ax
        )
    plt.show()