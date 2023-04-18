'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for graphing and decorating the assignment and schedule.
'''
from matplotlib.animation import FFMpegWriter
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import pandas as pd

from constants import station_type_names, station_type_numbers, Mj

dx = 10
dxx = 1
dxt = 2
tWidth = 10
xGap = 5
dy = 5
dyy = 1
maxTickets = 10
y = dy*(maxTickets + 2)
queueColors = ["#2f2f2f","#dc0001","#176d14","#006cdc","#b0b0b0"]

def draw_queue_rectangle(queue_id, station_type_num, ax):
    '''.'''
    rectangle = patches.Rectangle(
        (queue_id*(dx + xGap), 0),
        dx, y,
        linewidth=1,
        edgecolor="none",
        facecolor=queueColors[station_type_num]
    )
    ax.add_patch(rectangle)
    rx, ry = rectangle.get_xy()
    cx = rx + rectangle.get_width()/2.0
    cy = ry + y - (dy/2)
    ax.annotate(
        f"Queue {queue_id}",
        (cx, cy),
        color='black', weight='bold',
        fontsize=10, ha='center', va='center'
    )


def draw_ticket_rectangle(queue_id, ticket_id, ticket_position, ax):
    '''.'''
    # if queue_id 
    xPos = queue_id*(dx + xGap) + dxx 
    yPos = (y-(ticket_position+2)*dy) - (ticket_position*dyy)

    rectangle = patches.Rectangle(
        (xPos, yPos),
        dx-(2*dxx), dy,
        linewidth=1,
        edgecolor="none",
        facecolor="#6d8ac9"
    )
    ax.add_patch(rectangle)
    rx, ry = rectangle.get_xy()
    cx = rx + rectangle.get_width()/2.0
    cy = ry + rectangle.get_height()/2.0
    ax.annotate(
        f"Ticket {ticket_id}",
        (cx, cy),
        color='black', weight='bold',
        fontsize=10, ha='center', va='center'
    )

def draw_time(num_queues, T, ax):
    '''.'''
    xPos = num_queues*(dx+xGap)-xGap+dxt
    yPos = y-dy
    rectangle = patches.Rectangle(
        (xPos, yPos),
        tWidth, dy,
        linewidth=1,
        edgecolor="k",
        facecolor="#FFFFFF"
    )
    ax.add_patch(rectangle)
    rx, ry = rectangle.get_xy()
    cx = rx + rectangle.get_width()/2.0
    cy = ry + rectangle.get_height()/2.0
    ax.annotate(
        f"T = {T}",
        (cx, cy),
        color='black', weight='bold',
        fontsize=10, ha='center', va='center'
    )

def draw_env(num_queues):
    '''.'''
    # Create the axes and draw the base graph.
    plt.axis([
        0,
        num_queues*(dx+xGap)-xGap+dxt+tWidth,
        0,
        y
    ])
    plt.tick_params(left = False, bottom = False,
                    labelleft = False, labelbottom = False
    )

def draw_queues(queueSchedule: pd.DataFrame):
    '''.'''
    # Time is the extra column.
    num_queues = len(queueSchedule.columns) - 1


    fig, ax = plt.subplots(1, 1, figsize=(17,7))
    timelineLength = len(queueSchedule.index)


    metadata = dict(
        title=f"Queue evolution.")
    writer = FFMpegWriter(fps=1, metadata=metadata)

    with writer.saving(fig, "Videos/queueSchedule.mp4", 100):

        # Draw the queued tickets.
        for idx, row in queueSchedule.iterrows():
            draw_env(num_queues)
            for station_type_num, station_type in enumerate(Mj):
                for station in station_type:
                    draw_queue_rectangle(station, station_type_num, ax)

            for queue_id in range(num_queues):
                queue = row[f"Queue {queue_id}"]
                for ticket_position, ticket_id in enumerate(queue):
                    draw_ticket_rectangle(
                        queue_id, ticket_id, ticket_position, ax
                    )
            draw_time(num_queues, row["t"], ax)
            plt.draw()
            writer.grab_frame()

            if idx < timelineLength - 1:
                ax.clear()

        writer.grab_frame()
    # plt.show()
