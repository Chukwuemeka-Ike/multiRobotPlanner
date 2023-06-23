'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for graphing, decorating, and saving the assignment and schedule.
'''
import glob
import json
import math
from matplotlib.animation import FFMpegWriter
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import pandas as pd

# from constants.stations import Mjs, station_type_abvs
from constants.stations_old import Mjs, station_type_abvs
from utils.job_utils import *
from utils.sched_utils import convert_schedule_to_task_list, load_schedule


def draw_rectangle(job_num: int, station_type_num: int, station_num: int, start: int, duration: int, ax: plt.axes):
    '''Draw a rectangle at the x-y location given by the start-job numbers.'''
    colors = ["#2f2f2f","#dc0001","#176d14","#006cdc","#b0b0b0"]
    # Create the rectangle and add it to the axes object.
    rectangle = patches.Rectangle(
            (start, job_num),
            duration, 1, linewidth=3.5,
            edgecolor="#5e2222",
            # facecolor=colors[station_type_num],
            facecolor=f"C{station_type_num}"
            # linewidth=0.25
        )
    ax.add_patch(rectangle)
    rx, ry = rectangle.get_xy()
    cx = rx + rectangle.get_width()/2.0
    cy = ry + rectangle.get_height()/2.0
    ax.annotate(
        f"{station_type_abvs[station_type_num]} {str(Mjs[station_num]+1)}",
        # station_num,
        (cx, cy),
        color='black', weight='bold',
        fontsize=10, ha='center', va='center'
    )

def draw_env(bounds: list, fontSize: int):
    '''Draw the base environment for the schedule to be graphed.'''
    plt.axis([bounds[0], bounds[1], bounds[2], bounds[3]])
    numXTicks = 10
    number = int(bounds[1]/numXTicks)
    # print(bounds)
    # print(number)
    xSpacing = math.ceil(number/10)*10      # Round up to nearest 10.
    # xSpacing = round(number, -1)            # Round to nearest 10.
    # print(xSpacing)
    plt.xticks(np.arange(bounds[0], bounds[1], xSpacing), fontsize=fontSize)
    plt.yticks(np.arange(bounds[3], bounds[2], 1), fontsize=fontSize)
    plt.xlabel('Time (mins)', fontsize=fontSize)
    plt.ylabel('Job Branch', fontsize=fontSize)
    plt.grid()

def draw_linear_schedule(schedule: pd.DataFrame):
    '''Draws a schedule of only linear jobs.'''
    # Bounds are jobs on y-axis, time index on x-axis. y-axis is flipped.
    bounds = [0, schedule["end"].max()+1, schedule["job_id"].max()+1, 0]
    fontSize = 20

    # Create the axes and draw the base graph.
    _, ax = plt.subplots(1, 1, figsize=(17,7))
    draw_env(bounds, fontSize)

    # Draw rectangles for every task in the schedule.
    for i in range(len(schedule.index)):
        row = schedule.iloc[i]
        draw_rectangle(
            row["job_id"], row["station_type"],
            row["station_num"],
            row["start"], row["duration"], 
            ax
        )
    plt.show()

def draw_tree_schedule(schedule: pd.DataFrame, ax):
    '''Draws a schedule including tree jobs.'''
    # Bounds are jobs on y-axis, time index on x-axis. y-axis is flipped.
    # Each job on the y-axis may have more than one row.
    task_list = convert_schedule_to_task_list(schedule)
    num_jobs = schedule["job_id"].nunique()
    job_ids = schedule["job_id"].unique()
    sizes = get_job_subsizes(schedule, task_list)

    maxY = 0
    for i in range(num_jobs):
        maxY += sizes[i]

    bounds = [0, schedule["end"].max()+1, maxY, 0]
    fontSize = 15

    # Create the axes and draw the base graph.
    # _, ax = plt.subplots(1, 1, figsize=(17,7))
    draw_env(bounds, fontSize)

    # Draw rectangles for every task in the schedule.
    j = 0
    for job_id in job_ids:
        job = schedule.loc[schedule["job_id"] == job_id]
        linear_jobs = create_linear_jobs(job, task_list)
        # print(linear_jobs)

        for _, linear_job in enumerate(linear_jobs):
            for ticket_id in linear_job:
                ticket = task_list[ticket_id]
                draw_rectangle(
                    j, ticket["station_type"],
                    ticket["station_num"],
                    ticket["start"], ticket["duration"], 
                    ax
                )
            j+=1

    # Draw horizontal lines separating each tree job.
    runTotal = 0
    for job_num in range(num_jobs):
        runTotal += sizes[job_num]
        plt.axhline(y = runTotal, color = 'k', linestyle = '-', lw=5)
    # plt.savefig(scheduleFilename)
    # plt.show()

def draw_evolving_schedule(search_dir: str, saveFilename: str="animatedSched.mp4"):
    '''Draws a schedule including tree jobs.'''
    # Get the number of schedule files and load the save times for each.
    search_word = "schedule"
    file_paths = [file for file in glob.glob(search_dir + '/*' + search_word + '*')]
    num_scheds = len(file_paths)
    print(num_scheds)

    with open(f"{search_dir}/sched_times.txt", 'r') as f:
        times = [float(line.strip()) for line in f.readlines()]
    times = [t - times[0] for t in times]

    # Load the actual executed schedule.
    schedule = load_schedule(f"{search_dir}/savedSched.csv")
    min_time = schedule["start"].min()
    schedule["end"] = schedule["end"] - min_time
    actual_schedule_end = schedule["end"].max()+1

    # Create the axes and draw the base graph.
    fig, ax = plt.subplots(1, 1, figsize=(17,7))
    fontSize = 20

    metadata = dict(
        title=f"Evolving schedule.")
    writer = FFMpegWriter(fps=0.5, metadata=metadata)

    with writer.saving(fig, saveFilename, 100):
        for i in range(num_scheds):
            # print(i)
            schedule = load_schedule(f"{search_dir}/schedule{i+1}.csv")
            schedule["start"] = schedule["start"] + times[i]
            schedule["end"] = schedule["end"] + times[i]
            schedule["duration"] = schedule.apply(lambda x: x["end"]-x["start"], axis=1)

            task_list = convert_schedule_to_task_list(schedule)

            num_jobs = schedule["job_id"].nunique()
            job_ids = schedule["job_id"].unique()
            sizes = get_job_subsizes(schedule, task_list)
            bounds = [0, actual_schedule_end, sum(sizes), 0]
            # print(schedule)

            # Draw rectangles for every task in the schedule.
            draw_env(bounds, fontSize)
            j = 0
            for job_id in job_ids:
                job = schedule.loc[schedule["job_id"] == job_id]
                linear_jobs = create_linear_jobs(job, task_list)
                # print(linear_jobs)

                for _, linear_job in enumerate(linear_jobs):
                    for ticket_id in linear_job:
                        ticket = task_list[ticket_id]
                        draw_rectangle(
                            j, ticket["station_type"],
                            ticket["station_num"],
                            ticket["start"], ticket["time_left"], 
                            ax
                        )
                    j+=1

            # Draw horizontal lines separating each tree job.
            runTotal = 0
            for job_num in range(num_jobs):
                runTotal += sizes[job_num]
                plt.axhline(y = runTotal, color = 'k', linestyle = '-', lw=5)
            plt.title(f"Schedule at T = {times[i]}.", fontsize = fontSize)
            plt.draw()

            writer.grab_frame()

            if i < num_scheds - 1:
                ax.clear()

        # Add a frame, so the video doesn't end abruptly on the last update.
        writer.grab_frame()

def draw_labor_schedule(labor_schedule: pd.DataFrame, all_robots: list, scheduleFilename: str="Images/laborSched.png"):
    '''Draws the schedule for each labor unit.'''
    num_robots = len(all_robots)
    bounds = [0, labor_schedule["end"].max()+1, num_robots, 0]
    fontSize = 20

    # Create the axes and draw the base graph.
    _, ax = plt.subplots(1, 1, figsize=(17,7))
    draw_env(bounds, fontSize)

    for robot in range(num_robots):
        robot_assigned = labor_schedule.loc[labor_schedule["robot_num"] == robot]
        for assignment in range(len(robot_assigned)):
            row = robot_assigned.iloc[assignment]
            draw_rectangle(
                robot, row["station_type"],
                row["ticket_id"],
                row["start"], row["duration"],
                ax
            )
    plt.savefig(scheduleFilename)
    # plt.show()