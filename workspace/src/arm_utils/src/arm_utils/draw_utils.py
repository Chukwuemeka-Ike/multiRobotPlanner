'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for graphing the schedule.
'''
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd

from dateutil import tz
from matplotlib.dates import DateFormatter

from arm_utils.conversion_utils import convert_schedule_to_task_list
from arm_utils.job_utils import create_linear_jobs, get_job_subsizes


def draw_rectangle(
        x: int, y: int, width: int, height: int, facecolor: str, ax: plt.axes
    ) -> patches.Rectangle:
    '''Draw a rectangle at the x-y location of the given width.'''
    # Create the rectangle and add it to the axes object.
    rectangle = patches.Rectangle(
            (x, y), width,
            height, linewidth=3.5,
            edgecolor="#5e2222",
            facecolor=facecolor
        )
    ax.add_patch(rectangle)
    return rectangle

def draw_ticket(job_num: int, machine_type_num: int, machine_id: int,
                   start: int, duration: int, ax: plt.axes,
                   machine_type_indices: list, machine_type_abvs: list,
                   fontsize: int
    ) -> None:
    ''' given by the start-job numbers.

    Args:
        job_num:
        machine_type_num:
        machine_id:
        start:
        duration:
        ax:
        machine_type_indices:
        machine_type_abvs:
    '''
    rectangle = draw_rectangle(
        start, job_num, duration, 1, f"C{machine_type_num}", ax
    )
    rx, ry = rectangle.get_xy()
    cx = rx + rectangle.get_width()/2.0
    cy = ry + rectangle.get_height()/2.0
    ax.annotate(
        f"{machine_type_abvs[machine_type_num]} " +
        f"{str(machine_type_indices[machine_id])}",
        (cx, cy),
        color='black', weight='bold',
        fontsize=fontsize, ha='center', va='center'
    )

def draw_job_id(
        x: int, y: int, width: int, height: int,
        job_id: int, fontsize: int, ax: plt.axes
    ) -> None:
    '''.'''
    rectangle = draw_rectangle(x, y, width, height, 'black', ax)
    rx, ry = rectangle.get_xy()
    cx = rx + rectangle.get_width()/2.0
    cy = ry + rectangle.get_height()/2.0
    ax.annotate(
        f"{job_id}",
        (cx, cy),
        color='white', weight='bold',
        fontsize=fontsize, ha='center', va='center'
    )

def draw_env(
        bounds: list, fontSize: int, timezone: tz.tzlocal, ax: plt.axes
    ) -> None:
    '''Draw the base environment for the schedule to be graphed.'''
    ax.axis([bounds[0], bounds[1], bounds[2], bounds[3]])

    # Format to hour and minute only.
    formatter = DateFormatter("%H:%M", tz=timezone)
    ax.xaxis.set_major_formatter(formatter)
    ax.set_xlabel('Time', fontsize=fontSize)

    ax.set_yticks([])
    ax.set_ylabel('Job', fontsize=fontSize)
    ax.grid(axis='both')

def draw_no_schedule(ax: plt.Axes, current_time: int) -> None:
    '''Sets the bounds when there is no schedule to draw.'''
    current_time = pd.to_datetime(current_time, unit='s')
    maxY = 4
    bounds = [
        # (current_time - pd.Timedelta(hours=1)).to_pydatetime(),
        # (current_time + pd.Timedelta(hours=7)).to_pydatetime(),
        # Minutes for testing.
        (current_time - pd.Timedelta(minutes=1)).to_pydatetime(),
        (current_time + pd.Timedelta(minutes=7)).to_pydatetime(),
        maxY, 0
    ]
    fontSize = 10

    # Draw the base graph.
    draw_env(bounds, fontSize, ax)
    
    # Draw the "now" line.
    ax.axvline(x=current_time, color='r', linestyle='-', lw=5)

    ax.fill_betweenx(
        [maxY, 0],
        bounds[0], current_time,
        color='gray', alpha=0.5, label='Overlay')

def draw_schedule(
        schedule: pd.DataFrame,
        ongoing: list,
        machine_type_indices: list,
        machine_type_abvs: list,
        ax: plt.Axes,
        current_time: int,
        draw_full_schedule: bool=False
    ) -> None:
    '''Draws the schedule.'''
    # Bounds are jobs on y-axis, time index on x-axis. y-axis is flipped.
    # Each job on the y-axis may have more than one row.
    task_list = convert_schedule_to_task_list(schedule)
    num_jobs = schedule["job_id"].nunique()
    job_ids = schedule["job_id"].unique()
    sizes = get_job_subsizes(schedule, task_list)

    maxY = 0
    for i in range(num_jobs):
        maxY += sizes[i]

    local_timezone = tz.tzlocal()
    schedule["start"] = pd.to_datetime(schedule["start"], unit='s', utc=True).dt.tz_convert(local_timezone)
    schedule["end"] = pd.to_datetime(schedule["end"], unit='s', utc=True).dt.tz_convert(local_timezone)
    current_time = pd.to_datetime(current_time, unit='s', utc=True).tz_convert(local_timezone)

    if not draw_full_schedule:
        bounds = [
            # (current_time - pd.Timedelta(hours=1)).to_pydatetime(),
            # (current_time + pd.Timedelta(hours=7)).to_pydatetime(),
            # Minutes for testing.
            (current_time - pd.Timedelta(minutes=1)).to_pydatetime(),
            (current_time + pd.Timedelta(minutes=7)).to_pydatetime(),
            maxY, 0
        ]
        fontSize = 10
    else:
        bounds = [
            schedule["start"].min().to_pydatetime(),
            schedule["end"].max().to_pydatetime(),
            maxY, 0
        ]
        fontSize = 10
    # job_id_width = pd.Timedelta(minutes=6)
    job_id_width = pd.Timedelta(seconds=15)
    bounds[0] = bounds[0] - job_id_width

    # Draw the base graph.
    draw_env(bounds, fontSize, local_timezone, ax)

    # Draw rectangles for every task in the schedule.
    j = 0
    for job_id in job_ids:
        job = schedule.loc[schedule["job_id"] == job_id]
        linear_jobs = create_linear_jobs(job, task_list)

        for linear_job in linear_jobs:
            for ticket_id in linear_job:
                ticket = task_list[ticket_id]
                start_time = pd.to_datetime(ticket["start"], unit='s')
                end_time = pd.to_datetime(ticket["end"], unit='s')
                time_left = pd.Timedelta(ticket["time_left"], unit='s')

                # If a ticket is not done, its time left is useful.
                # If done, it's 0 and we use the relationship btw start & end.
                if ticket_id in ongoing:
                    duration = (current_time+time_left)-start_time
                else:
                    duration = end_time-start_time
                draw_ticket(
                    j, ticket["machine_type"],
                    ticket["machine_id"],
                    start_time,
                    duration,
                    ax,
                    machine_type_indices,
                    machine_type_abvs,
                    fontSize
                )
            j+=1

    # Draw horizontal lines separating each tree job.
    runTotal = 0
    for job_num in range(num_jobs):
        draw_job_id(
            bounds[0], runTotal, 
            job_id_width,
            sizes[job_num],
            job_ids[job_num],
            fontSize,
            ax
        )
        runTotal += sizes[job_num]
        ax.axhline(y = runTotal, color='k', linestyle='-', lw=5)

    # Draw the "now" line.
    ax.axvline(x=current_time, color='r', linestyle='-', lw=5)

    ax.fill_betweenx(
        [maxY, 0],
        bounds[0], current_time,
        color='gray', alpha=0.5, label='Overlay')
