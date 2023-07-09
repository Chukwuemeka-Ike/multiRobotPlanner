'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for extracting information from a generated schedule.
'''
import json
import pandas as pd

from arm_constants.stations import station_type_ws_nums, Mjs
from arm_utils.job_utils import *


def load_schedule(filename):
    '''.'''
    schedule = pd.read_csv(filename, index_col=0)

    # Parents list gets saved as a string. Convert it back to a list.
    schedule["parents"] = schedule["parents"].apply(lambda x:json.loads(x))

    # Ensure start and end times are floats.
    schedule["start"] = schedule["start"].apply(lambda x:float(x))
    schedule["end"] = schedule["end"].apply(lambda x:float(x))

    return schedule

def extract_schedule_lp(X, S, C, job_list: list, all_machines: list, station_type_names: list):
    '''Extracts and formats the schedule solved by the LP solver and job data.

    Returns:
        Schedule - DataFrame of Job-Task schedule with start and end times.
    '''
    jobs, tasks = [], []
    locations, station_nums, station_type_nums = [], [], []
    ticket_ids, parentses = [], []
    starts, ends, durations, time_lefts = [], [], [], []

    for job_idx, job in enumerate(job_list):
        for task_idx, task in enumerate(job):
            for machine in all_machines:
                if X[job_idx, task_idx, machine].solution_value() > 0.5:
                    # jobs.append(job_idx)
                    jobs.append(task["job_id"])
                    tasks.append(task_idx)
                    ticket_ids.append(task["ticket_id"])
                    parentses.append(task["parents"])
                    station_nums.append(machine)
                    station_type_nums.append(task["station_type"])
                    locations.append(station_type_names[task["station_type"]])
                    starts.append(int(S[job_idx, task_idx, machine].solution_value()))
                    ends.append(int(C[job_idx, task_idx, machine].solution_value()))
                    durations.append(task["duration"])
                    time_lefts.append(task["time_left"])

    schedule = pd.DataFrame()
    schedule["job_id"] = jobs
    schedule["task_idx"] = tasks
    schedule["ticket_id"] = ticket_ids
    schedule["parents"] = parentses
    schedule["station_num"] = station_nums
    schedule["station_type"] = station_type_nums
    schedule["location"] = locations
    schedule["start"] = starts
    schedule["end"] = ends
    schedule["duration"] = durations
    schedule["time_left"] = time_lefts

    return schedule

def extract_schedule_cpsat(solver, X, S, C, job_list: list, all_machines: list, station_type_names: list):
    '''Extracts and formats the schedule solved by the CP-SAT solver and job data.

    Returns:
        Schedule - DataFrame of Job-Task schedule with start and end times.
    '''
    jobs, tasks = [], []
    locations, station_nums, station_type_nums = [], [], []
    ticket_ids, parentses = [], []
    starts, ends, durations, time_lefts = [], [], [], []

    for job_idx, job in enumerate(job_list):
        for task_idx, task in enumerate(job):
            for machine in all_machines:
                if solver.Value(X[job_idx, task_idx, machine]) > 0.5:
                    # jobs.append(job_idx)
                    jobs.append(task["job_id"])
                    tasks.append(task_idx)
                    ticket_ids.append(task["ticket_id"])
                    parentses.append(task["parents"])
                    station_nums.append(machine)
                    station_type_nums.append(task["station_type"])
                    locations.append(station_type_names[task["station_type"]])
                    starts.append(solver.Value(S[job_idx, task_idx, machine]))
                    ends.append(solver.Value(C[job_idx, task_idx, machine]))
                    durations.append(task["duration"])
                    time_lefts.append(task["time_left"])

    schedule = pd.DataFrame()
    schedule["job_id"] = jobs
    schedule["task_idx"] = tasks
    schedule["ticket_id"] = ticket_ids
    schedule["parents"] = parentses
    schedule["station_num"] = station_nums
    schedule["station_type"] = station_type_nums
    schedule["location"] = locations
    schedule["start"] = starts
    schedule["end"] = ends
    schedule["duration"] = durations
    schedule["time_left"] = time_lefts

    return schedule

def convert_schedule_to_task_list(schedule: pd.DataFrame):
    '''Converts a given schedule to a task list.
    Returns:
        Task_list - dictionary of tickets with ticket ID's as keys.
    '''
    task_list = {}
    for idx in schedule.index:
        row = schedule.iloc[idx]
        ticket = {}
        ticket["job_id"] = row["job_id"]
        ticket["task_idx"] = row["task_idx"]
        ticket["parents"] = row["parents"]
        ticket["station_num"] = row["station_num"]
        ticket["station_type"] = row["station_type"]
        ticket["location"] = row["location"]
        ticket["start"] = row["start"]
        ticket["end"] = row["end"]
        ticket["duration"] = row["duration"]
        ticket["time_left"] = row["duration"]

        # Key is ticket id.
        task_list[row["ticket_id"]] = ticket

    return task_list


def extract_labor_schedule(R, S, C, job_list, all_robots):
    ''''''
    jobs, tasks = [], []
    robot_nums, station_type_nums = [], []
    ticket_ids, parentses = [], []
    starts, ends, durations = [], [], []

    for job_idx, job in enumerate(job_list):
        for task_idx, task in enumerate(job):
            for robot in all_robots:
                if R[job_idx, task_idx, robot].solution_value() > 0.5:
                    jobs.append(job_idx)
                    tasks.append(task_idx)
                    ticket_ids.append(task["ticket_id"])
                    robot_nums.append(robot)
                    station_type_nums.append(task["station_type"])
                    starts.append(int(S[job_idx, task_idx, robot].solution_value()))
                    ends.append(int(C[job_idx, task_idx, robot].solution_value()))
                    durations.append(task["duration"])

    schedule = pd.DataFrame()
    schedule["job_id"] = jobs
    schedule["ticket_id"] = ticket_ids
    schedule["robot_num"] = robot_nums
    schedule["station_type"] = station_type_nums
    schedule["start"] = starts
    schedule["end"] = ends
    schedule["duration"] = durations

    return schedule


def get_total_idle_time(schedule, job_list, parent_ids):
    '''Gets the total idle time between connected tasks in the schedule.'''
    sum = 0
    for i in range(len(job_list)):
        job = schedule.loc[schedule["job_id"] == i]

        for j in range(len(job)):
            task = job.iloc[j]
            for parent in parent_ids[i][j]:
                sum += (task["start"] - job.iloc[parent, 8])
    return sum
