'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for extracting information from a generated schedule.
'''
import json
import pandas as pd

from arm_constants.machines import machine_type_ws_nums, Mjs
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

def extract_schedule_lp(X, S, C, job_list: list, all_machines: list, machine_type_names: list):
    '''Extracts and formats the schedule solved by the LP solver and job data.

    Returns:
        Schedule - DataFrame of Job-Task schedule with start and end times.
    '''
    jobs, tasks = [], []
    locations, machine_nums, machine_type_nums = [], [], []
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
                    machine_nums.append(machine)
                    machine_type_nums.append(task["machine_type"])
                    locations.append(machine_type_names[task["machine_type"]])
                    starts.append(int(S[job_idx, task_idx, machine].solution_value()))
                    ends.append(int(C[job_idx, task_idx, machine].solution_value()))
                    durations.append(task["duration"])
                    time_lefts.append(task["time_left"])

    schedule = pd.DataFrame()
    schedule["job_id"] = jobs
    schedule["task_idx"] = tasks
    schedule["ticket_id"] = ticket_ids
    schedule["parents"] = parentses
    schedule["machine_num"] = machine_nums
    schedule["machine_type"] = machine_type_nums
    schedule["location"] = locations
    schedule["start"] = starts
    schedule["end"] = ends
    schedule["duration"] = durations
    schedule["time_left"] = time_lefts

    return schedule

def extract_schedule_cpsat(solver, X, S, C, job_list: list, all_machines: list, machine_type_names: list):
    '''Extracts and formats the schedule solved by the CP-SAT solver and job data.

    Returns:
        Schedule - DataFrame of Job-Task schedule with start and end times.
    '''
    jobs, tasks = [], []
    locations, machine_nums, machine_type_nums = [], [], []
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
                    machine_nums.append(machine)
                    machine_type_nums.append(task["machine_type"])
                    locations.append(machine_type_names[task["machine_type"]])
                    starts.append(solver.Value(S[job_idx, task_idx, machine]))
                    ends.append(solver.Value(C[job_idx, task_idx, machine]))
                    durations.append(task["duration"])
                    time_lefts.append(task["time_left"])

    schedule = pd.DataFrame()
    schedule["job_id"] = jobs
    schedule["task_idx"] = tasks
    schedule["ticket_id"] = ticket_ids
    schedule["parents"] = parentses
    schedule["machine_num"] = machine_nums
    schedule["machine_type"] = machine_type_nums
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
        # ticket["task_idx"] = row["task_idx"]
        ticket["parents"] = row["parents"]
        ticket["machine_num"] = row["machine_num"]
        ticket["machine_type"] = row["machine_type"]
        ticket["location"] = row["location"]
        ticket["start"] = row["start"]
        ticket["end"] = row["end"]
        ticket["duration"] = row["duration"]
        ticket["time_left"] = row["duration"]

        # Key is ticket id.
        task_list[row["ticket_id"]] = ticket

    return task_list

def convert_task_list_to_schedule(task_dict: dict, machine_type_names: list) -> pd.DataFrame:
    """Converts a task dict to a schedule.

    Returns
        schedule: DataFrame of the schedule contained in the task_dict.
    """
    if len(task_dict) == 0:
        return None

    schedule = pd.DataFrame(
            columns=[
                "job_id", "ticket_id", "parents", "machine_num",
                "machine_type", "location", "start", "end", "duration",
                "time_left"
            ]
    )
    jobs, ticket_ids, parentses = [], [], []
    locations, machine_nums, machine_type_nums = [], [], []
    starts, ends, durations, time_lefts = [], [], [], []

    for ticket_id, ticket in task_dict.items():
        # First try to get all the important data. If it doesn't fail,
        # Append each.
        try:
            job_id = ticket["job_id"]
            parents = ticket["parents"]
            machine_num = ticket["machine_num"]
            machine_type = ticket["machine_type"]
            location = machine_type_names[ticket["machine_type"]]
            start = ticket["start"]
            end = ticket["end"]
            duration = ticket["duration"]
            time_left = ticket["time_left"]
        except:
            print(f"Ticket {ticket_id} can't be added to schedule.")
            continue
        jobs.append(job_id)
        ticket_ids.append(ticket_id)
        parentses.append(parents)
        machine_nums.append(machine_num)
        machine_type_nums.append(machine_type)
        locations.append(location)
        starts.append(start)
        ends.append(end)
        durations.append(duration)
        time_lefts.append(time_left)

    schedule["job_id"] = jobs
    schedule["ticket_id"] = ticket_ids
    schedule["parents"] = parentses
    schedule["machine_num"] = machine_nums
    schedule["machine_type"] = machine_type_nums
    schedule["location"] = locations
    schedule["start"] = starts
    schedule["end"] = ends
    schedule["duration"] = durations
    schedule["time_left"] = time_lefts

    if len(jobs):
        return schedule
    else:
        return None

def extract_labor_schedule(R, S, C, job_list, all_robots):
    ''''''
    jobs, tasks = [], []
    robot_nums, machine_type_nums = [], []
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
                    machine_type_nums.append(task["machine_type"])
                    starts.append(int(S[job_idx, task_idx, robot].solution_value()))
                    ends.append(int(C[job_idx, task_idx, robot].solution_value()))
                    durations.append(task["duration"])

    schedule = pd.DataFrame()
    schedule["job_id"] = jobs
    schedule["ticket_id"] = ticket_ids
    schedule["robot_num"] = robot_nums
    schedule["machine_type"] = machine_type_nums
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
