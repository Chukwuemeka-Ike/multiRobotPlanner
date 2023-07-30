'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for extracting information from a generated schedule.
'''
import json
import pandas as pd


def load_schedule(filename: str):
    '''Loads a schedule from the given filename.'''
    schedule = pd.read_csv(filename, index_col=0)

    # Parents list gets saved as a string. Convert it back to a list.
    schedule["parents"] = schedule["parents"].apply(lambda x:json.loads(x))

    # Ensure start and end times are floats.
    schedule["start"] = schedule["start"].apply(lambda x:float(x))
    schedule["end"] = schedule["end"].apply(lambda x:float(x))

    return schedule

def extract_schedule_lp(X, S, C, job_list: list, machine_ids: list, machine_type_names: list):
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
            for machine in machine_ids:
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
    schedule["machine_id"] = machine_nums
    schedule["machine_type"] = machine_type_nums
    schedule["location"] = locations
    schedule["start"] = starts
    schedule["end"] = ends
    schedule["duration"] = durations
    schedule["time_left"] = time_lefts

    return schedule

def extract_schedule_cpsat(solver, X, S, C, job_list: list, machine_ids: list, machine_type_names: list):
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
            for machine in machine_ids:
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
    schedule["machine_id"] = machine_nums
    schedule["machine_type"] = machine_type_nums
    schedule["location"] = locations
    schedule["start"] = starts
    schedule["end"] = ends
    schedule["duration"] = durations
    schedule["time_left"] = time_lefts

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
