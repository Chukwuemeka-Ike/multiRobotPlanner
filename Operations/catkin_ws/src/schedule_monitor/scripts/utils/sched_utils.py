'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for extracting information from a generated schedule.
'''
import json
import pandas as pd
from utils.job_utils import *
from constants import station_type_ws_nums, Mjs


def load_schedule(filename):
    '''.'''
    schedule = pd.read_csv(filename, index_col=0)

    # Parents list gets saved as a string. Convert it back to a list.
    schedule["parents"] = schedule["parents"].apply(lambda x:json.loads(x))

    # Ensure start and end times are floats.
    schedule["start"] = schedule["start"].apply(lambda x:float(x))
    schedule["end"] = schedule["end"].apply(lambda x:float(x))

    return schedule

def extract_schedule(X, S, C, job_list: list, all_machines: list, station_type_names: list):
    '''Extracts and formats the schedule from the solved MILP and job data.
    Returns:
        Schedule - DataFrame of Job-Task schedule with start and end times.
    '''
    jobs, tasks = [], []
    locations, station_nums, station_type_nums = [], [], []
    ticket_ids, parentses = [], []
    starts, ends, durations, time_lefts = [], [], [], []

    for job_id, job in enumerate(job_list):
        for task_id, task in enumerate(job):
            for machine in all_machines:
                if X[job_id, task_id, machine].solution_value() > 0.5:
                    jobs.append(job_id)
                    tasks.append(task_id)
                    ticket_ids.append(task["ticket_id"])
                    parentses.append(task["parents"])
                    station_nums.append(machine)
                    station_type_nums.append(task["station_type"])
                    locations.append(station_type_names[task["station_type"]])
                    starts.append(int(S[job_id, task_id, machine].solution_value()))
                    ends.append(int(C[job_id, task_id, machine].solution_value()))
                    durations.append(task["duration"])
                    time_lefts.append(task["time_left"])

    schedule = pd.DataFrame()
    schedule["job_id"] = jobs
    schedule["task_id"] = tasks
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
        ticket["task_id"] = row["task_id"]
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

    for job_id, job in enumerate(job_list):
        for task_id, task in enumerate(job):
            for robot in all_robots:
                if R[job_id, task_id, robot].solution_value() > 0.5:
                    jobs.append(job_id)
                    tasks.append(task_id)
                    ticket_ids.append(task["ticket_id"])
                    robot_nums.append(robot)
                    station_type_nums.append(task["station_type"])
                    starts.append(int(S[job_id, task_id, robot].solution_value()))
                    ends.append(int(C[job_id, task_id, robot].solution_value()))
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


def generate_subjob_timeline(job: pd.DataFrame, subjob: list, timeline_length: int):
    '''Generates a minute-to-minute timeline given a linear subjob from a job.
    Arguments:
        job - DataFrame of a possible tree job containing multiple subjobs.
        subjob - List of indices in Job that make up a subjob sequence.
        timeline_length - Length of the desired timeline. Should match
                            the overall schedule.
    Returns:
        timeline - List of locations based on the subjob's schedule.
    '''
    # TODO: Appending makes an implicit assumption that the schedule will
    # always be solid. This fails if an end is after the subsequent task's
    # start. Example:
    # Start - 5, End - 10
    # Start - 9, End - 29
    # This case will give more entries than needed

    subjob_length = len(subjob)
    # print(job.iloc[subjob])
    timeline = []
    
    # Padding at the beginning.
    j = 0
    for j in range(job.iloc[subjob[0]]["start"]):
        timeline.append("WS_0_0")

    # Start 
    for i in range(subjob_length):
        row = job.iloc[subjob[i]]
        j = row["start"]

        for _ in range(row["duration"]):
            location = station_type_ws_nums[row["location"]] + str(Mjs[row["station_num"]])

            timeline.append(
                location
                # row["location"]
            )
            j+=1

        if i < subjob_length - 1:
            while j < (job.iloc[subjob[i+1]]["start"]):
                timeline.append(location)
                j+=1

    for j in range(job["end"].max(), timeline_length):
        timeline.append(station_type_ws_nums["Loading Area"] + str(Mjs[0]))
    print(len(timeline))
    # print(timeline_length)
    return timeline

def create_robot_timelines(schedule):
    ''''''
    robot_timelines = pd.DataFrame()
    
    num_jobs = schedule["job_id"].max()+1
    sizes = get_job_subsizes(schedule, num_jobs)
    timeline_length = schedule["end"].max()+1

    numRobots = [
        [2, 2],
        [3],
        [2,1,2],
        [1,2],
        [2],
        [2],
        [2,2],
    ]
    robNum = 0

    for job_num in range(num_jobs):
        job = schedule.loc[schedule["job_id"] == job_num]
        linear_jobs = create_linear_jobs(job)

        for linear_job_id, linear_job in enumerate(linear_jobs):
            for m in range(numRobots[job_num][linear_job_id]):
                # robot_timelines[f"Job {job_num} Sub-job {linear_job_id} Robot {m}"] = generate_subjob_timeline(job, linear_job, timeline_length)
                robot_timelines[f"robot_{robNum}"] = generate_subjob_timeline(job, linear_job, timeline_length)
                robNum += 1

    return robot_timelines