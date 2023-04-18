import pandas as pd
from utils.job_utils import *
from constants import station_type_ws_nums, Mjs

def extract_schedule(X, S, C, jobs_data: list, all_machines: list, station_type_names: list):
    '''Extracts and formats the schedule from the solved MILP and job data.
    Returns:
        Schedule - DataFrame of Job-Task schedule with start and end times.
    '''
    jobs, tasks = [], []
    locations, station_nums, station_type_nums = [], [], []
    ticket_ids, parentses = [], []
    starts, ends, durations = [], [], []

    for job_id, job in enumerate(jobs_data):
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

    schedule = pd.DataFrame()
    schedule["Job #"] = jobs
    schedule["Task #"] = tasks
    schedule["Ticket ID"] = ticket_ids
    schedule["Parents"] = parentses
    schedule["Station #"] = station_nums
    schedule["Station Type #"] = station_type_nums
    schedule["Location"] = locations
    schedule["Start"] = starts
    schedule["End"] = ends
    schedule["Duration"] = durations

    return schedule

def extract_labor_schedule(R, S, C, jobs_data, all_robots):
    ''''''
    jobs, tasks = [], []
    robot_nums, station_type_nums = [], []
    ticket_ids, parentses = [], []
    starts, ends, durations = [], [], []

    for job_id, job in enumerate(jobs_data):
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
    schedule["Job #"] = jobs
    schedule["Ticket ID"] = ticket_ids
    schedule["Robot #"] = robot_nums
    schedule["Station Type #"] = station_type_nums
    schedule["Start"] = starts
    schedule["End"] = ends
    schedule["Duration"] = durations

    return schedule


def get_total_idle_time(schedule, jobs_data, parent_ids):
    '''Gets the total idle time between connected tasks in the schedule.'''
    sum = 0
    for i in range(len(jobs_data)):
        job = schedule.loc[schedule["Job #"] == i]

        for j in range(len(job)):
            task = job.iloc[j]
            for parent in parent_ids[i][j]:
                sum += (task["Start"] - job.iloc[parent, 8])
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
    for j in range(job.iloc[subjob[0]]["Start"]):
        timeline.append("WS_0_0")

    # Start 
    for i in range(subjob_length):
        row = job.iloc[subjob[i]]
        j = row["Start"]

        for _ in range(row["Duration"]):
            location = station_type_ws_nums[row["Location"]] + str(Mjs[row["Station #"]])

            timeline.append(
                location
                # row["Location"]
            )
            j+=1

        if i < subjob_length - 1:
            while j < (job.iloc[subjob[i+1]]["Start"]):
                timeline.append(location)
                j+=1

    for j in range(job["End"].max(), timeline_length):
        timeline.append(station_type_ws_nums["Loading Area"] + str(Mjs[0]))
    print(len(timeline))
    # print(timeline_length)
    return timeline

def create_robot_timelines(schedule):
    ''''''
    robot_timelines = pd.DataFrame()
    
    num_jobs = schedule["Job #"].max()+1
    sizes = get_job_subsizes(schedule, num_jobs)
    timeline_length = schedule["End"].max()+1

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
        job = schedule.loc[schedule["Job #"] == job_num]
        linear_jobs = create_linear_jobs(job)

        for linear_job_id, linear_job in enumerate(linear_jobs):
            for m in range(numRobots[job_num][linear_job_id]):
                # robot_timelines[f"Job {job_num} Sub-job {linear_job_id} Robot {m}"] = generate_subjob_timeline(job, linear_job, timeline_length)
                robot_timelines[f"robot_{robNum}"] = generate_subjob_timeline(job, linear_job, timeline_length)
                robNum += 1

    return robot_timelines