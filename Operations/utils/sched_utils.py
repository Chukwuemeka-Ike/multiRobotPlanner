import pandas as pd
from utils.job_utils import *

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
    subjob_length = len(subjob)
    timeline = []
    
    # Padding at the beginning.
    j = 0
    for j in range(job.iloc[subjob[0]]["Start"]):
        timeline.append("")

    # Start 
    for i in range(subjob_length):
        row = job.iloc[subjob[i]]
        j = row["Start"]

        for _ in range(row["Duration"]):
            timeline.append(row["Location"])
            j+=1

        if i < subjob_length - 1:
            while j < (job.iloc[subjob[i+1]]["Start"]):
                timeline.append("")
                j+=1

    for j in range(int(job["End"].max()), timeline_length):
        timeline.append("")

    return timeline

def create_robot_timelines(schedule):
    ''''''
    robot_timelines = pd.DataFrame()
    
    num_jobs = schedule["Job #"].max()+1
    sizes = get_job_subsizes(schedule, num_jobs)
    timeline_length = schedule["End"].max()+1

    numRobots = [
        [2, 3],
        [4, 2],
        [2,2,2],
        [3,2],
        [2],
        [2],
        [2,2],
    ]

    for job_num in range(num_jobs):
        job = schedule.loc[schedule["Job #"] == job_num]
        linear_jobs = create_linear_jobs(job)

        for linear_job_id, linear_job in enumerate(linear_jobs):
            for m in range(numRobots[job_num][linear_job_id]):
                robot_timelines[f"Job {job_num} Sub-job {linear_job_id} Robot {m}"] = generate_subjob_timeline(job, linear_job, timeline_length)

    return robot_timelines