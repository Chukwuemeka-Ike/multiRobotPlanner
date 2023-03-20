import pandas as pd

def extract_schedule(X, S, C, jobs_data, all_machines, station_names):
    '''Extract the schedule.'''
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
                    locations.append(station_names[task["station_type"]])
                    starts.append(S[job_id, task_id, machine].solution_value())
                    ends.append(C[job_id, task_id, machine].solution_value())
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
