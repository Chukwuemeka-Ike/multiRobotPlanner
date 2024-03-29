import pandas as pd


def get_start_points(jobs_data: list):
    '''Gets the indices of tickets within each job that are at loading.'''
    start_ids = []
    for job in range(len(jobs_data)):
        job_start_ids = []
        for task in range(len(jobs_data[job])):
            if jobs_data[job][task]["station_type"] == 0:
                job_start_ids.append(task)
        start_ids.append(job_start_ids)
    return start_ids


def get_task_parent_indices(jobs_data: list):
    '''Get the indices of each task's parents within the job list.'''
    parent_indices = []

    for job in jobs_data:
        job_parent_indices = []
        for task_idx in range(len(job)):
            task_parent_indices = []
            for parent_ticket_id in job[task_idx]["parents"]:
                for task_idx in range(len(job)):
                    if job[task_idx]["ticket_id"] == parent_ticket_id:
                        task_parent_indices.append(task_idx)
            job_parent_indices.append(task_parent_indices)
        parent_indices.append(job_parent_indices)
    return parent_indices

# TODO: This can almost certainly be done better.
def get_immediate_child(id: int, tree_job: pd.DataFrame, linear_job: list):
    '''Recursively get the next child in tree job and add it to linear job.'''
    for next_tix in range(len(tree_job)):
        # TODO: Referring to tix that don't exist? Can maybe reduce the
        # effective time by only starting from row.
        if id in tree_job.iloc[next_tix, 3]:
            linear_job.append(next_tix)
            get_immediate_child(
                tree_job.iloc[next_tix, 2],
                tree_job, 
                linear_job
            )

def create_linear_jobs(tree_job: pd.DataFrame):
    '''Creates linear jobs from the tree job.'''
    start_rows = tree_job.loc[tree_job["Station Type #"] == 0]
    linear_jobs = []
    for row in range(len(start_rows)):
        id = start_rows.iloc[row, 2]
        linear_job = [row, ]
        get_immediate_child(id, tree_job, linear_job)
        linear_jobs.append(linear_job)

    return linear_jobs

def get_job_subsizes(schedule: pd.DataFrame, num_jobs: int):
    '''Get the number of starting points within each tree job.'''
    sizes = []
    for job_num in range(num_jobs):
        job = schedule.loc[schedule["Job #"] == job_num]
        sizes.append(len(job.loc[job["Station Type #"] == 0]))
    return sizes