import pandas as pd


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