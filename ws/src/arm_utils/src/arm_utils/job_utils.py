'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for handling data about an individual jobs.
'''
import pandas as pd
from arm_msgs.msg import Ticket, Tickets


def get_job_id_ticket_ids(job_list: list) -> dict:
    '''Creates a dictionary with job IDs as keys and lists of ticket IDs as values.'''
    job_task_ids = {}
    for job in job_list:
        ticket_ids = []
        for ticket in job:
            ticket_ids.append(ticket["ticket_id"])
        job_task_ids[job[0]["job_id"]] = ticket_ids
    return job_task_ids



def convert_ticket_list_to_task_dict(tickets: Tickets) -> dict:
    '''Converts a list of tickets to a dictionary of tickets.

    Tickets is a custom message type that holds multiple Ticket types.
    '''
    task_dict = {}

    for ticket in tickets:
        tix = {}
        tix["job_id"] = ticket.job_id
        tix["station_type"] = ticket.machine_type
        tix["duration"] = ticket.duration
        tix["parents"] = ticket.parents

        # These only exist once a schedule has been created.
        # Failing shouldn't stop the workflow.
        # TODO: Time left 
        try:
            tix["time_left"] = ticket.time_left
            tix["start"] = ticket.start
            tix["end"] = ticket.end
            tix["station_num"] = ticket.station_num
        except KeyError as e:
            pass
            # print(f"Warning: {e}")
        task_dict[ticket.ticket_id] = tix
    return task_dict

def create_ticket_list(ticket_dict: dict):
        '''Creates a list of Ticket messages from ticket_dict.'''
        ticket_list = []
        for ticket_id, ticket in ticket_dict.items():
            msg = Ticket()
            msg.ticket_id = ticket_id
            msg.job_id = ticket["job_id"]
            msg.machine_type = ticket["station_type"]
            msg.duration = ticket["duration"]
            msg.parents = ticket["parents"]

            # These only exist once a schedule has been created.
            # Failing shouldn't stop the workflow.
            try:
                msg.start = ticket["start"]
                msg.end = ticket["end"]
                msg.station_num = ticket["station_num"]
                msg.time_left = ticket["time_left"]
            except KeyError as e:
                pass
                # print(f"Warning: {e}")

            ticket_list.append(msg)
        return ticket_list

def convert_job_list_to_task_list(job_list: list):
    '''Converts a list of jobs to a dictionary of tickets.'''
    task_list = {}

    for job in job_list:
        for task in job:
            ticket = {}
            ticket["job_id"] = task["job_id"]
            ticket["ticket_id"] = task["ticket_id"]
            ticket["parents"] = task["parents"]
            ticket["station_type"] = task["station_type"]
            ticket["duration"] = task["duration"]
            ticket["time_left"] = task["duration"]
            task_list[task["ticket_id"]] = ticket
    return task_list

def convert_task_list_to_job_list(task_list: dict):
    '''Converts a dictionary of tickets to a job list.'''
    job_list, visited = [], []
    job_id = 0

    for ticket_id, _ in task_list.items():
        if ticket_id not in visited:
            linear_job = [ticket_id, ]
            get_all_children_from_task_list(
                ticket_id, task_list, linear_job
            )

            # The last id in the linear job is the root of the job tree.
            # Start there to traverse the whole tree.
            last_job_task = linear_job[-1]
            all_job_tasks = [last_job_task, ]
            get_all_parents_from_task_list(
                last_job_task, task_list, all_job_tasks
            )

            # Add the list of all the job's tasks to the job_list.
            # Add all the indices to visited to avoid duplicates.
            # job_tasks = [task_list[id] for id in all_job_tasks]
            job_tasks = []
            for task in all_job_tasks:
                task_list[task]["ticket_id"] = task
                task_list[task]["job_id"] = job_id
                job_tasks.append(task_list[task])
                visited.append(task)
            job_tasks.reverse()
            job_list.append(job_tasks)
            job_id += 1
    return job_list

def get_task_parent_indices(job_list: list):
    '''Get the indices of each task's parents within the job list.'''
    parent_indices = []

    for job in job_list:
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

def get_all_children_from_task_list(id: int, task_list: dict, linear_job: list):
    '''Recursively get the next child in a list of tasks and add it to a list.
    '''
    for ticket_id, ticket in task_list.items():
        if id in ticket["parents"]:
            linear_job.append(ticket_id)
            get_all_children_from_task_list(
                ticket_id, 
                task_list, 
                linear_job
            )

def get_all_parents_from_task_list(id: int, task_list: dict, linear_job: list):
    '''Recursively get the parents of a node using DFS.'''
    for parent in task_list[id]["parents"]:
        if parent in task_list.keys():
            linear_job.append(parent)
            get_all_parents_from_task_list(
                parent,
                task_list,
                linear_job
            )

def get_tree_job_start_ids(ticket_id: int, task_list: dict):
    '''Gets the start nodes of a tree job.

    Returns ID's of the tickets in the job, not the task ID's.
    Need to use those ticket ID's with a different function.
    '''
    # Use any ticket in the job tree to traverse all the way to the end.
    # any_ticket = tree_job.iloc[0]
    linear_job = [ticket_id]
    get_all_children_from_task_list(
        ticket_id, task_list, linear_job
    )

    job_leaf_locations = []
    get_leaf_locations(linear_job[-1], job_leaf_locations, task_list)
    return job_leaf_locations

def get_leaf_locations(ticket_id, leaf_locations, task_list: dict):
    '''Traverses the tree with DFS and stores the id's of leaves.'''
    parents = task_list[ticket_id]["parents"]
    parents_in_job = get_parents_in_job(parents, task_list)
    if len(parents_in_job) == 0:
        leaf_locations.append(ticket_id)
    for parent in parents_in_job:
        get_leaf_locations(
            parent, leaf_locations, task_list
        )

def get_parents_in_job(parents, task_list):
    '''Searches the job for the parents.

    Checks that the parents are not done and are still in the task_list.
    '''
    parents_in_job = []
    for parent in parents:
        if parent in task_list:
            parents_in_job.append(parent)
    return parents_in_job

def get_task_idx_in_job(ticket_id: int, job_list: list):
    '''Gets the ticket's job_idx and task_idx in the job list.'''
    for job_idx, job in enumerate(job_list):
        for task_idx, task in enumerate(job):
            if ticket_id == task["ticket_id"]:
                return job_idx, task_idx
    raise LookupError(f"The ticket_id {ticket_id} is not in the job list.")

def create_linear_jobs(tree_job: pd.DataFrame, task_list: dict):
    '''Creates linear jobs from the tree job.
    Returns:
        linear_jobs - List of lists of ticket id's that form a linear path
                        through the tree from start to final node.
    '''
    # Last id is the root of the job tree. Use that to get the leaf nodes.
    start_ids = get_tree_job_start_ids(tree_job.iloc[0].ticket_id, task_list)

    linear_jobs = []
    for ticket in start_ids:
        linear_job = [ticket]
        get_all_children_from_task_list(ticket, task_list, linear_job)
        linear_jobs.append(linear_job)
    return linear_jobs

def get_job_subsizes(schedule: pd.DataFrame, task_list: dict):
    '''Get the number of starting points within each tree job.'''
    job_ids = schedule["job_id"].unique()
    sizes = []
    for job_id in job_ids:
        job = schedule.loc[schedule["job_id"] == job_id]
        start_ids = get_tree_job_start_ids(job.iloc[0].ticket_id, task_list)
        sizes.append(len(start_ids))
    return sizes