'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for handling data about an individual jobs.
'''
import pandas as pd
from typing import Tuple


def get_job_id_ticket_ids(job_list: list) -> dict:
    '''Creates a dictionary with job IDs as keys and lists of ticket IDs as values.'''
    job_task_ids = {}
    for job in job_list:
        ticket_ids = []
        for ticket in job:
            ticket_ids.append(ticket["ticket_id"])
        job_task_ids[job[0]["job_id"]] = ticket_ids
    return job_task_ids

def get_task_parent_indices(job_list: list) -> list:
    '''Get the indices of each task's parents within the job list.'''
    parent_indices = []

    for job in job_list:
        job_parent_indices = []
        for task_idx in range(len(job)):
            task_parent_indices = []
            for parent_ticket_id in job[task_idx]["parents"]:
                for task in range(len(job)):
                    if job[task]["ticket_id"] == parent_ticket_id:
                        task_parent_indices.append(task)
            job_parent_indices.append(task_parent_indices)
        parent_indices.append(job_parent_indices)
    return parent_indices

def get_all_children_from_task_list(id: int, ticket_dict: dict, linear_job: list) -> None:
    '''Recursively get the next child in a list of tasks and add it to a list.
    '''
    for ticket_id, ticket in ticket_dict.items():
        if id in ticket["parents"]:
            linear_job.append(ticket_id)
            get_all_children_from_task_list(
                ticket_id, 
                ticket_dict, 
                linear_job
            )

def get_all_parents_from_task_list(id: int, ticket_dict: dict, linear_job: list) -> None:
    '''Recursively get the parents of a node using DFS.'''
    for parent in ticket_dict[id]["parents"]:
        if parent in ticket_dict.keys():
            linear_job.append(parent)
            get_all_parents_from_task_list(
                parent,
                ticket_dict,
                linear_job
            )

def get_tree_job_start_ids(ticket_id: int, ticket_dict: dict) -> list:
    '''Gets the start nodes of a tree job.

    Returns:
        job_leaf_locations: ID's of the tickets that start the job.
    '''
    # Use any ticket in the job tree to traverse all the way to the end.
    # any_ticket = tree_job.iloc[0]
    linear_job = [ticket_id]
    get_all_children_from_task_list(
        ticket_id, ticket_dict, linear_job
    )

    job_leaf_locations = []
    get_leaf_locations(linear_job[-1], ticket_dict, job_leaf_locations)
    return job_leaf_locations

def get_leaf_locations(ticket_id: int, ticket_dict: dict, leaf_locations: list) -> None:
    '''Traverses the tree with DFS and stores the id's of leaves.'''
    parents = ticket_dict[ticket_id]["parents"]
    parents_in_job = get_parents_in_job(parents, ticket_dict)
    if len(parents_in_job) == 0:
        leaf_locations.append(ticket_id)
    for parent in parents_in_job:
        get_leaf_locations(
            parent, ticket_dict, leaf_locations
        )

def get_parents_in_job(parents: list, ticket_dict: dict) -> list:
    '''Searches the job for the parents.

    Checks that the parents are not done and are still in the ticket_dict.
    '''
    parents_in_job = []
    for parent in parents:
        if parent in ticket_dict:
            parents_in_job.append(parent)
    return parents_in_job

def get_task_idx_in_job(ticket_id: int, job_list: list) -> Tuple[int, int]:
    '''Gets the ticket's job_idx and task_idx in the job list.'''
    for job_idx, job in enumerate(job_list):
        for task_idx, task in enumerate(job):
            if ticket_id == task["ticket_id"]:
                return job_idx, task_idx
    raise LookupError(f"The ticket_id {ticket_id} is not in the job list.")

def create_linear_jobs(tree_job: pd.DataFrame, ticket_dict: dict):
    '''Creates linear jobs from the tree job.
    Returns:
        linear_jobs - List of lists of ticket id's that form a linear path
                        through the tree from start to final node.
    '''
    # Last id is the root of the job tree. Use that to get the leaf nodes.
    start_ids = get_tree_job_start_ids(tree_job.iloc[0].ticket_id, ticket_dict)

    linear_jobs = []
    for ticket in start_ids:
        linear_job = [ticket]
        get_all_children_from_task_list(ticket, ticket_dict, linear_job)
        linear_jobs.append(linear_job)
    return linear_jobs

def get_job_subsizes(schedule: pd.DataFrame, ticket_dict: dict):
    '''Get the number of starting points within each tree job.'''
    job_ids = schedule["job_id"].unique()
    sizes = []
    for job_id in job_ids:
        job = schedule.loc[schedule["job_id"] == job_id]
        start_ids = get_tree_job_start_ids(job.iloc[0].ticket_id, ticket_dict)
        sizes.append(len(start_ids))
    return sizes

def get_all_job_start_points(job_list: list, ticket_dict: dict) -> dict:
    '''Gets the starting points of each job in the job list.

    Args:
        job_list: list of lists where each job is a list of dictionaries.
    Returns:
        start_points: dictionary of {job_id: [starting points]}.
    '''
    start_points = {}
    for job in job_list:
        start_points[job[0]["job_id"]] = get_tree_job_start_ids(
            job[0]["ticket_id"],
            ticket_dict
        )
    return start_points

def has_job_started(job: list, ticket_dict: dict) -> bool:
    '''Checks if any starting ticket has started within a job.

    Used by robot assigner to know whether a job can still be given
    robots.

    Args:
        job: list of tickets where each ticket is a dictionary.
        ticket_dict: dictionary of all tickets.
    Returns:
        is_started: True or False.
    '''
    # Take the first ticket in the list and use it to traverse all the way down.
    any_task = job[0]
    started_num = 0

    start_points = get_tree_job_start_ids(any_task["ticket_id"], ticket_dict)
    for start_point in start_points:
        # Top-level tickets should be ready, not waiting, since they're not
        # dependent on any other tickets.
        if ticket_dict[start_point]["status"] != "Ready":
            started_num += 1

    if started_num == 0:
        return False
    else:
        return True

def get_job_last_ticket_status(job: list, ticket_dict: dict) -> str:
    '''Gets the status of the last ticket in a job.

    Args:
        job: list of tickets where each ticket is a dictionary.
        ticket_dict: dictionary of all tickets.
    Returns:
        job_status: string - Unfinished or Finished.
    '''
    # TODO: This will need more status options if we want to make more
    # decisions based on the status.

    # Take the first ticket in the list and use it to traverse all the way down.
    any_task = job[0]
    linear_job = [any_task["ticket_id"]]
    get_all_children_from_task_list(
        any_task["ticket_id"], ticket_dict, linear_job
    )

    # linear_job's last element is the end of the job. Check the status.
    # If it's not in the task dictionary, it must have been deleted.
    if linear_job[-1] not in ticket_dict:
        return "Deleted"
    elif ticket_dict[linear_job[-1]]["status"] == "Done":
        return "Done"
    else:
        return "Unfinished"
