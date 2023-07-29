'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Common utilities for manipulating data structures.

    Functions here are for converting between:
        1. ticket lists,
        2. ticket dictionaries,
        3. schedules, and
        4. job lists.
    
'''
import pandas as pd

from arm_msgs.msg import Ticket, Tickets

from arm_utils.job_utils import get_all_children_from_task_list, get_all_parents_from_task_list

def convert_task_dict_to_ticket_list(ticket_dict: dict) -> list:
        '''Creates a list of Ticket messages from ticket_dict.'''
        ticket_list = []
        for ticket_id, ticket in ticket_dict.items():
            msg = Ticket()
            msg.ticket_id = ticket_id
            msg.job_id = ticket["job_id"]
            msg.machine_type = ticket["machine_type"]
            msg.duration = ticket["duration"]
            msg.parents = ticket["parents"]

            # These only exist once a schedule has been created.
            # Failing shouldn't stop the workflow.
            try:
                msg.time_left = ticket["time_left"]
                msg.status = ticket["status"]
                msg.start = ticket["start"]
                msg.end = ticket["end"]
                msg.machine_num = ticket["machine_num"]

                if len(ticket["parents"]) == 0:
                    msg.num_robots = ticket["num_robots"]
            except KeyError as e:
                pass
                # print(f"Warning: {e}")

            ticket_list.append(msg)
        return ticket_list

def convert_ticket_list_to_task_dict(tickets: Tickets) -> dict:
    '''Converts a list of tickets to a dictionary of tickets.

    Tickets is a custom message type that holds multiple Ticket types.
    '''
    task_dict = {}

    for ticket in tickets:
        tix = {}
        tix["job_id"] = ticket.job_id
        tix["ticket_id"] = ticket.ticket_id
        tix["machine_type"] = ticket.machine_type
        tix["duration"] = ticket.duration
        tix["parents"] = ticket.parents

        # These only exist once a schedule has been created.
        # Failing shouldn't stop the workflow.
        try:
            tix["status"] = ticket.status
            tix["start"] = ticket.start
            tix["end"] = ticket.end
            tix["machine_num"] = ticket.machine_num
            tix["time_left"] = ticket.time_left

            if len(ticket.parents) == 0:
                tix["num_robots"] = ticket.num_robots
        except KeyError as e:
            pass
            # print(f"Warning: {e}")

        task_dict[ticket.ticket_id] = tix
    return task_dict


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
        ticket["time_left"] = row["time_left"]

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


def convert_job_list_to_task_list(job_list: list):
    '''Converts a list of jobs to a dictionary of tickets.'''
    task_list = {}

    for job in job_list:
        for task in job:
            ticket = {}
            ticket["job_id"] = task["job_id"]
            ticket["ticket_id"] = task["ticket_id"]
            ticket["parents"] = task["parents"]
            ticket["machine_type"] = task["machine_type"]
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
                # TODO: Cross-check this doesn't leave any side effects.
                # task_list[task]["ticket_id"] = task
                # task_list[task]["job_id"] = job_id
                job_tasks.append(task_list[task])
                visited.append(task)
            job_tasks.reverse()
            job_list.append(job_tasks)
            job_id += 1
    return job_list
