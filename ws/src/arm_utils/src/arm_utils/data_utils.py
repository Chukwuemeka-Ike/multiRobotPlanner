'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Common utilities for manipulating data structures.

    Functions here are for converting between:
        1. ticket lists
        2. task dicts
        3. schedules, and
        4. job lists
    
'''
import pandas as pd

from arm_msgs.msg import Ticket, Tickets


def create_ticket_list(ticket_dict: dict) -> list:
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
                msg.start = ticket["start"]
                msg.end = ticket["end"]
                msg.machine_num = ticket["machine_num"]
                msg.time_left = ticket["time_left"]
            except KeyError as e:
                pass
                print(f"Warning: {e}")

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
        tix["machine_type"] = ticket.machine_type
        tix["duration"] = ticket.duration
        tix["parents"] = ticket.parents

        # These only exist once a schedule has been created.
        # Failing shouldn't stop the workflow.
        # TODO: Time left 
        try:
            tix["time_left"] = ticket.time_left
            tix["start"] = ticket.start
            tix["end"] = ticket.end
            tix["machine_num"] = ticket.machine_num
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
        ticket["time_left"] = row["duration"]

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
