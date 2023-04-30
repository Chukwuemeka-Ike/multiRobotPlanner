'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for displaying information about the scheduling process.
'''
from ortools.linear_solver import pywraplp

def display_job_data(job_list: list):
    '''Displays all tickets in a job list.'''
    num_jobs = len(job_list)
    print(f"Number of Jobs: {num_jobs}.")
    print()
    print("All tickets:")
    print("[")
    for job in job_list:
        for task in job:
            print("    ", [f"{k}: {v}" for k, v in task.items()])
    print("]")
    print()

def display_station_numbers(station_type_names: list, station_numbers: list):
    '''Displays the station numbers of each type.'''
    print("Station numbers:")
    print("[")
    for i in range(len(station_type_names)):
        print(f"{station_type_names[i]:>15}:    {station_numbers[i]}")
    print("]")
    print()

def display_solver_information(solver):
    ''''''
    print(f"Number of variables: {solver.NumVariables()}")
    print(f"Number of constraints: {solver.NumConstraints()}")


def display_solution_stats(solver, status, horizon, solutionTime):
    ''''''
    if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
        print(f"Max Schedule Length: {horizon: .2f}")
        print(f"Objective Value: {solver.Objective().Value(): .2f}")
        print(f"Solution Runtime: {solutionTime: .2f} seconds.")
    else:
        print("Infeasible program. Exiting.\n")
        exit()

def display_task_list(task_list: dict):
    '''Displays the task list.'''
    for ticket_id, ticket in task_list.items():
        print(f"Ticket {ticket_id}: \t {ticket}")