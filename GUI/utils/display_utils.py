'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for displaying information about the scheduling process.
'''
from ortools.linear_solver import pywraplp
from ortools.sat.python import cp_model

def display_job_data(job_list: list):
    '''Displays all tickets in a job list.'''
    num_jobs = len(job_list)
    print(f"Number of Jobs: {num_jobs}.")
    print()
    print("All tickets:")
    print("[")
    for job in job_list:
        print("    [")
        for task in job:
            print("        ", task)
            # print("    ", [f"{k}: {v}" for k, v in task.items()])
        print("    ]")
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


def display_solution_stats_lp(solver, status, horizon, solutionTime):
    '''Display the LP solver statistics.'''
    if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
        if status == pywraplp.Solver.OPTIMAL:
            print("Optimal solution found.")
        else:
            print("Feasible solution found.")

        print(f"Max Schedule Length: {horizon: .2f}")
        print(f"Objective Value: {solver.Objective().Value(): .2f}")
        print(f"Solution Runtime: {solutionTime: .2f} seconds.")
    else:
        print("Infeasible program. Exiting.\n")
        exit()

def display_solution_stats_cpsat(solver, status):
    '''Display the CP-SAT solver statistics.'''
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        # Statistics.
        print('\nStatistics')
        print(f"  objective value: {solver.ObjectiveValue(): .2f}")
        print(f'  status   : {solver.StatusName(status)}')
        print(f'  conflicts: {solver.NumConflicts()}')
        print(f'  branches : {solver.NumBranches()}')
        print(f'  wall time: {solver.WallTime()} s')
        # print(f"  solution runtime: {solutionTime: .2f} seconds.")
    else:
        print("Infeasible program. Exiting.\n")
        exit()


def display_task_list(task_list: dict):
    '''Displays the task list.'''
    for ticket_id, ticket in task_list.items():
        print(f"Ticket {ticket_id}: \t {ticket}")