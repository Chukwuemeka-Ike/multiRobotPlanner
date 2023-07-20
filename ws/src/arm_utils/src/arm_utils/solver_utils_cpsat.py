'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for creating variables and constraints for the Google OR-TOOLS
    CP-SAT solver.
'''
import math

from ortools.sat.python import cp_model
from itertools import combinations

from arm_utils.job_utils import get_task_idx_in_job


def intersection(lst1, lst2):
    '''Returns the common items between two lists.'''
    return list(set(lst1) & set(lst2))

def create_opt_variables(model: cp_model.CpModel, job_list: list, all_machines: list, Mj: list):
    '''Creates all the optimization variables for the MILP problem.
    Args:
        model: - .
        job_list: List of lists, where each sub-list is a job made up of
                    task dictionaries.
        all_machines: List of unique identifiers for each machine.
        Mj: List of lists, where each sub-list contains the unique identifiers
            of the machines of a specific type. 

    Returns:
        X: Binary variable - 1 if job i task j is assigned to machine k.
        Y: Binary variable - 1 if (job_a, task_a) precedes
            (job_b, task_b) on machine k. 0, otherwise.
        Z: Binary variable - 1 if (job_idx, task_a) precedes
            (job_idx, task_b) on machine k. 0, otherwise.
        S: Start time of (job, task) on machine if assigned. 0, otherwise.
        C: Completion time of (job, task) on machine if assigned. 0, otherwise.
        S_job: Start time of each job.
        C_job: Completion time of each job.
        C_max: Overall makespan (length of the schedule).
    '''
    X = {}
    Y = {}
    Z = {}
    S = {}
    C = {}
    S_job = {}
    C_job = {}

    horizon = sum(math.ceil(task["duration"]) for job in job_list for task in job)
    num_jobs = len(job_list)

    for job in range(num_jobs):
        for task in range(len(job_list[job])):
            for machine in all_machines:
                # Binary variable that is 1 if (job, task) is assigned to machine.
                X[job, task, machine] = model.NewBoolVar(f'X{job}{task}{machine}')

                # Start time of (job, task) on machine if assigned. 0, otherwise.
                S[job, task, machine] = model.NewIntVar(0, horizon, f'S{job}{task}{machine}')

                # Completion time of (job, task) on machine if assigned. 0, otherwise.
                C[job, task, machine] = model.NewIntVar(0, horizon, f'C{job}{task}{machine}')

    for i in range(num_jobs):
        # Start and completion time of each job.
        S_job[i] = model.NewIntVar(0, horizon, f'S{i}')
        C_job[i] = model.NewIntVar(0, horizon, f'C{i}')

    for job_b in range(1, num_jobs):
        for job_a in range(job_b):
            for task_b in range(len(job_list[job_b])):
                for task_a in range(len(job_list[job_a])):
                    M_intersection = intersection(
                        Mj[job_list[job_b][task_b]["machine_type"]],
                        Mj[job_list[job_a][task_a]["machine_type"]]
                    )
                    for machine in M_intersection:
                        # Binary variable - 1 if (job_a, task_a) precedes
                        # (job_b, task_b) on machine. 0, otherwise.
                        Y[job_a, task_a, job_b, task_b, machine] = model.NewBoolVar(
                            f'Y{job_a}{task_a}{job_b}{task_b}{machine}'
                        )

    for job_idx, job in enumerate(job_list):
        combos = combinations(range(len(job)), 2)
        for combo in combos:
            M_intersection = intersection(
                Mj[job[combo[0]]["machine_type"]],
                Mj[job[combo[1]]["machine_type"]]
            )
            for machine in M_intersection:
                # Binary variable - 1 if (job_idx, combo[0]) precedes
                # (job_idx, combo[1]) on machine. 0, otherwise.
                Z[job_idx, combo[0], combo[1], machine] = model.NewBoolVar(
                    f'Z{job_idx}{combo[0]}{combo[1]}{machine}'
                )

    # Overall makespan.
    C_max = model.NewIntVar(0, horizon, 'makespan')

    return X, Y, Z, S, C, S_job, C_job, C_max

def define_constraints(model: cp_model.CpModel, X, Y, Z, S, C, S_job, C_job, C_max, job_list, parent_ids, Mj):
    '''Defines all the optimization constraints.
    
    Args:
        model: - Instance of cp_model.
        X: Binary variable - 1 if job i task j is assigned to machine k.
        Y: Binary variable - 1 if (job_a, task_a) precedes
            (job_b, task_b) on machine k. 0, otherwise.
        Z: Binary variable - 1 if (job_idx, task_a) precedes
            (job_idx, task_b) on machine k. 0, otherwise.
        S: Start time of (job, task) on machine if assigned. 0, otherwise.
        C: Completion time of (job, task) on machine if assigned. 0, otherwise.
        S_job: Start time of each job.
        C_job: Completion time of each job.
        C_max: Overall makespan (length of the schedule).
        job_list: List of lists, where each sub-list is a job made up of
                    task dictionaries.
        parent_ids: List of lists, where each sub-list contains the ticket IDs
            of the corresponding ticket's parents.
        Mj: List of lists, where each sub-list contains the unique identifiers
            of the machines of a specific type.
    '''
    # Job-specific constraints.
    for job_idx, job in enumerate(job_list):
        for task_idx, task in enumerate(job):
            # Each operation can only be assigned to one machine.
            model.AddExactlyOne(
                X[job_idx, task_idx, machine] for machine in Mj[task["machine_type"]]
            )

            # Within a job, each task must start after the all parent tasks end.
            for parent in parent_ids[job_idx][task_idx]:
                model.Add(
                    sum(S[job_idx, task_idx, machine] for machine in Mj[task["machine_type"]]) >=
                    sum(C[job_idx, parent, machine] for machine in Mj[job[parent]["machine_type"]])
                )

    for job_idx, job in enumerate(job_list):
        combos = combinations(range(len(job)), 2)
        for combo in combos:
            M_intersection = intersection(
                Mj[job[combo[0]]["machine_type"]],
                Mj[job[combo[1]]["machine_type"]]
            )
            # No two tasks within the same job can overlap on the same machine.
            for machine in M_intersection:
                model.Add(
                    S[job_idx, combo[0], machine] >=
                        C[job_idx, combo[1], machine]
                ).OnlyEnforceIf(Z[job_idx, combo[0], combo[1], machine].Not())
                model.Add(
                    S[job_idx, combo[1], machine] >=
                        C[job_idx, combo[0], machine]
                ).OnlyEnforceIf(Z[job_idx, combo[0], combo[1], machine])

        # Task completion constraints.
        for job_idx, job in enumerate(job_list):
            for task_idx, task in enumerate(job):
                # Set constraints for all machines that match the task requirement.
                for machine in Mj[task["machine_type"]]:
                    # The start and end time must equal zero if the task is not
                    # assigned to that machine.
                    model.Add(
                        S[job_idx, task_idx, machine] + C[job_idx, task_idx, machine] == 0
                    ).OnlyEnforceIf(X[job_idx, task_idx, machine].Not())

                    # Ensure that the completion time is exactly start + duration.
                    try:
                        model.Add(
                            S[job_idx, task_idx, machine] + math.ceil(task["time_left"])
                                == C[job_idx, task_idx, machine]
                        ).OnlyEnforceIf(X[job_idx, task_idx, machine])
                    except TypeError as e:
                        print(e)
                        print(task["time_left"])

        # Precedence constraints. Iterate between every job-task-job-task pair to
        # make sure that no two tasks are assigned to the same machine at the
        # same time.
        for job_b in range(1, len(job_list)):
            for job_a in range(job_b):
                for task_b in range(len(job_list[job_b])):
                    for task_a in range(len(job_list[job_a])):
                        # Check if the task-task pair have overlapping machines.
                        M_intersection = intersection(
                            Mj[job_list[job_b][task_b]["machine_type"]],
                            Mj[job_list[job_a][task_a]["machine_type"]]
                        )   
                        # print(M_intersection)
                        for machine in M_intersection:
                            model.Add(
                                S[job_a, task_a, machine] >=
                                    C[job_b, task_b, machine]
                            ).OnlyEnforceIf(Y[job_a, task_a, job_b, task_b, machine].Not())
                            model.Add(
                                S[job_b, task_b, machine] >=
                                    C[job_a, task_a, machine]
                            ).OnlyEnforceIf(Y[job_a, task_a, job_b, task_b, machine])

        # Overall job start and completion constraints.
        for job_idx, job in enumerate(job_list):
            # TODO: Streamline this if possible.
            for task_idx, task in enumerate(job):
                # Job's start must be before the first task's start time.
                model.Add(
                    S_job[job_idx] <= 
                        sum(S[job_idx, task_idx, machine] for machine in Mj[job[task_idx]["machine_type"]])
                )
                # Job's completion must be after the last task's completion time.
                model.Add(
                    C_job[job_idx] >= 
                        sum(C[job_idx, task_idx, machine] for machine in Mj[job[task_idx]["machine_type"]])
                )

            # The overall makespan must be after the last job's completion.
            model.Add(
                C_max >= C_job[job_idx]
            )

def respect_ongoing_constraints(model: cp_model.CpModel, X, S, job_list: list, ongoing: dict):
    '''Constraints to ensure ongoing tasks are assigned to the
    same machines and start at time zero.

    Args:
        model: - Instance of cp_model.
        job_list: List of lists, where each sub-list is a job made up of
                    task dictionaries.
        X: Binary variable - 1 if job i task j is assigned to machine k.
        S: Start time of (job, task) on machine if assigned. 0, otherwise.
        ongoing: Dictionary of ongoing tickets. Ticket ID is the key.
    '''
    for ticket_id, ticket in ongoing.items():
        # Get the ticket location in the job list and its assigned machine.
        job_idx, task_idx = get_task_idx_in_job(ticket_id, job_list)
        machine_num = ticket["machine_num"]

        # Set the 
        model.Add(
            X[job_idx, task_idx, machine_num] == 1
        )
        model.Add(
            S[job_idx, task_idx, machine_num] == 0
        )

def create_idle_time_objective(model: cp_model.CpModel, S, C, C_max, job_list, parent_ids, Mj, optimum):
    '''Sets the schedule's total idle time as the objective.'''
    # TODO: Ensure this actually works.
    # Add constraint for makespan based on previously found optimum.
    model.Add(C_max <= optimum)

    # Define the objective function to minimize the idle time.
    idle_times = []
    for job_idx, job in enumerate(job_list):
        for task_idx, task in enumerate(job):
            # Idle time between each task's start and its parents' completion.
            for parent in parent_ids[job_idx][task_idx]:
                idle_times.append(
                    sum(S[job_idx, task_idx, machine] for machine in Mj[job[task_idx]["machine_type"]])
                    - sum(C[job_idx, parent, machine] for machine in Mj[job[parent]["machine_type"]])
                )
    model.Minimize(sum(idle_times))
