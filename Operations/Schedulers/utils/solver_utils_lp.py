'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Utilities for creating variables and constraints for the Google OR-TOOLS
    linear solver.
'''
from itertools import combinations
from utils.job_utils import get_task_idx_in_job


def intersection(lst1, lst2):
    '''Returns the common items between two lists.'''
    return list(set(lst1) & set(lst2))

def create_opt_variables(solver, job_list: list, all_machines: list, Mj: list):
    '''Creates all the optimization variables for the MILP problem.

    Returns:
        X: Binary variable - 1 if job i task j is assigned to machine k.
        Y: Binary variable - 1 if (job_a, task_a) precedes
            (job_b, task_b) on machine k. 0, otherwise.
        Z: Binary variable - 1 if (job_idx, task_a) precedes
            (job_idx, task_b) on machine k. 0, otherwise.
        S: Start time of (job, task) on machine if assigned. 0, otherwise.
        C: Completion time of (job, task) on machine if assigned. 0, otherwise.
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

    num_jobs = len(job_list)
    horizon = sum(task["duration"] for job in job_list for task in job)

    for job in range(num_jobs):
        for task in range(len(job_list[job])):
            for machine in all_machines:
                # Binary variable that is 1 if (job, task) is assigned to machine.
                X[job, task, machine] = solver.IntVar(0, 1, f'X{job}{task}{machine}')

                # Start time of (job, task) on machine if assigned. 0, otherwise.
                S[job, task, machine] = solver.Var(0, horizon, False, f'S{job}{task}{machine}')

                # Completion time of (job, task) on machine if assigned. 0, otherwise.
                C[job, task, machine] = solver.Var(0, horizon, False, f'C{job}{task}{machine}')

    for i in range(num_jobs):
        # Start and completion time of each job.
        S_job[i] = solver.Var(0, horizon, False, f'S{i}')
        C_job[i] = solver.Var(0, horizon, False, f'C{i}')

    for job_b in range(1, num_jobs):
        for job_a in range(job_b):
            for task_b in range(len(job_list[job_b])):
                for task_a in range(len(job_list[job_a])):
                    M_intersection = intersection(
                        Mj[job_list[job_b][task_b]["station_type"]],
                        Mj[job_list[job_a][task_a]["station_type"]]
                    )
                    for machine in M_intersection:
                        # Binary variable - 1 if (job_a, task_a) precedes
                        # (job_b, task_b) on machine. 0, otherwise.
                        Y[job_a, task_a, job_b, task_b, machine] = solver.IntVar(
                            0, 1, f'Y{job_a}{task_a}{job_b}{task_b}{machine}'
                        )
                    
    for job_idx, job in enumerate(job_list):
        combos = combinations(range(len(job)), 2)
        for combo in combos:
            M_intersection = intersection(
                Mj[job[combo[0]]["station_type"]],
                Mj[job[combo[1]]["station_type"]]
            )
            for machine in M_intersection:
                # Binary variable - 1 if (job_idx, combo[0]) precedes
                # (job_idx, combo[1]) on machine. 0, otherwise.
                Z[job_idx, combo[0], combo[1], machine] = solver.IntVar(
                    0, 1, f'Z{job_idx}{combo[0]}{combo[1]}{machine}'
                )

    # Overall makespan.
    C_max = solver.Var(0, horizon, False, 'makespan')

    return X, Y, Z, S, C, S_job, C_job, C_max

def define_constraints(solver, X, Y, Z, S, C, S_job, C_job, C_max, job_list, parent_ids, Mj):
    '''Defines all the optimization constraints.'''
    # Large number for slack variables. This value has to be larger than the
    # maximum horizon + a buffer, or the program will be infeasible.
    L = 10000

    # Job-specific constraints.
    for job_idx, job in enumerate(job_list):
        for task_idx, task in enumerate(job):
            # Each operation can only be assigned to one machine.
            solver.Add( 
                sum(X[job_idx, task_idx, machine] for machine in Mj[task["station_type"]]) == 1
            )

            # Within a job, each task must start after the all parent tasks end.
            for parent in parent_ids[job_idx][task_idx]:
                solver.Add(
                    sum(S[job_idx, task_idx, machine] for machine in Mj[task["station_type"]]) >=
                    sum(C[job_idx, parent, machine] for machine in Mj[job[parent]["station_type"]])
                )

    for job_idx, job in enumerate(job_list):
        combos = combinations(range(len(job)), 2)
        for combo in combos:
            M_intersection = intersection(
                Mj[job[combo[0]]["station_type"]],
                Mj[job[combo[1]]["station_type"]]
            )
            # No two tasks within the same job can overlap on the same machine.
            for machine in M_intersection:
                solver.Add(
                    S[job_idx, combo[0], machine] >=
                    C[job_idx, combo[1], machine] -
                        Z[job_idx, combo[0], combo[1], machine]*L
                )
                solver.Add(
                    S[job_idx, combo[1], machine] >=
                    C[job_idx, combo[0], machine] -
                        (1-Z[job_idx, combo[0], combo[1], machine])*L
                )

    # Task completion constraints.
    for job_idx, job in enumerate(job_list):
        for task_idx, task in enumerate(job):
            # Set constraints for all machines that match the task requirement.
            for machine in Mj[task["station_type"]]:
                # The start and end time must equal zero if the task is not
                # assigned to that machine.
                solver.Add(
                    S[job_idx, task_idx, machine] + C[job_idx, task_idx, machine] <=
                    X[job_idx, task_idx, machine]*L
                )

                # Task completion must happen after task duration + start time.
                solver.Add(
                    C[job_idx, task_idx, machine] >=
                    S[job_idx, task_idx, machine] + task["time_left"] - 
                    (1 - X[job_idx, task_idx, machine])*L
                )
                # Ensure that the completion time is exactly start + duration.
                solver.Add(
                    S[job_idx, task_idx, machine] + task["time_left"] 
                        >= C[job_idx, task_idx, machine]
                )

    # Precedence constraints. Iterate between every job-task-job-task pair to
    # make sure that no two tasks are assigned to the same machine at the
    # same time.
    for job_b in range(1, len(job_list)):
        for job_a in range(job_b):
            for task_b in range(len(job_list[job_b])):
                for task_a in range(len(job_list[job_a])):
                    # Check if the task-task pair have overlapping machines.
                    M_intersection = intersection(
                        Mj[job_list[job_b][task_b]["station_type"]],
                        Mj[job_list[job_a][task_a]["station_type"]]
                    )   
                    # print(M_intersection)
                    for machine in M_intersection:
                        solver.Add(
                            S[job_a, task_a, machine] >=
                                C[job_b, task_b, machine] - 
                                Y[job_a, task_a, job_b, task_b, machine]*L
                        )
                        solver.Add(
                            S[job_b, task_b, machine] >=
                                C[job_a, task_a, machine] - 
                                (1-Y[job_a, task_a, job_b, task_b, machine])*L
                        )

    # Overall job start and completion constraints.
    for job_idx, job in enumerate(job_list):
        # TODO: Streamline this if possible.
        for task_idx, task in enumerate(job):
            # Job's start must be before the first task's start time.
            solver.Add(
                S_job[job_idx] <= 
                    sum(S[job_idx, task_idx, machine] for machine in Mj[job[task_idx]["station_type"]])
            )
            # Job's completion must be after the last task's completion time.
            solver.Add(
                C_job[job_idx] >= 
                    sum(C[job_idx, task_idx, machine] for machine in Mj[job[task_idx]["station_type"]])
            )
        
        # The overall makespan must be after the last job's completion.
        solver.Add(
            C_max >= C_job[job_idx]
        )

# (solver, X, Y, Z, S, C, S_job, C_job, C_max, job_list, parent_ids, Mj):
def respect_ongoing_constraints(solver, X, S, job_list: list, ongoing: dict):
    '''Adds constraints to ensure ongoing tasks are assigned to the
    same stations and start at time zero.
    '''
    for ticket_id, ticket in ongoing.items():
        # Get the ticket location in the job list and its assigned station.
        job_idx, task_idx = get_task_idx_in_job(ticket_id, job_list)
        station_num = ticket["station_num"]

        # Set the 
        solver.Add(
            X[job_idx, task_idx, station_num] == 1
        )
        solver.Add(
            S[job_idx, task_idx, station_num] == 0
        )

def create_idle_time_objective(solver, S, C, C_max, job_list, parent_ids, Mj, optimum):
    '''Sets the schedule's total idle time as the objective.'''
    # Add constraint for makespan based on previously found optimum.
    solver.Add(C_max <= optimum)

    # Define the objective function to minimize the idle time.
    idle_times = []
    for job_idx, job in enumerate(job_list):
        for task_idx, task in enumerate(job):
            # Idle time between each task's start and its parents' completion.
            for parent in parent_ids[job_idx][task_idx]:
                idle_times.append(
                    sum(S[job_idx, task_idx, machine] for machine in Mj[job[task_idx]["station_type"]])
                    - sum(C[job_idx, parent, machine] for machine in Mj[job[parent]["station_type"]])
                )
    solver.Minimize(solver.Sum(idle_times))
