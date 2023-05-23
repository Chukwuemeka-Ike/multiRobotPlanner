'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Implementation of the flexible job shop problem using
    Google OR-Tools Linear Solver.

    Paper:
        Mathematical models for job-shop scheduling problems with routing
        and process plan flexibility - Ozguven et al (2010).
'''
import collections
import math
import time

from ortools.sat.python import cp_model

from constants.jobs import *
from constants.stations import *
# from constants.jobs_no_overlap import *
# from constants.stations_no_overlap import *
from utils.draw_utils import draw_tree_schedule
from utils.display_utils import *
from utils.job_utils import *
from utils.sched_utils import *
from utils.solver_utils import *

# Job data.
# jobs_data = tree_jobs
# jobs_data = fifo_jobs
# jobs_data = physical_demo_jobs
jobs_data = anchor_jobs
# jobs_data = convert_task_list_to_job_list(complete_ticket_list)
for job in jobs_data:
    for task in job:
        task["time_left"] = math.ceil(task["duration"])

# # Print out the job and station information.
# display_job_data(jobs_data)
# display_station_numbers(station_type_names, Mj)

# Get the indices of each task's parents in the job list.
# This is done now to ease lookup later.
parent_ids = get_task_parent_indices(jobs_data)
# for i in range(len(parent_ids)):
#     print(parent_ids[i])

# Maximum horizon if all jobs and tasks were done in sequence.
horizon = math.ceil(sum(task["duration"] for job in jobs_data for task in job))
print(f"Max Schedule Length: {horizon: .2f}")


def create_and_solve_program():
    '''Creates a new model and solves the optimization problem.'''
    # Declare the model for the problem.
    model = cp_model.CpModel()


    # *****************************************************************************
    # Create optimization variables.
    # Named tuple to store information about created variables.
    task_type = collections.namedtuple('task_type', 'start end interval')
    # Named tuple to manipulate solution information.
    assigned_task_type = collections.namedtuple('assigned_task_type',
                                                'start job index duration')

    X = {}
    Y = {}
    Z = {}
    S = {}
    C = {}
    S_job = {}
    C_job = {}


    job_list = jobs_data
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
                        Mj[job_list[job_b][task_b]["station_type"]],
                        Mj[job_list[job_a][task_a]["station_type"]]
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
                Mj[job[combo[0]]["station_type"]],
                Mj[job[combo[1]]["station_type"]]
            )
            for machine in M_intersection:
                # Binary variable - 1 if (job_idx, combo[0]) precedes
                # (job_idx, combo[1]) on machine. 0, otherwise.
                Z[job_idx, combo[0], combo[1], machine] = model.NewBoolVar(
                    f'Z{job_idx}{combo[0]}{combo[1]}{machine}'
                )

    # Overall makespan.
    C_max = model.NewIntVar(0, horizon, 'makespan')
    
    # *****************************************************************************














    # *****************************************************************************
    # Define the constraints.
    L = 10000

    # Job-specific constraints.
    for job_idx, job in enumerate(job_list):
        for task_idx, task in enumerate(job):
            # Each operation can only be assigned to one machine.
            model.AddExactlyOne(
                X[job_idx, task_idx, machine] for machine in Mj[task["station_type"]]
            )

            # Within a job, each task must start after the all parent tasks end.
            for parent in parent_ids[job_idx][task_idx]:
                model.Add(
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
                model.Add(
                    S[job_idx, combo[0], machine] >=
                    C[job_idx, combo[1], machine] -
                        Z[job_idx, combo[0], combo[1], machine]*L
                )
                model.Add(
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
                    model.Add(
                        S[job_idx, task_idx, machine] + C[job_idx, task_idx, machine] == 0
                    ).OnlyEnforceIf(X[job_idx, task_idx, machine].Not())

                    # Ensure that the completion time is exactly start + duration.
                    model.Add(
                        S[job_idx, task_idx, machine] + task["time_left"] 
                            == C[job_idx, task_idx, machine]
                    ).OnlyEnforceIf(X[job_idx, task_idx, machine])

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
                            model.Add(
                                S[job_a, task_a, machine] >=
                                    C[job_b, task_b, machine] - 
                                    Y[job_a, task_a, job_b, task_b, machine]*L
                            )
                            model.Add(
                                S[job_b, task_b, machine] >=
                                    C[job_a, task_a, machine] - 
                                    (1-Y[job_a, task_a, job_b, task_b, machine])*L
                            )

        # Overall job start and completion constraints.
        for job_idx, job in enumerate(job_list):
            # TODO: Streamline this if possible.
            for task_idx, task in enumerate(job):
                # Job's start must be before the first task's start time.
                model.Add(
                    S_job[job_idx] <= 
                        sum(S[job_idx, task_idx, machine] for machine in Mj[job[task_idx]["station_type"]])
                )
                # Job's completion must be after the last task's completion time.
                model.Add(
                    C_job[job_idx] >= 
                        sum(C[job_idx, task_idx, machine] for machine in Mj[job[task_idx]["station_type"]])
                )
            
            # The overall makespan must be after the last job's completion.
            model.Add(
                C_max >= C_job[job_idx]
            )

    # Define the objective function to minimize the makespan, then
    # display some solver information.
    model.Minimize(C_max)
    # display_solver_information(solver)
    # print()

    # Creates the solver and solve.
    # solutionStart = time.time()
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 10.0
    status = solver.Solve(model)
    # solutionEnd = time.time()

    # Display the solution.
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        # Statistics.
        # print('\nStatistics')
        print(f"  objective value: {solver.ObjectiveValue(): .2f}")
        # print(f'  status   : {solver.StatusName(status)}')
        # print(f'  conflicts: {solver.NumConflicts()}')
        # print(f'  branches : {solver.NumBranches()}')
        # print(f'  wall time: {solver.WallTime()} s')
        # print(f"  solution runtime: {solutionEnd-solutionStart: .2f} seconds.")
    else:
        print("Infeasible program. Exiting.\n")
        return


if __name__ == '__main__':
    for i in range(30):
        create_and_solve_program()