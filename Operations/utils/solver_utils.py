from itertools import combinations



def intersection(lst1, lst2):
    '''Returns the common items between two lists.'''
    return list(set(lst1) & set(lst2))


def create_opt_variables(solver, jobs_data, horizon, all_machines, Mj):
    '''Creates all the optimization variables for the MILP problem.

    Returns:
        X: Binary variable - 1 if job i task j is assigned to machine k.
        Y: Binary variable - 1 if (job_a, task_a) precedes
            (job_b, task_b) on machine k. 0, otherwise.
        Z: Binary variable - 1 if (job_id, task_a) precedes
            (job_id, task_b) on machine k. 0, otherwise.
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
    C_job = {}

    num_jobs = len(jobs_data)

    for job in range(len(jobs_data)):
        for task in range(len(jobs_data[job])):
            for machine in all_machines:
                # Binary variable that is 1 if (job, task) is assigned to machine.
                X[job, task, machine] = solver.IntVar(0, 1, f'X{job}{task}{machine}')

                # Start time of (job, task) on machine if assigned. 0, otherwise.
                S[job, task, machine] = solver.Var(0, horizon, False, f'S{job}{task}{machine}')

                # Completion time of (job, task) on machine if assigned. 0, otherwise.
                C[job, task, machine] = solver.Var(0, horizon, False, f'C{job}{task}{machine}')

    for i in range(num_jobs):
        # Completion time of each job.
        C_job[i] = solver.Var(0, horizon, False, 'Ci')

    for job_b in range(1, len(jobs_data)):
        for job_a in range(job_b):
            for task_b in range(len(jobs_data[job_b])):
                for task_a in range(len(jobs_data[job_a])):
                    M_intersection = intersection(
                        Mj[jobs_data[job_b][task_b]["station_type"]],
                        Mj[jobs_data[job_a][task_a]["station_type"]]
                    )
                    for machine in M_intersection:
                        # Binary variable - 1 if (job_a, task_a) precedes
                        # (job_b, task_b) on machine. 0, otherwise.
                        Y[job_a, task_a, job_b, task_b, machine] = solver.IntVar(
                            0, 1, f'Y{job_a}{task_a}{job_b}{task_b}{machine}'
                        )

    for job_id, job in enumerate(jobs_data):
        combos = combinations(range(len(job)), 2)
        for combo in combos:
            M_intersection = intersection(
                Mj[job[combo[0]]["station_type"]],
                Mj[job[combo[1]]["station_type"]]
            )
            for machine in M_intersection:
                # Binary variable - 1 if (job_id, combo[0]) precedes
                # (job_id, combo[1]) on machine. 0, otherwise.
                Z[job_id, combo[0], combo[1], machine] = solver.IntVar(
                    0, 1, f'Z{job_id}{combo[0]}{combo[1]}{machine}'
                )

    # Overall makespan.
    C_max = solver.IntVar(0, horizon, 'makespan')

    return X, Y, Z, S, C, C_job, C_max

def define_constraints(solver, X, Y, Z, S, C, C_job, C_max, jobs_data, parent_ids, Mj):
    '''Defines all the optimization constraints.'''
    # Large number for slack variables.
    L = 1000

    # Job-specific constraints.
    j = 0
    for job_id, job in enumerate(jobs_data):
        for task_id, task in enumerate(job):
            # Each operation can only be assigned to one machine.
            solver.Add( 
                sum(X[job_id, task_id, machine] for machine in Mj[task["station_type"]]) == 1
            )

            # Within a job, each task must start after the previous parent task ends.
            if task_id > 0:
                for parent in parent_ids[j]:
                    solver.Add(
                        sum(S[job_id, task_id, machine] for machine in Mj[task["station_type"]]) >=
                        sum(C[job_id, parent, machine] for machine in Mj[job[parent]["station_type"]])
                    )
                    # print(parent, Mj[job[parent]["station_type"]])
            j += 1
    
    for job_id, job in enumerate(jobs_data):
        combos = combinations(range(len(job)), 2)
        for combo in combos:
            M_intersection = intersection(
                Mj[job[combo[0]]["station_type"]],
                Mj[job[combo[1]]["station_type"]]
            )
            for machine in M_intersection:
                solver.Add(
                    S[job_id, combo[0], machine] >=
                    C[job_id, combo[1], machine] -
                        Z[job_id, combo[0], combo[1], machine]*L
                )
                solver.Add(
                    S[job_id, combo[1], machine] >=
                    C[job_id, combo[0], machine] -
                        (1-Z[job_id, combo[0], combo[1], machine])*L
                )

    # Task completion constraints.
    for job_id, job in enumerate(jobs_data):
        for task_id, task in enumerate(job):
            # Set constraints for all machines that match the task requirement.
            for machine in Mj[task["station_type"]]:
                # The start and end time must equal zero if the task is not
                # assigned to that machine.
                solver.Add(
                    S[job_id, task_id, machine] + C[job_id, task_id, machine] <=
                    X[job_id, task_id, machine]*L
                )

                # Task completion must happen after task duration + start time.
                solver.Add(
                    C[job_id, task_id, machine] >=
                    S[job_id, task_id, machine] + task["duration"] - 
                    (1 - X[job_id, task_id, machine])*L
                )

    # Precedence constraints. Iterate between every job-task-job-task pair to
    # make sure that no two tasks are assigned to the same machine at the
    # same time.
    for job_b in range(1, len(jobs_data)):
        for job_a in range(job_b):
            for task_b in range(len(jobs_data[job_b])):
                for task_a in range(len(jobs_data[job_a])):
                    # Check if the task-task pair have overlapping machines.
                    M_intersection = intersection(
                        Mj[jobs_data[job_b][task_b]["station_type"]],
                        Mj[jobs_data[job_a][task_a]["station_type"]]
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

    # Overall job completion constraints.
    for job_id, job in enumerate(jobs_data):    
        # Job's completion must be after the last task's completion time.
        solver.Add(
            C_job[job_id] >= 
                sum(C[job_id, len(job)-1, machine] for machine in Mj[job[-1]["station_type"]])
        )
        
        # The overall makespan must be after the last job's completion.
        solver.Add(
            C_max >= C_job[job_id]
        )