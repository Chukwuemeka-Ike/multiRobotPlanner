from itertools import combinations



def intersection(lst1, lst2):
    '''Returns the common items between two lists.'''
    return list(set(lst1) & set(lst2))


def create_opt_variables(solver, jobs_data, horizon, all_machines, Mj):
    '''.'''
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
                X[job, task, machine] = solver.IntVar(0, 1, f'X{job}{task}{machine}')
                
                S[job, task, machine] = solver.IntVar(0, horizon, f'S{job}{task}{machine}')
                C[job, task, machine] = solver.IntVar(0, horizon, f'C{job}{task}{machine}')

    for i in range(num_jobs):
        C_job[i] = solver.IntVar(0, horizon, 'Ci')

    for job_b in range(1, len(jobs_data)):
        for job_a in range(job_b):
            for task_b in range(len(jobs_data[job_b])):
                for task_a in range(len(jobs_data[job_a])):
                    M_intersection = intersection(
                        Mj[jobs_data[job_b][task_b]["station_type"]],
                        Mj[jobs_data[job_a][task_a]["station_type"]]
                    )
                    # print(M_intersection)
                    for machine in M_intersection:
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
                Z[job_id, combo[0], combo[1], machine] = solver.IntVar(
                    0, 1, f'Z{job_id}{combo[0]}{combo[1]}{machine}'
                )
    return X, Y, Z, S, C, C_job

def define_constraints(solver, X, Y, Z, S, C, C_job, jobs_data, Mj):
    pass