'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Implementation of the vanilla job shop problem.
    This version assumes we have one machine for each operation type:
        Loading Area - 0
        Mega Stitch - 1
        RF - 2
        Perimeter - 3
        Inspection - 4
'''
import collections
import time
from ortools.sat.python import cp_model

# Time everything.
overallStart = time.time()

# Job data. Every job starts at the loading area.
jobs_data = [
	[(0,5), (1,30), (3,20), (4,20)],
	[(0,5), (2,20), (3,10), (4,10)],
	[(0,5), (1,20), (2,15), (3,40), (4, 30)],
	[(0,5), (2,30), (1,20), (3,25), (4,40)]
]
# [print(job) for job in jobs_data]

# The number of machines is currently the max number in the job data.
# We need a way to have more than one type of machine.
machines_count = 1 + max(task[0] for job in jobs_data for task in job)
all_machines = range(machines_count)

# Maximum horizon if all jobs were sequential.
horizon = sum(task[1] for job in jobs_data for task in job)

# Declare the model.
model = cp_model.CpModel()

# Creates job intervals and add to the corresponding machine lists.
all_tasks = {}
machine_to_intervals = collections.defaultdict(list)

# Named tuple to store information about created variables.
task_type = collections.namedtuple('task_type', 'start end interval')

# Named tuple to manipulate solution information.
assigned_task_type = collections.namedtuple('assigned_task_type',
                                            'start job index duration')

for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        # print(job, task)

        machine = task[0]
        duration = task[1]

        suffix = '_%i_%i' % (job_id, task_id)

        start_var = model.NewIntVar(0, horizon, 'start' + suffix)
        end_var = model.NewIntVar(0, horizon, 'end' + suffix)

        interval_var = model.NewIntervalVar(start_var, duration, end_var,
                                            'interval' + suffix)
        all_tasks[job_id, task_id] = task_type(start=start_var,
                                               end=end_var,
                                               interval=interval_var)
        machine_to_intervals[machine].append(interval_var)

    #     print(start_var)
    #     print(end_var)
    #     print(interval_var)
    #     print(all_tasks[job_id, task_id])
    #     print(machine_to_intervals[machine])

    #     break
    # break

# Define the constraints.
# Create and add disjunctive constraints.
for machine in all_machines:
    model.AddNoOverlap(machine_to_intervals[machine])

# Precedences inside a job.
for job_id, job in enumerate(jobs_data):
    for task_id in range(len(job) - 1):
        model.Add(all_tasks[job_id, task_id +
                            1].start >= all_tasks[job_id, task_id].end)

# Define the objective.
# Makespan objective.
obj_var = model.NewIntVar(0, horizon, 'makespan')
model.AddMaxEquality(obj_var, [
    all_tasks[job_id, len(job) - 1].end
    for job_id, job in enumerate(jobs_data)
])
model.Minimize(obj_var)

# Invoke the solver.
solutionStart = time.time()
solver = cp_model.CpSolver()
status = solver.Solve(model)
print(f"Solution runtime: {time.time() - solutionStart: .3f} seconds.")
print()

# Display the results.
if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    print('Solution:')
    # Create one list of assigned tasks per machine.
    assigned_jobs = collections.defaultdict(list)
    for job_id, job in enumerate(jobs_data):
        for task_id, task in enumerate(job):
            machine = task[0]
            assigned_jobs[machine].append(
                assigned_task_type(start=solver.Value(
                    all_tasks[job_id, task_id].start),
                                   job=job_id,
                                   index=task_id,
                                   duration=task[1]))

    # Create per machine output lines.
    output = ''
    for machine in all_machines:
        # Sort by starting time.
        assigned_jobs[machine].sort()
        sol_line_tasks = f'Machine {str(machine)}: '
        sol_line = '           '

        for assigned_task in assigned_jobs[machine]:
            name = f'job_{assigned_task.job}_task_{assigned_task.index}'

            # Add spaces to output to align columns.
            sol_line_tasks += '%-15s' % name

            start = assigned_task.start
            duration = assigned_task.duration
            sol_tmp = f'[{start}, {start+duration}]'
            # Add spaces to output to align columns.
            sol_line += '%-15s' % sol_tmp

        sol_line += '\n'
        sol_line_tasks += '\n'
        output += sol_line_tasks
        output += sol_line

    # Finally print the solution found.
    print(f'Max horizon Length: {horizon}')
    print(f'Optimal Schedule Length: {solver.ObjectiveValue()}')
    print(output)
else:
    print('No solution found.')

print(f"Overall runtime: {time.time() - overallStart: .3f} seconds.")
print()