'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Implementation of the flexible job shop problem.
    This version assumes we have multiple machines for some operation types:
        Loading Area - 0
        Mega Stitch - [1, 2]
        RF - 3
        Perimeter - [4, 5]
        Inspection - 6
    Paper:
        Mathematical models for job-shop scheduling problems with routing
        and process plan flexibility - Ozguven et al (2010).
'''

import collections
from ortools.sat.python import cp_model

station_names = [
    "Loading Area",
    "Mega Stitch",
    "RF",
    "Perimeter",
    "Inspection"
]
M = [0, 1, 2, 3, 4]
num_stations = [1, 2, 1, 2, 1]

# Automatically create increasing station numbers based on how many
# there are of each type.
num = 0
Mj = []
print("Station numbers:")
print("[")
for i in range(len(M)):
    stations = []
    for j in range(1, num_stations[i]+1):
        stations.append(num)
        num += 1
    print(f"{station_names[i]:>15}:    {stations}")
    Mj.append(stations)
print("]")


# Job data. Every job starts at the loading area.
jobs_data = [  # task = (machine_id, processing_time).
    [(0, 3), (1, 2), (2, 2)],  # Job0.
    [(0, 2), (2, 1), (1, 4)],  # Job1.
    [(1, 4), (2, 3)]           # Job2.
]

# Maximum horizon if all jobs were sequential.
horizon = sum(task[1] for job in jobs_data for task in job)

# Declare the model.
model = cp_model.CpModel()

for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        for machine in Mj

for job_id, job in enumerate(jobs_data):
    for task_id, task in enumerate(job):
        model.Add(

        )