'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Constants useful across scripts.
'''
station_names = [
    "Loading Area",
    "Mega Stitch",
    "RF",
    "Perimeter",
    "Inspection"
]

# Station type numbers are used for specifying tasks in a job. 
# Num stations is how many of each type there are.
station_type_numbers = list(range(len(station_names)))
num_stations = [1, 2, 1, 3, 1]

# Create increasing station numbers based on how many
# there are of each type.
num = 0
Mj = []
for i in range(len(station_type_numbers)):
    stations = []
    for j in range(1, num_stations[i]+1):
        stations.append(num)
        num += 1
    Mj.append(stations)

all_machines = [i for stations in Mj for i in stations]
# print(f"All machines: {all_machines}")

# Sample set of linear jobs.
linear_jobs_0 = [  # task = (machine_id, processing_time).
    [(0, 3), (2, 2), (2, 2)],
    [(0, 2), (2, 1), (2, 4)],
    [(1, 4), (2, 3)]
]

# Set of 7 linear jobs.
linear_jobs = [
	[(0,5), (2,20), (3,40), (4,40)],
	[(0,5), (1,40), (3,50), (4,40)],
	[(0,5), (1,40), (3,50), (4,40)],
	[(0,5), (1,30), (2,30), (3,60), (4,50)],
	[(0,5), (2,30), (1,20), (3,35), (4,40)],
    [(0,5), (2,30), (1,20), (3,35), (4,40)],
	[(0,5), (1,30), (2,20), (1,30), (3,45), (4,60)]
]

# Set of 7 jobs - some linear, some tree.
tree_jobs =  [
    [ # Tree Job.
        {"ticket_id": 0, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 1, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 2, "station_type": 1, "duration": 20, "parents": [0]},
        {"ticket_id": 3, "station_type": 2, "duration": 30, "parents": [1]},
        {"ticket_id": 4, "station_type": 3, "duration": 50, "parents": [2,3]},
        {"ticket_id": 5, "station_type": 4, "duration": 40, "parents": [4]},
    ],
    [ # Linear Job.
        {"ticket_id": 6, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 7, "station_type": 1, "duration": 40, "parents": [6]},
        {"ticket_id": 8, "station_type": 3, "duration": 50, "parents": [7]},
        {"ticket_id": 9, "station_type": 4, "duration": 40, "parents": [8]},
    ],
    [ # Tree Job.
        {"ticket_id": 10, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 11, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 12, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 13, "station_type": 2, "duration": 40, "parents": [10]},
        {"ticket_id": 14, "station_type": 1, "duration": 10, "parents": [11]},
        {"ticket_id": 15, "station_type": 1, "duration": 30, "parents": [13,12]},
        {"ticket_id": 16, "station_type": 2, "duration": 40, "parents": [14,15]},
        {"ticket_id": 17, "station_type": 3, "duration": 45, "parents": [16]},
        {"ticket_id": 18, "station_type": 4, "duration": 60, "parents": [17]},
    ],
    [ # Tree Job.
        {"ticket_id": 19, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 20, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 21, "station_type": 2, "duration": 30, "parents": [19]},
        {"ticket_id": 22, "station_type": 3, "duration": 30, "parents": [20]},
        {"ticket_id": 23, "station_type": 2, "duration": 30, "parents": [21,22]},
        {"ticket_id": 24, "station_type": 3, "duration": 40, "parents": [23]},
        {"ticket_id": 25, "station_type": 4, "duration": 50, "parents": [24]},
    ],
    [ # Linear Job.
        {"ticket_id": 26, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 27, "station_type": 1, "duration": 40, "parents": [26]},
        {"ticket_id": 28, "station_type": 2, "duration": 30, "parents": [27]},
        {"ticket_id": 29, "station_type": 3, "duration": 60, "parents": [28]},
        {"ticket_id": 30, "station_type": 4, "duration": 50, "parents": [29]},
    ],
    [ # Linear Job.
        {"ticket_id": 31, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 32, "station_type": 2, "duration": 30, "parents": [31]},
        {"ticket_id": 33, "station_type": 1, "duration": 20, "parents": [32]},
        {"ticket_id": 34, "station_type": 3, "duration": 35, "parents": [33]},
        {"ticket_id": 35, "station_type": 4, "duration": 30, "parents": [34]},
    ],
    [ # Tree Job.
        {"ticket_id": 36, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 37, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 38, "station_type": 1, "duration": 30, "parents": [36]},
        {"ticket_id": 39, "station_type": 2, "duration": 20, "parents": [37]},
        {"ticket_id": 40, "station_type": 1, "duration": 10, "parents": [38,39]},
        {"ticket_id": 41, "station_type": 3, "duration": 45, "parents": [40]},
        {"ticket_id": 42, "station_type": 4, "duration": 60, "parents": [41]},
    ],
]