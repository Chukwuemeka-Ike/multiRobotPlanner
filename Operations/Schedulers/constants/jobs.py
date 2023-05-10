'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Job sets.
'''
# Job set template.
# jobs = [
#     [
#         {"job_id": , "ticket_id": , "station_type": , "duration": , "parents": []},
#         {"job_id": , "ticket_id": , "station_type": , "duration": , "parents": []},
#         {"job_id": , "ticket_id": , "station_type": , "duration": , "parents": []},
#         {"job_id": , "ticket_id": , "station_type": , "duration": , "parents": []},
#         {"job_id": , "ticket_id": , "station_type": , "duration": , "parents": []},
#         {"job_id": , "ticket_id": , "station_type": , "duration": , "parents": []},
#     ],
# ]


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

# Every job starts at the loading area.
# task_template = {"ticket_id": , "station_type": , "duration": , "parents": []}
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

physical_demo_jobs = [
    [ # Tree Job.
        {"ticket_id": 0, "station_type": 0, "duration": 5, "parents": [], "num_robots": 2},
        {"ticket_id": 1, "station_type": 0, "duration": 5, "parents": [], "num_robots": 2},
        {"ticket_id": 2, "station_type": 1, "duration": 40, "parents": [0], "num_robots": 2},
        {"ticket_id": 3, "station_type": 2, "duration": 20, "parents": [1], "num_robots": 2},
        {"ticket_id": 4, "station_type": 1, "duration": 40, "parents": [2,3], "num_robots": 4},
        {"ticket_id": 5, "station_type": 3, "duration": 60, "parents": [4], "num_robots": 4},
        {"ticket_id": 6, "station_type": 4, "duration": 60, "parents": [5], "num_robots": 4},
    ],
    [ # Linear Job.
        {"ticket_id": 31, "station_type": 0, "duration": 5, "parents": [], "num_robots": 3},
        {"ticket_id": 32, "station_type": 2, "duration": 30, "parents": [31], "num_robots": 3},
        {"ticket_id": 33, "station_type": 1, "duration": 20, "parents": [32], "num_robots": 3},
        {"ticket_id": 34, "station_type": 3, "duration": 35, "parents": [33], "num_robots": 3},
        {"ticket_id": 35, "station_type": 4, "duration": 30, "parents": [34], "num_robots": 3},
    ],
    [ # Tree Job.
        {"ticket_id": 14, "station_type": 0, "duration": 5, "parents": [], "num_robots": 2},
        {"ticket_id": 15, "station_type": 0, "duration": 5, "parents": [], "num_robots": 1},
        {"ticket_id": 16, "station_type": 0, "duration": 5, "parents": [], "num_robots": 2},
        {"ticket_id": 17, "station_type": 2, "duration": 40, "parents": [14], "num_robots": 2},
        {"ticket_id": 18, "station_type": 1, "duration": 10, "parents": [15], "num_robots": 1},
        {"ticket_id": 19, "station_type": 1, "duration": 30, "parents": [17,16], "num_robots": 4},
        {"ticket_id": 20, "station_type": 2, "duration": 40, "parents": [18,19], "num_robots": 5},
        {"ticket_id": 21, "station_type": 3, "duration": 45, "parents": [20], "num_robots": 5},
        {"ticket_id": 22, "station_type": 4, "duration": 60, "parents": [21], "num_robots": 5},
    ],
    [ # Tree Job.
        {"ticket_id": 7, "station_type": 0, "duration": 5, "parents": [], "num_robots": 1},
        {"ticket_id": 8, "station_type": 0, "duration": 5, "parents": [], "num_robots": 2},
        {"ticket_id": 9, "station_type": 2, "duration": 25, "parents": [7], "num_robots": 1},
        {"ticket_id": 10, "station_type": 2, "duration": 25, "parents": [8], "num_robots": 2},
        {"ticket_id": 11, "station_type": 1, "duration": 45, "parents": [9,10], "num_robots": 3},
        {"ticket_id": 12, "station_type": 3, "duration": 45, "parents": [11], "num_robots": 3},
        {"ticket_id": 13, "station_type": 4, "duration": 60, "parents": [12], "num_robots": 3},
    ],
]

fifo_jobs = [
    [ # Tree Job.
        {"job_id": 0, "ticket_id": 1, "station_type": 0, "duration": 10, "parents": []},
        {"job_id": 0, "ticket_id": 2, "station_type": 0, "duration": 5, "parents": []},
        {"job_id": 0, "ticket_id": 3, "station_type": 2, "duration": 40, "parents": [1,2]},
        {"job_id": 0, "ticket_id": 4, "station_type": 3, "duration": 45, "parents": [3]},
        {"job_id": 0, "ticket_id": 5, "station_type": 4, "duration": 30, "parents": [4]},
    ],
    [ # Linear Job.
        {"job_id": 1, "ticket_id": 6, "station_type": 0, "duration": 5, "parents": []},
        {"job_id": 1, "ticket_id": 7, "station_type": 2, "duration": 40, "parents": [6]},
        {"job_id": 1, "ticket_id": 8, "station_type": 3, "duration": 50, "parents": [7]},
        {"job_id": 1, "ticket_id": 9, "station_type": 4, "duration": 40, "parents": [8]},
    ],
    [ # Tree Job.
        {"job_id": 2, "ticket_id": 10, "station_type": 0, "duration": 5, "parents": []},
        {"job_id": 2, "ticket_id": 11, "station_type": 0, "duration": 5, "parents": []},
        {"job_id": 2, "ticket_id": 12, "station_type": 2, "duration": 20, "parents": [10]},
        {"job_id": 2, "ticket_id": 13, "station_type": 2, "duration": 30, "parents": [11]},
        {"job_id": 2, "ticket_id": 14, "station_type": 3, "duration": 50, "parents": [12,13]},
        {"job_id": 2, "ticket_id": 15, "station_type": 4, "duration": 40, "parents": [14]},
    ],
]

for job in fifo_jobs:
    for ticket in job:
        ticket["duration"] *= 2

anchor_jobs = [
    [
        {"job_id": 0, "ticket_id": 0, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 0, "ticket_id": 1, "station_type": 1, "duration": 95.7, "parents": [0]},
        {"job_id": 0, "ticket_id": 2, "station_type": 2, "duration": 44.8, "parents": [1]},
        {"job_id": 0, "ticket_id": 3, "station_type": 3, "duration": 37.6, "parents": [2]},
        {"job_id": 0, "ticket_id": 4, "station_type": 4, "duration": 45, "parents": [3]},
        {"job_id": 0, "ticket_id": 5, "station_type": 7, "duration": 40.6, "parents": [4]},
    ],
    [
        {"job_id": 1, "ticket_id": 6, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 1, "ticket_id": 7, "station_type": 1, "duration": 65.9, "parents": [6]},
        {"job_id": 1, "ticket_id": 8, "station_type": 2, "duration": 42.5, "parents": [7]},
        {"job_id": 1, "ticket_id": 9, "station_type": 3, "duration": 26.9, "parents": [8]},
        {"job_id": 1, "ticket_id": 10, "station_type": 4, "duration": 34, "parents": [9]},
        {"job_id": 1, "ticket_id": 11, "station_type": 7, "duration": 24.6, "parents": [10]},
    ],
    [
        {"job_id": 2, "ticket_id": 12, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 2, "ticket_id": 13, "station_type": 1, "duration": 205.3, "parents": [12]},
        {"job_id": 2, "ticket_id": 14, "station_type": 2, "duration": 67.8, "parents": [13]},
        {"job_id": 2, "ticket_id": 15, "station_type": 3, "duration": 57.5, "parents": [14]},
        {"job_id": 2, "ticket_id": 16, "station_type": 4, "duration": 74, "parents": [15]},
        {"job_id": 2, "ticket_id": 17, "station_type": 7, "duration": 82.8, "parents": [16]},
    ],
    [
        {"job_id": 3, "ticket_id": 18, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 3, "ticket_id": 19, "station_type": 1, "duration": 79.9, "parents": [18]},
        {"job_id": 3, "ticket_id": 20, "station_type": 2, "duration": 42.5, "parents": [19]},
        {"job_id": 3, "ticket_id": 21, "station_type": 3, "duration": 33.8, "parents": [20]},
        {"job_id": 3, "ticket_id": 22, "station_type": 4, "duration": 34, "parents": [21]},
        {"job_id": 3, "ticket_id": 23, "station_type": 7, "duration": 34.7, "parents": [22]},
    ],
    [
        {"job_id": 4, "ticket_id": 24, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 4, "ticket_id": 25, "station_type": 1, "duration": 93.1, "parents": [24]},
        {"job_id": 4, "ticket_id": 26, "station_type": 2, "duration": 51.7, "parents": [25]},
        {"job_id": 4, "ticket_id": 27, "station_type": 3, "duration": 40.5, "parents": [26]},
        {"job_id": 4, "ticket_id": 28, "station_type": 4, "duration": 39, "parents": [27]},
        {"job_id": 4, "ticket_id": 29, "station_type": 7, "duration": 52.7, "parents": [28]},
    ],
    [
        {"job_id": 5, "ticket_id": 30, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 5, "ticket_id": 31, "station_type": 1, "duration": 166.1, "parents": [30]},
        {"job_id": 5, "ticket_id": 32, "station_type": 2, "duration": 60.9, "parents": [31]},
        {"job_id": 5, "ticket_id": 33, "station_type": 3, "duration": 51.5, "parents": [32]},
        {"job_id": 5, "ticket_id": 34, "station_type": 4, "duration": 71.5, "parents": [33]},
        {"job_id": 5, "ticket_id": 35, "station_type": 7, "duration": 67.6, "parents": [34]},
    ],
    [
        {"job_id": 6, "ticket_id": 36, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 6, "ticket_id": 37, "station_type": 5, "duration": 71, "parents": [36]},
        {"job_id": 6, "ticket_id": 38, "station_type": 3, "duration": 73.9, "parents": [37]},
        {"job_id": 6, "ticket_id": 39, "station_type": 6, "duration": 27.4, "parents": [38]},
        {"job_id": 6, "ticket_id": 40, "station_type": 7, "duration": 103.9, "parents": [39]},
    ],
    [
        {"job_id": 7, "ticket_id": 41, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 7, "ticket_id": 42, "station_type": 5, "duration": 72, "parents": [41]},
        {"job_id": 7, "ticket_id": 43, "station_type": 3, "duration": 67.2, "parents": [42]},
        {"job_id": 7, "ticket_id": 44, "station_type": 6, "duration": 14.4, "parents": [43]},
        {"job_id": 7, "ticket_id": 45, "station_type": 7, "duration": 60.1, "parents": [44]},
    ],
    [
        {"job_id": 8, "ticket_id": 46, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 8, "ticket_id": 47, "station_type": 5, "duration": 51, "parents": [46]},
        {"job_id": 8, "ticket_id": 48, "station_type": 3, "duration": 97, "parents": [47]},
        {"job_id": 8, "ticket_id": 49, "station_type": 6, "duration": 21.6, "parents": [48]},
        {"job_id": 8, "ticket_id": 50, "station_type": 7, "duration": 57.2, "parents": [49]},
    ],
    [
        {"job_id": 9, "ticket_id": 51, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 9, "ticket_id": 52, "station_type": 5, "duration": 25, "parents": [51]},
        {"job_id": 9, "ticket_id": 53, "station_type": 3, "duration": 45.8, "parents": [52]},
        {"job_id": 9, "ticket_id": 54, "station_type": 6, "duration": 19.1, "parents": [53]},
        {"job_id": 9, "ticket_id": 55, "station_type": 7, "duration": 31.6, "parents": [54]},
    ],
    [
        {"job_id": 10, "ticket_id": 56, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 10, "ticket_id": 57, "station_type": 5, "duration": 43, "parents": [56]},
        {"job_id": 10, "ticket_id": 58, "station_type": 3, "duration": 62.4, "parents": [57]},
        {"job_id": 10, "ticket_id": 59, "station_type": 6, "duration": 23, "parents": [58]},
        {"job_id": 10, "ticket_id": 60, "station_type": 7, "duration": 77.3, "parents": [59]},
    ],
    [
        {"job_id": 11, "ticket_id": 61, "station_type": 0, "duration": 0.5, "parents": []},
        {"job_id": 11, "ticket_id": 62, "station_type": 5, "duration": 151, "parents": [61]},
        {"job_id": 11, "ticket_id": 63, "station_type": 3, "duration": 84.7, "parents": [62]},
        {"job_id": 11, "ticket_id": 64, "station_type": 6, "duration": 19.4, "parents": [63]},
        {"job_id": 11, "ticket_id": 65, "station_type": 7, "duration": 53.8, "parents": [64]},
    ],
]







complete_ticket_list = {
    # Tree Job 2 roots.
    11: {"job_id": 1, "station_type": 0, "duration": 5, "parents": [], "actual_duration": 5},
    12: {"job_id": 1, "station_type": 0, "duration": 5, "parents": [], "actual_duration": 5},
    13: {"job_id": 1, "station_type": 2, "duration": 45.7, "parents": [11], "actual_duration": 45.7},
    14: {"job_id": 1, "station_type": 3, "duration": 54.8, "parents": [12], "actual_duration": 54.8},
    15: {"job_id": 1, "station_type": 1, "duration": 89.2, "parents": [13,14], "actual_duration": 89.2},
    16: {"job_id": 1, "station_type": 3, "duration": 43.1, "parents": [15], "actual_duration": 43.1},
    17: {"job_id": 1, "station_type": 7, "duration": 38.7, "parents": [16], "actual_duration": 38.7},
    # Tree Job 3 roots.
    1: {"job_id": 0, "station_type": 0, "duration": 10, "parents": [], "actual_duration": 10, "time_left": 10},
    2: {"job_id": 0, "station_type": 0, "duration": 5, "parents": [], "actual_duration": 5, "time_left": 5},
    3: {"job_id": 0, "station_type": 0, "duration": 5, "parents": [], "actual_duration": 7, "time_left": 5},
    4: {"job_id": 0, "station_type": 2, "duration": 45, "parents": [1], "actual_duration": 55, "time_left": 45},
    5: {"job_id": 0, "station_type": 2, "duration": 30, "parents": [2], "actual_duration": 25, "time_left": 30},
    6: {"job_id": 0, "station_type": 3, "duration": 30, "parents": [3], "actual_duration": 28, "time_left": 30},
    7: {"job_id": 0, "station_type": 2, "duration": 60, "parents": [4,5], "actual_duration": 75, "time_left": 60},
    8: {"job_id": 0, "station_type": 2, "duration": 30, "parents": [6,7], "actual_duration": 30, "time_left": 30},
    9: {"job_id": 0, "station_type": 3, "duration": 45, "parents": [8], "actual_duration": 48, "time_left": 45},
    10: {"job_id": 0, "station_type": 4, "duration": 60, "parents": [9], "actual_duration": 60, "time_left": 60},
}

# First pass tickets 1-7.
initial_ticket_list = {}
for ticket in range(1,8):
    initial_ticket_list[ticket] = complete_ticket_list[ticket]

# Then pass ticket 8.

# Then pass tickets 9-10.
last_ticket_list = {}
for ticket in range(9, 11):
    last_ticket_list[ticket] = complete_ticket_list[ticket]