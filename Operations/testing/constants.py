fifo_jobs = [
    [ # Tree Job.
        {"ticket_id": 1, "station_type": 0, "duration": 10, "parents": []},
        {"ticket_id": 2, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 3, "station_type": 2, "duration": 40, "parents": [1,2]},
        {"ticket_id": 4, "station_type": 3, "duration": 45, "parents": [3]},
        {"ticket_id": 5, "station_type": 4, "duration": 30, "parents": [4]},
    ],
    [ # Linear Job.
        {"ticket_id": 6, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 7, "station_type": 2, "duration": 40, "parents": [6]},
        {"ticket_id": 8, "station_type": 3, "duration": 50, "parents": [7]},
        {"ticket_id": 9, "station_type": 4, "duration": 40, "parents": [8]},
    ],
    [ # Tree Job.
        {"ticket_id": 10, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 11, "station_type": 0, "duration": 5, "parents": []},
        {"ticket_id": 12, "station_type": 2, "duration": 20, "parents": [10]},
        {"ticket_id": 13, "station_type": 2, "duration": 30, "parents": [11]},
        {"ticket_id": 14, "station_type": 3, "duration": 50, "parents": [12,13]},
        {"ticket_id": 15, "station_type": 4, "duration": 40, "parents": [14]},
    ],
]

fifo_tasks = [
    # Tree Job 1.
    {"ticket_id": 1, "station_type": 0, "duration": 10, "parents": []},
    {"ticket_id": 2, "station_type": 0, "duration": 5, "parents": []},
    {"ticket_id": 3, "station_type": 2, "duration": 40, "parents": [1,2]},
    {"ticket_id": 4, "station_type": 3, "duration": 45, "parents": [3]},
    {"ticket_id": 5, "station_type": 4, "duration": 30, "parents": [4]},
    # Linear Job 1.
    {"ticket_id": 6, "station_type": 0, "duration": 5, "parents": []},
    {"ticket_id": 7, "station_type": 2, "duration": 40, "parents": [6]},
    {"ticket_id": 8, "station_type": 3, "duration": 50, "parents": [7]},
    {"ticket_id": 9, "station_type": 4, "duration": 40, "parents": [8]},
    # Tree Job 2.
    {"ticket_id": 10, "station_type": 0, "duration": 5, "parents": []},
    {"ticket_id": 11, "station_type": 0, "duration": 5, "parents": []},
    {"ticket_id": 12, "station_type": 2, "duration": 20, "parents": [10]},
    {"ticket_id": 13, "station_type": 2, "duration": 30, "parents": [11]},
    {"ticket_id": 14, "station_type": 3, "duration": 50, "parents": [12,13]},
    {"ticket_id": 15, "station_type": 4, "duration": 40, "parents": [14]},
]

fifo_tasks = {
    1: {"station_type": 0, "duration": 10, "parents": [], "time_left": 0},
    2: {"station_type": 0, "duration": 5, "parents": [], "time_left": 0},
    3: {"station_type": 2, "duration": 40, "parents": [1,2], "time_left": 0},
    4: {"station_type": 3, "duration": 45, "parents": [3], "time_left": 0},
    5: {"station_type": 4, "duration": 30, "parents": [4], "time_left": 0},

    # Linear Job 1.
    6: {"station_type": 0, "duration": 5, "parents": [], "time_left": 0},
    7: {"station_type": 2, "duration": 40, "parents": [6], "time_left": 0},
    8: {"station_type": 3, "duration": 50, "parents": [7], "time_left": 0},
    9: {"station_type": 4, "duration": 40, "parents": [8], "time_left": 0},

    # Tree Job 2.
    10: {"station_type": 0, "duration": 5, "parents": [], "time_left": 0},
    11: {"station_type": 0, "duration": 5, "parents": [], "time_left": 0},
    12: {"station_type": 2, "duration": 20, "parents": [10], "time_left": 0},
    13: {"station_type": 2, "duration": 30, "parents": [11], "time_left": 0},
    23: {"station_type": 3, "duration": 50, "parents": [12,13], "time_left": 0},
    24: {"station_type": 4, "duration": 40, "parents": [23], "time_left": 0},

    # Tree Job 3 roots.
    14: {"station_type": 0, "duration": 5, "parents": [], "time_left": 0},
    15: {"station_type": 0, "duration": 5, "parents": [], "time_left": 0},
    16: {"station_type": 0, "duration": 5, "parents": [], "time_left": 0},
    17: {"station_type": 2, "duration": 40, "parents": [14], "time_left": 0},
    18: {"station_type": 1, "duration": 10, "parents": [15], "time_left": 0},
    19: {"station_type": 1, "duration": 30, "parents": [17,16], "time_left": 0},
    20: {"station_type": 2, "duration": 40, "parents": [18,19], "time_left": 0},
    21: {"station_type": 3, "duration": 45, "parents": [20], "time_left": 0},
    22: {"station_type": 4, "duration": 60, "parents": [21], "time_left": 0},

    # Tree Job 2 branch joins.
    25: {"station_type": 0, "duration": 5, "parents": [], "time_left": 0},
    26: {"station_type": 0, "duration": 5, "parents": [], "time_left": 0},
    27: {"station_type": 0, "duration": 5, "parents": [], "time_left": 0},
    28: {"station_type": 2, "duration": 40, "parents": [25], "time_left": 0},
    29: {"station_type": 1, "duration": 10, "parents": [26], "time_left": 0},
    30: {"station_type": 1, "duration": 30, "parents": [28,27], "time_left": 0},
    31: {"station_type": 2, "duration": 40, "parents": [29,30], "time_left": 0},
    32: {"station_type": 3, "duration": 45, "parents": [31], "time_left": 0},
    33: {"station_type": 4, "duration": 60, "parents": [32], "time_left": 0},
}