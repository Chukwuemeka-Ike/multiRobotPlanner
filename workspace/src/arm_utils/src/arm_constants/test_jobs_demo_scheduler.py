'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    2-Job set representing the planned scheduler demo.
'''
complete_ticket_list = {
    # Job 1.
    1: {"ticket_id": 1, "machine_type": 0, "duration": 10, "parents": [], "num_robots": 3},
    2: {"ticket_id": 2, "machine_type": 1, "duration": 60, "parents": [1]},
    3: {"ticket_id": 3, "machine_type": 3, "duration": 47.2, "parents": [2]},
    4: {"ticket_id": 4, "machine_type": 4, "duration": 35, "parents": [3]},

    # Job 2.
    5: {"ticket_id": 5, "machine_type": 0, "duration": 25, "parents": [], "num_robots": 0},
    6: {"ticket_id": 6, "machine_type": 2, "duration": 85, "parents": [5]},
    7: {"ticket_id": 7, "machine_type": 3, "duration": 65, "parents": [6]},
    8: {"ticket_id": 8, "machine_type": 4, "duration": 50, "parents": [7]},

    # Job 3.
    9: {"ticket_id": 9, "machine_type": 0, "duration": 10, "parents": [], "num_robots": 2},
    10: {"ticket_id": 10, "machine_type": 2, "duration": 45, "parents": [9]},
    11: {"ticket_id": 11, "machine_type": 3, "duration": 30, "parents": [10]},
    12: {"ticket_id": 12, "machine_type": 4, "duration": 20, "parents": [11]},

    # Job 4.
    13: {"ticket_id": 13, "machine_type": 0, "duration": 7, "parents": [], "num_robots": 2},
    14: {"ticket_id": 14, "machine_type": 1, "duration": 40, "parents": [13]},
    15: {"ticket_id": 15, "machine_type": 3, "duration": 65, "parents": [14]},
    16: {"ticket_id": 16, "machine_type": 4, "duration": 50, "parents": [15]},

    # Job 5.
    17: {"ticket_id": 17, "machine_type": 0, "duration": 15, "parents": [], "num_robots": 3},
    18: {"ticket_id": 18, "machine_type": 2, "duration": 60, "parents": [17]},
    19: {"ticket_id": 19, "machine_type": 3, "duration": 75, "parents": [18]},
    20: {"ticket_id": 20, "machine_type": 4, "duration": 50, "parents": [19]},

    # Job 6.
    21: {"ticket_id": 21, "machine_type": 0, "duration": 20, "parents": [], "num_robots": 4},
    22: {"ticket_id": 22, "machine_type": 1, "duration": 40, "parents": [21]},
    23: {"ticket_id": 23, "machine_type": 3, "duration": 55, "parents": [22]},
    24: {"ticket_id": 24, "machine_type": 4, "duration": 40, "parents": [23]},

    # Job 7.
    25: {"ticket_id": 25, "machine_type": 0, "duration": 15, "parents": [], "num_robots": 3},
    26: {"ticket_id": 26, "machine_type": 1, "duration": 30, "parents": [25]},
    27: {"ticket_id": 27, "machine_type": 3, "duration": 55, "parents": [26]},
    28: {"ticket_id": 28, "machine_type": 4, "duration": 40, "parents": [27]},
}