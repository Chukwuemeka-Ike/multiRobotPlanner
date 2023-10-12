'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    5-Job set for the user demo.
'''
complete_ticket_list = {
    # Job 1.
    1: {"ticket_id": 1, "machine_type": 0, "actual_duration": 5, "duration": 10, "parents": [], "num_robots": 3},
    2: {"ticket_id": 2, "machine_type": 1, "actual_duration": 55, "duration": 60, "parents": [1]},
    3: {"ticket_id": 3, "machine_type": 3, "actual_duration": 44.8, "duration": 47.2, "parents": [2]},
    4: {"ticket_id": 4, "machine_type": 4, "actual_duration": 37.6, "duration": 35, "parents": [3]},

    5: {"ticket_id": 5, "machine_type": 0, "actual_duration": 5, "duration": 10, "parents": [], "num_robots": 3},
    6: {"ticket_id": 6, "machine_type": 1, "actual_duration": 55, "duration": 60, "parents": [5]},
    7: {"ticket_id": 7, "machine_type": 3, "actual_duration": 44.8, "duration": 47.2, "parents": [6]},
    8: {"ticket_id": 8, "machine_type": 4, "actual_duration": 37.6, "duration": 35, "parents": [7]},

    9: {"ticket_id": 9, "machine_type": 0, "actual_duration": 5, "duration": 10, "parents": [], "num_robots": 3},
    10: {"ticket_id": 10, "machine_type": 1, "actual_duration": 55, "duration": 60, "parents": [9]},
    11: {"ticket_id": 11, "machine_type": 3, "actual_duration": 44.8, "duration": 47.2, "parents": [10]},
    12: {"ticket_id": 12, "machine_type": 4, "actual_duration": 37.6, "duration": 35, "parents": [11]},

    13: {"ticket_id": 13, "machine_type": 0, "actual_duration": 5, "duration": 10, "parents": [], "num_robots": 3},
    14: {"ticket_id": 14, "machine_type": 1, "actual_duration": 55, "duration": 60, "parents": [13]},
    15: {"ticket_id": 15, "machine_type": 3, "actual_duration": 44.8, "duration": 47.2, "parents": [14]},
    16: {"ticket_id": 16, "machine_type": 4, "actual_duration": 37.6, "duration": 35, "parents": [15]},

    17: {"ticket_id": 17, "machine_type": 0, "actual_duration": 5, "duration": 10, "parents": [], "num_robots": 3},
    18: {"ticket_id": 18, "machine_type": 1, "actual_duration": 55, "duration": 60, "parents": [17]},
    19: {"ticket_id": 19, "machine_type": 3, "actual_duration": 44.8, "duration": 47.2, "parents": [18]},
    20: {"ticket_id": 20, "machine_type": 4, "actual_duration": 37.6, "duration": 35, "parents": [19]},
}