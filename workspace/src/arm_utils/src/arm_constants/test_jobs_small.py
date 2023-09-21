'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Small subset of Anchor jobs for working with fewer tickets.
'''
complete_ticket_list = {
    0: {"ticket_id": 0, "machine_type": 0, "actual_duration": 5, "duration": 10, "parents": [], "num_robots": 3},
    1: {"ticket_id": 1, "machine_type": 1, "actual_duration": 95.7, "duration": 85, "parents": [0]},
    2: {"ticket_id": 2, "machine_type": 2, "actual_duration": 44.8, "duration": 47.2, "parents": [1]},
    3: {"ticket_id": 3, "machine_type": 3, "actual_duration": 37.6, "duration": 40, "parents": [2]},
    4: {"ticket_id": 4, "machine_type": 4, "actual_duration": 45, "duration": 50, "parents": [3]},
    5: {"ticket_id": 5, "machine_type": 7, "actual_duration": 40.6, "duration": 44.2, "parents": [4]},

    6: {"ticket_id": 6, "machine_type": 0, "actual_duration": 5, "duration": 5, "parents": [], "num_robots": 1},
    7: {"ticket_id": 7, "machine_type": 1, "actual_duration": 65.9, "duration": 50, "parents": [6]},
    8: {"ticket_id": 8, "machine_type": 2, "actual_duration": 42.5, "duration": 48, "parents": [7]},
    9: {"ticket_id": 9, "machine_type": 3, "actual_duration": 26.9, "duration": 35, "parents": [8]},
    10: {"ticket_id": 10, "machine_type": 4, "actual_duration": 34, "duration": 40.4, "parents": [9]},
    11: {"ticket_id": 11, "machine_type": 7, "actual_duration": 24.6, "duration": 40, "parents": [10]},

    12: {"ticket_id": 12, "machine_type": 0, "actual_duration": 5, "duration": 5, "parents": [], "num_robots": 2},
    13: {"ticket_id": 13, "machine_type": 1, "actual_duration": 205.3, "duration": 180, "parents": [12]},
    14: {"ticket_id": 14, "machine_type": 2, "actual_duration": 67.8, "duration": 78, "parents": [13]},
    15: {"ticket_id": 15, "machine_type": 3, "actual_duration": 57.5, "duration": 55, "parents": [14]},
}
