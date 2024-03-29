'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    2-Job set representing the planned physical demo.
'''
complete_ticket_list = {
    # Job 1.
    1: {"ticket_id": 1, "machine_type": 0, "actual_duration": 5, "duration": 8, "parents": [], "num_robots": 3},
    2: {"ticket_id": 2, "machine_type": 1, "actual_duration": 55, "duration": 15, "parents": [1]},
    3: {"ticket_id": 3, "machine_type": 3, "actual_duration": 44.8, "duration": 8, "parents": [2]},
    4: {"ticket_id": 4, "machine_type": 4, "actual_duration": 37.6, "duration": 5, "parents": [3]},

    # Job 2.
    5: {"ticket_id": 5, "machine_type": 0, "actual_duration": 30, "duration": 10, "parents": [], "num_robots": 0},
    6: {"ticket_id": 6, "machine_type": 2, "actual_duration": 96, "duration": 20, "parents": [5]},
    7: {"ticket_id": 7, "machine_type": 3, "actual_duration": 65.9, "duration": 10, "parents": [6]},
    8: {"ticket_id": 8, "machine_type": 4, "actual_duration": 42.5, "duration": 5, "parents": [7]},   
}
