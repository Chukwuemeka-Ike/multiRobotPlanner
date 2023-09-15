'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Single job set.
'''
complete_ticket_list = {
    0: {"ticket_id": 0, "machine_type": 0, "actual_duration": 5, "duration": 10, "parents": [], "num_robots": 2},
    1: {"ticket_id": 1, "machine_type": 1, "actual_duration": 95.7, "duration": 85, "parents": [0]},
    2: {"ticket_id": 2, "machine_type": 2, "actual_duration": 44.8, "duration": 47.2, "parents": [1]},
    3: {"ticket_id": 3, "machine_type": 3, "actual_duration": 37.6, "duration": 40, "parents": [2]},
    4: {"ticket_id": 4, "machine_type": 4, "actual_duration": 45, "duration": 50, "parents": [3]},
}
