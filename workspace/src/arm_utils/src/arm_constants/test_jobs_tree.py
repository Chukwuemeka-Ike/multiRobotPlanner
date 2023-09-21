'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Set of 2 tree jobs for testing branching behaviors.
    Requires Anchor machine setup.
'''
complete_ticket_list = {
    # Tree Job with 2 roots.
    11: {"job_id": 1, "machine_type": 0, "duration": 5, "parents": [], "actual_duration": 5, "num_robots": 3},
    12: {"job_id": 1, "machine_type": 0, "duration": 5, "parents": [], "actual_duration": 5, "num_robots": 2},
    13: {"job_id": 1, "machine_type": 2, "duration": 45.7, "parents": [11], "actual_duration": 45.7},
    14: {"job_id": 1, "machine_type": 3, "duration": 54.8, "parents": [12], "actual_duration": 54.8},
    15: {"job_id": 1, "machine_type": 1, "duration": 89.2, "parents": [13,14], "actual_duration": 89.2},
    16: {"job_id": 1, "machine_type": 3, "duration": 43.1, "parents": [15], "actual_duration": 43.1},
    17: {"job_id": 1, "machine_type": 7, "duration": 38.7, "parents": [16], "actual_duration": 38.7},
    # Tree Job with 3 roots.
    1: {"job_id": 0, "machine_type": 0, "duration": 10, "parents": [], "actual_duration": 10, "time_left": 10, "num_robots": 2},
    2: {"job_id": 0, "machine_type": 0, "duration": 5, "parents": [], "actual_duration": 5, "time_left": 5, "num_robots": 1},
    3: {"job_id": 0, "machine_type": 0, "duration": 5, "parents": [], "actual_duration": 7, "time_left": 5, "num_robots": 3},
    4: {"job_id": 0, "machine_type": 2, "duration": 45, "parents": [1], "actual_duration": 55, "time_left": 45},
    5: {"job_id": 0, "machine_type": 2, "duration": 30, "parents": [2], "actual_duration": 25, "time_left": 30},
    6: {"job_id": 0, "machine_type": 3, "duration": 30, "parents": [3], "actual_duration": 28, "time_left": 30},
    7: {"job_id": 0, "machine_type": 2, "duration": 60, "parents": [4,5], "actual_duration": 75, "time_left": 60},
    8: {"job_id": 0, "machine_type": 2, "duration": 30, "parents": [6,7], "actual_duration": 30, "time_left": 30},
    9: {"job_id": 0, "machine_type": 3, "duration": 45, "parents": [8], "actual_duration": 48, "time_left": 45},
    10: {"job_id": 0, "machine_type": 4, "duration": 60, "parents": [9], "actual_duration": 60, "time_left": 60},
}
