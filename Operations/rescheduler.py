'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
'''

rescheduler_initial_jobs = [
    [
        {"ticket_id": 0, "station_type": 0, "duration": 10, "parents": [], "num_robots": 2},
        {"ticket_id": 1, "station_type": 0, "duration": 5, "parents": [], "num_robots": 2},
        {"ticket_id": 2, "station_type": 0, "duration": 5, "parents": [], "num_robots": 2},
        {"ticket_id": 3, "station_type": 2, "duration": 45, "parents": [0], "num_robots": 2},
        {"ticket_id": 4, "station_type": 2, "duration": 30, "parents": [1], "num_robots": 2},
        {"ticket_id": 5, "station_type": 3, "duration": 30, "parents": [2], "num_robots": 2},
        {"ticket_id": 6, "station_type": 2, "duration": 60, "parents": [3, 4], "num_robots": 4},
        {"ticket_id": 7, "station_type": 2, "duration": 30, "parents": [5,6], "num_robots": 6},
        {"ticket_id": 8, "station_type": 3, "duration": 45, "parents": [7], "num_robots": 6},
        {"ticket_id": 9, "station_type": 4, "duration": 60, "parents": [8], "num_robots": 6},
        # {"ticket_id": , "station_type": , "duration": , "parents": [], "num_robots": },
    ]
]