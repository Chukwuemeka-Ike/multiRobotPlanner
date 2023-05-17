complete_ticket_list = {
    # Tree Job 3 roots.
    1: {"station_type": 0, "duration": 10, "parents": [], "actual_duration": 10, "time_left": 10},
    2: {"station_type": 0, "duration": 5, "parents": [], "actual_duration": 5, "time_left": 5},
    3: {"station_type": 0, "duration": 5, "parents": [], "actual_duration": 7, "time_left": 5},
    4: {"station_type": 2, "duration": 45, "parents": [1], "actual_duration": 55, "time_left": 45},
    5: {"station_type": 2, "duration": 30, "parents": [2], "actual_duration": 25, "time_left": 30},
    6: {"station_type": 3, "duration": 30, "parents": [3], "actual_duration": 28, "time_left": 30},
    7: {"station_type": 2, "duration": 60, "parents": [4,5], "actual_duration": 75, "time_left": 60},
    8: {"station_type": 2, "duration": 30, "parents": [6,7], "actual_duration": 30, "time_left": 30},
    9: {"station_type": 3, "duration": 45, "parents": [8], "actual_duration": 48, "time_left": 45},
    10: {"station_type": 4, "duration": 60, "parents": [9], "actual_duration": 60, "time_left": 60},
}