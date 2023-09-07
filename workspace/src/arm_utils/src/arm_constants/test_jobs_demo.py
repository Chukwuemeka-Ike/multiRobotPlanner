complete_ticket_list = {
    # Job 1.
    1: {"ticket_id": 1, "machine_type": 0, "actual_duration": 5, "duration": 10, "parents": [], "num_robots": 3},
    2: {"ticket_id": 2, "machine_type": 1, "actual_duration": 95.7, "duration": 85, "parents": [1]},
    3: {"ticket_id": 3, "machine_type": 3, "actual_duration": 44.8, "duration": 47.2, "parents": [2]},
    4: {"ticket_id": 4, "machine_type": 4, "actual_duration": 37.6, "duration": 40, "parents": [3]},

    # Job 2.
    5: {"ticket_id": 5, "machine_type": 0, "actual_duration": 45, "duration": 50, "parents": []},
    6: {"ticket_id": 6, "machine_type": 2, "actual_duration": 40.6, "duration": 44.2, "parents": [5]},
    7: {"ticket_id": 7, "machine_type": 3, "actual_duration": 65.9, "duration": 50, "parents": [6]},
    8: {"ticket_id": 8, "machine_type": 4, "actual_duration": 42.5, "duration": 48, "parents": [7]},
    
}

test_data_1_info = {
    'num_ticket_adds': 4
}
test_data_1 = {
    'complete_ticket_list': 
    complete_ticket_list.copy(),
}
test_data_1["ticket_add_0"] = {  
    ticket_id: test_data_1["complete_ticket_list"][ticket_id] for ticket_id in range(
        min(complete_ticket_list.keys()), 16
    )
}