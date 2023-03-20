def display_all_tickets(job_list: list):
    '''Displays all tickets in a job list.'''
    print("All tickets:")
    print("[")
    for job in job_list:
        for task in job:
            print("    ", [f"{k}: {v}" for k, v in task.items()])
    print("]")
    print()

def display_station_numbers(station_names: list, station_numbers: list):
    '''Displays the station numbers of each type.'''
    print("Station numbers:")
    print("[")
    for i in range(len(station_names)):
        print(f"{station_names[i]:>15}:    {station_numbers[i]}")
    print("]")
    print()