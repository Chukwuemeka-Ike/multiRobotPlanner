'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Station information.

    Currently assumes we have multiple machines for some operation types:
        0. Loading - 2
        1. Mega - 2
        2. Vinyl - 2
        3. Hem - 4
        4. Tack - 2
        5. Weld - 4
        6. Grommet - 4
        7. Inspection - 1
'''
station_type_names = [
    "Loading",
    "Mega",
    "Vinyl",
    "Hem",
    "Tack",
    "Weld",
    "Grommet",
    "Inspection",
]
station_type_abvs = ["L", "MS", "V", "H", "T", "RF", "G", "I"]
num_stations = [2, 2, 2, 4, 2, 4, 4, 1]
# num_stations = [1, 1, 1, 1, 1, 1, 1, 1]

# Station type numbers are used for specifying tasks in a job. 
# Num stations is how many of each type there are.
station_type_numbers = list(range(len(station_type_names)))

# Create increasing station numbers based on how many
# there are of each type. Output looks similar to description.
station_num = 0
Mj, all_machines = [], []
for i in range(len(station_type_numbers)):
    stations = []
    for j in range(1, num_stations[i]+1):
        stations.append(station_num)
        all_machines.append(station_num)
        station_num += 1
    Mj.append(stations)
# print(f"All machines: {all_machines}")


# *******************************************************************
# Variables for high level visualization. This is the old set.
station_type_ws_nums = {
    "Loading Area": "WS_0_",
    "Mega Stitch": "WS_1_",
    "RF": "WS_2_",
    "Perimeter": "WS_3_",
    "Inspection": "WS_4_",
}

# Idx of station number of its type. For WS_0_# where # is the number of
# that particular station, not the unique ID.
Mjs = []
for i in range(len(Mj)):
    for j in range(len(Mj[i])):
        Mjs.append(j)
# *******************************************************************