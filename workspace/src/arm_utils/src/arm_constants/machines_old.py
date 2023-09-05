'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Constants useful across scripts.

    Currently assumes we have multiple machines for some operation types:
        Loading Area - 0
        Mega Stitch - [1, 2]
        RF - 3
        Perimeter - [4, 5, 6]
        Inspection - 7
'''
machine_type_names = [
    "Loading Area",
    "Mega Stitch",
    "RF",
    "Perimeter",
    "Inspection"
]
machine_type_abvs = ["L", "MS", "RF", "P", "I"]
num_machines = [1, 2, 1, 3, 1]

# Machine type numbers are used for specifying tasks in a job. 
# Num Machines is how many of each type there are.
machine_type_numbers = list(range(len(machine_type_names)))

# Create increasing machine numbers based on how many
# there are of each type. Output looks similar to description.
machine_id = 0
grouped_machine_ids, machine_ids = [], []
for i in range(len(machine_type_numbers)):
    machines = []
    for j in range(1, num_machines[i]+1):
        machines.append(machine_id)
        machine_ids.append(machine_id)
        machine_id += 1
    grouped_machine_ids.append(machines)
# print(f"All machines: {machine_ids}")


# *******************************************************************
# Variables for high level visualization.
machine_type_ws_nums = {
    "Loading Area": "WS_0_",
    "Mega Stitch": "WS_1_",
    "RF": "WS_2_",
    "Perimeter": "WS_3_",
    "Inspection": "WS_4_",
}

# Idx of machine number of its type. For WS_0_# where # is the number of
# that particular machine, not the unique ID.
machine_type_indices = []
for i in range(len(grouped_machine_ids)):
    for j in range(len(grouped_machine_ids[i])):
        machine_type_indices.append(j)
# *******************************************************************