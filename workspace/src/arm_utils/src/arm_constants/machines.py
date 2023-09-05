'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Machine information.

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
machine_type_names = [
    "Loading",
    "Mega",
    "Vinyl",
    "Hem",
    "Tack",
    "Weld",
    "Grommet",
    "Inspection",
]
machine_type_abvs = ["L", "MS", "V", "H", "T", "RF", "G", "I"]
num_machines = [2, 2, 2, 4, 2, 4, 4, 1]
# num_machines = [1, 1, 1, 1, 1, 1, 1, 1]

# Machine type numbers are used for specifying tasks in a job. 
# Num machines is how many of each type there are.
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
# Variables for high level visualization. This is the old set.
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