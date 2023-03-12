'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Contains the node numbers of stations in the graph.
'''
from utils.grid_utils import create_grid

# Station and base nodes in the graph. Each node is an integer.
Base = 1
Sewing_Machine_1 = 2
Sewing_Machine_2 = 3
Sewing_Machine_3 = 4
Sewing_Machine_4 = 5
Mega_Stitch = 6
RF_Welder = 7
Long_Arm = 8
Grommet = 9
Cutter = 10

# Station and base locations in grid workspace.
bounds = [0, 120, 0, 40]
spaceStep = 5
timeStep = 10
grid = create_grid(bounds, spaceStep)


loc_names = [
    "Base",
    "Sewing_Machine_1",
    "Sewing_Machine_2",
    "Sewing_Machine_3",
    "Sewing_Machine_4",
    "Mega_Stitch",
    "RF_Welder",
    "Long_Arm",
    "Grommet",
    "Cutter"
]
grid_locations = {
    "Base": [23],
    "Sewing_Machine_1": [9],
    "Sewing_Machine_2": [12],
    "Sewing_Machine_3": [15],
    "Sewing_Machine_4": [18],
    "Mega_Stitch": [27, 28, 51, 52],
    "RF_Welder": [176, 177],
    "Long_Arm": [184],
    "Grommet": [191],
    "Cutter": [168]
}


workspace = list(range(1, len(loc_names)+1))
numStates = len(workspace)

obs = []
stations = [j for i in range(len(loc_names)) for j in grid_locations[loc_names[i]]]

if __name__ == '__main__':
    print(f"Defined nodes: {workspace}")
    print(f"Number of nodes: {numStates}")