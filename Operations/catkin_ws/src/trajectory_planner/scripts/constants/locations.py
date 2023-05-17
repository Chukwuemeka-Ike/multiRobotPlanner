'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Contains the locations of obstacles, stations, and the target visit spots.
'''
import numpy as np


# Station locations.
station_locations = {
    0: [660],
    1: [670],
    2: [452, 453, 402, 403],
    3: [302, 303, 252, 253],
    4: [40],
    5: [45],
    6: [420],
    7: [425],
    8: [430],
    9: [435],
    10: [3],
    11: [103],
    12: [14, 15],
    13: [17, 18],
    14: [20, 21],
    15: [23, 24],
    16: [149],
    17: [249],
    18: [349],
    19: [449],
    20: [j*50 + i for i in range(39, 50) for j in range(11, 14)],
}

stations = [i for k, v in station_locations.items() for i in v]

if __name__ == '__main__':
    print(station_locations)
    print(f"Defined station locations: {stations}")