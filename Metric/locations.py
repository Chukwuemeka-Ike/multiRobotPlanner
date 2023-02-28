'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Contains the locations of obstacles, stations, and the target visit spots.
'''

# Final and initial locations.
initPose = 0
finalPose = 190

# # Obstacle locations.
# obs = [82, 83, 84, 85, 106, 107, 108, 109, 138, 139, 140]
obs = []

# Station locations.
Sewing_Machine_1 = [9]
Sewing_Machine_2 = [12]
Sewing_Machine_3 = [15]
Sewing_Machine_4 = [18]
Mega_Stitch = [27, 28, 51, 52]
RF_Welder = [176, 177]
Long_Arm = [184]
Grommet = [191]

stations = RF_Welder + Grommet + Sewing_Machine_1 + Sewing_Machine_2\
            + Sewing_Machine_3 + Sewing_Machine_4 + Mega_Stitch + Long_Arm

if __name__ == '__main__':
    print(f"Defined station locations: {stations}")