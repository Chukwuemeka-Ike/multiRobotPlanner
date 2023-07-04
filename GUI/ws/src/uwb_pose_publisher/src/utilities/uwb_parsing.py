#!/usr/bin/env python3
import numpy as np


IND_NUMBER_READINGS = 1;

IND_ID_0 = 3;
IND_X_0 = 4; IND_Y_0 = 5; IND_Z_0 = 6; IND_DIST_0 = 7;

IND_ID_1 = 9;
IND_X_1 = 10; IND_Y_1 = 11; IND_Z_1 = 12; IND_DIST_1 = 13;

IND_ID_2 = 15;
IND_X_2 = 16; IND_Y_2 = 17; IND_Z_2 = 18; IND_DIST_2 = 19;

IND_ID_3 = 21;
IND_X_3 = 22; IND_Y_3 = 23; IND_Z_3 = 24; IND_DIST_3 = 25;

def parse_lec_line(lec_string):
	'''
	Parse the LEC string and convert into numpy variables

	We ignore:
		ID of anchor
		tag's own localization
		quality factors reported for each distance
		quality factor reported for tag's own localization

	Input:
		lec_string: string from UWB tag sent over serial (using 'lec' command)
	
	Outputs:
		valid: True if a valid string, else false (very quick testing done)
		anchor_mat: Each column is XYZ of the UWB anchor
		dists: Vector of distances to each anchor

	'''
	spl_str = lec_string.strip().split(",")

	# Make sure we're dealing with a valid line
	if spl_str[0] != "DIST":
		return False, 0., 0., 0.

	# Two cases: 3 readings or 4 readings
	if spl_str[IND_NUMBER_READINGS] == '4':
		anchor_mat = np.array(
			[[float(spl_str[IND_X_0]), float(spl_str[IND_X_1]) , float(spl_str[IND_X_2]) , float(spl_str[IND_X_3])],
			 [float(spl_str[IND_Y_0]), float(spl_str[IND_Y_1]) , float(spl_str[IND_Y_2]) , float(spl_str[IND_Y_3])],
			 [float(spl_str[IND_Z_0]), float(spl_str[IND_Z_1]) , float(spl_str[IND_Z_2]) , float(spl_str[IND_Z_3])]])
		dists = np.array([[float(spl_str[IND_DIST_0])], [float(spl_str[IND_DIST_1])], [float(spl_str[IND_DIST_2])], [float(spl_str[IND_DIST_3])]])
		
		indices = [IND_ID_0, IND_ID_1, IND_ID_2, IND_ID_3]
		ids = [spl_str[ind] for ind in indices]

	elif spl_str[IND_NUMBER_READINGS] == '3': # Three readings
		anchor_mat = np.array(
			[[float(spl_str[IND_X_0]), float(spl_str[IND_X_1]) , float(spl_str[IND_X_2])],
			 [float(spl_str[IND_Y_0]), float(spl_str[IND_Y_1]) , float(spl_str[IND_Y_2])],
			 [float(spl_str[IND_Z_0]), float(spl_str[IND_Z_1]) , float(spl_str[IND_Z_2])]])
		dists = np.array([[float(spl_str[IND_DIST_0])], [float(spl_str[IND_DIST_1])], [float(spl_str[IND_DIST_2])]])
		
		indices = [IND_ID_0, IND_ID_1, IND_ID_2]
		ids = [spl_str[ind] for ind in indices]

	else:
		return False, 0., 0., 0.

	return True, anchor_mat, dists, ids
	

def test_uwb_parsing():
	uwb_string_1 = "DIST,4,AN0,2F2F,3.05,2.68,0.00,2.21,AN1,2C9D,-0.04,2.91,0.00,2.39,AN2,2ED0,3.02,0.00,0.00,2.19,AN3,2BA2,0.00,0.00,0.00,2.56,POS,1.59,1.65,1.27,44"
	#               0    1 2   3    4    5    6    7    8   9    10    11   12   13   14  15   16   17   18   19   20  21   22    23   24  25   26  27   28   29   30

	uwb_string_2 = "DIST,3,AN0,2F2F,3.05,2.68,0.00,2.20,AN1,2ED0,3.02,0.00,0.00,2.23,AN2,2BA2,0.00,0.00,0.00,3.13,POS,1.76,1.59,0.51,40"
	#               0    1 2   3    4    5    6    7    8   9    10   11   12   13   14  15   16   17   18   19   20  21   22   23   24
	uwb_string_3 = "dwm> "
	
	print("String 1:")
	valid, anchor_mat, dists, ids = parse_lec_line(uwb_string_1)
	print(valid)
	print(anchor_mat)
	print(dists)
	print(ids)

	print("\nString 2:")
	valid, anchor_mat, dists, ids = parse_lec_line(uwb_string_2)
	print(valid)
	print(anchor_mat)
	print(dists)
	print(ids)

	print("\nString 3:")
	valid, anchor_mat, dists, ids = parse_lec_line(uwb_string_3)
	print(valid)
	print(anchor_mat)
	print(dists)
	print(ids)

if __name__ == '__main__':
	test_uwb_parsing()