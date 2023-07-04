#!/usr/bin/env python3
import numpy as np
from scipy.optimize import least_squares # nonlinear least-squares

def tag_pair_min_z(anchor_mat_front, anchor_mat_back,
	dists_meas_front, dists_meas_back, tag_loc_front, tag_loc_back, z=0.0):
	# Find the position of the robot given distance readings from 2 UWB tags

	# The two tags can have different numbers of readings
	# (Since the UWB tags may report 3 or 4 readings in general)

	# If there are two solutions (e.g. when anchors are in a plane)
	# then this function gives the solution with the smaller Z value

	# Inputs:
	#	anchor_mat_front:
	#		each col is XYZ position of anchor measured with tag 1
	#	anchor_mat_back:
	#		each col is XYZ position of anchor measured with tag 2
	#	dists_meas_front: vector of measured distances from tag 1
	#	dists_meas_back: vector of measured distances from tag 2
	#	tag_loc_front: XY position of the 1st anchor in the robot frame
	#	tag_loc_back: XY position of the 2nd anchor in the robot frame
	#	z: Height of the tags on the robot from floor (optional, if not specified z will be estimated as well)

	# Outputs: 
	# 	robot_pos: X;Y;Z;theta position of the robot (in world frame)
	#	rmse: Root-mean-square multilateration error
	#		sqrt(mean(distance_error.^2))

	# Find the first guess w.r.t. the mean of the tag positions
	# (later we will translate back to the robot frame)
	tag_loc_mean = (tag_loc_front + tag_loc_back)/2.
	P_mean_front = tag_loc_front - tag_loc_mean
	P_mean_back = tag_loc_back - tag_loc_mean

	# Ignoring the tag-tag distance constraint,
	# find a first guess for each tag position
	tag_guess_front = linear_multilateration_min_z(
		dists_meas_front, anchor_mat_front)
	tag_guess_back = linear_multilateration_min_z(
		dists_meas_back, anchor_mat_back)

	# Initial guess for actual mean position is average of tags
	xyz_mean_guess = (tag_guess_front + tag_guess_back)/2.

	# Intial guess for robot angle is average of angles from
	# xyz_mean_guess to tag_gess

	# Known angle of each tag w.r.t. mean
	tag_mean_angle_front = np.arctan2(P_mean_front[1], P_mean_front[0])
	tag_mean_angle_back = np.arctan2(P_mean_back[1], P_mean_back[0])

	# Angle of each tag w.r.t guess of mean position
	tag_world_angle_front = np.arctan2(
		tag_guess_front[1,:]-xyz_mean_guess[1,:],
		tag_guess_front[0,:]-xyz_mean_guess[0,:])
	tag_world_angle_back  = np.arctan2(
		tag_guess_back[1,:]-xyz_mean_guess[1,:],
		tag_guess_back[0,:]-xyz_mean_guess[0,:])
	
	# Now find the relative angles and take the mean
	angle_guess = (
		wrapToPi(-tag_mean_angle_front+tag_world_angle_front) + 
		wrapToPi(-tag_mean_angle_back+tag_world_angle_back)) / 2.0
	# angle_guess is the amount of rotation needed from world x axis to robot x axis wrapped btw. -pi to +pi

	# Go from the mean frame back to the robot frame
	xyz_guess = xyz_mean_guess - rot_mat(angle_guess).dot(np.block([[tag_loc_mean],[z]]))
	xyzt_guess = np.block([[xyz_guess],[angle_guess]])

	# Using this close first guess, apply nonlinear minimization
	opt_result = least_squares(tag_pair_err_fun, 
			    			   xyzt_guess.flatten(),
							   args=(anchor_mat_front, anchor_mat_back,
									 dists_meas_front, dists_meas_back, 
									 np.block([[tag_loc_front],[z]]), 
									 np.block([[tag_loc_back],[z]])))

	robot_pos = opt_result.x
	robot_pos[3] = wrapToPi(robot_pos[3])
	
	rmse = np.sqrt(np.mean(opt_result.fun**2.))
	return robot_pos, rmse

def tag_pair_err_fun(robot_pos, 
					 anchor_mat_1, anchor_mat_2,
					 dists_meas_1, dists_meas_2, 
					 p_1, p_2):
	
	# The error output is minimized to find the best robot position
	# Error is the diffrence between measured and actual UWB distances

	# Here we assume the robot and tags all have the same Z position

	# This function needs to work when there are a different number of
	# measurements for each tag (each UWB tag may have 3 or 4 measurements)

	# Inputs:
	# 	robot_pos: X;Y;Z;theta position of the robot (in world frame)
	#	anchor_mat_1: each col is XYZ position of anchor measured with tag 1
	#	anchor_mat_2: each col is XYZ position of anchor measured with tag 2
	#	dists_meas_1: vector of measured distances from tag 1
	#	dists_meas_2: vector of measured distances from tag 2
	#	p_1: XYZ position of the 1st anchor in the robot frame
	#	p_2: XYZ position of the 2nd anchor in the robot frame
	# Outputs:
	# 	err: [dists_meas_1; dists_meas_2]  - (distances according to inputs)

	robot_pos = robot_pos.flatten() # scipy compatibility
	# (Scipi already robot_pos flattened,
	# but flatten it in case we're calling from numpy)

	# From the robot position, calcualate tag positions
	tag_1 = robot_pos[0:2+1, np.newaxis]\
		+ rot_mat(robot_pos[3]).dot(p_1)
	tag_2 =	robot_pos[0:2+1, np.newaxis]\
		+ rot_mat(robot_pos[3]).dot(p_2)

	N_anchors_1 = anchor_mat_1.shape[1]
	N_anchors_2 = anchor_mat_2.shape[1]

	dists = np.zeros((N_anchors_1+N_anchors_2, 1))

	for i in range(N_anchors_1):
		dists[i] = np.linalg.norm(tag_1 - anchor_mat_1[:, [i]])
	for i in range(N_anchors_2):
		dists[N_anchors_1+i] = np.linalg.norm(tag_2 - anchor_mat_2[:, [i]])

	err_vec = np.block([[dists_meas_1],[dists_meas_2]]) - dists
	return err_vec.flatten() # scipy compatibility

def linear_multilateration_min_z(distance_mat, anchors):
	# Without doing any nonlinear optimization, find the XYZ position of tags
	# given the position of anchors and the distances between tags and anchors

	# When the anchor matrix is low rank
	# (i.e. anchors are planar or nearly planar)
	# there are two solutions
	# (since we could mirror the tag on the other side of the plane).
	# This function chooses solutions with the minimum Z value

	# Inputs:
	#	distance_mat: each col is distance measurements from one tag
	#	anchors: each col is XYZ position of anchor
	# Outputs:
	#	tag_mat: Each col is XYZ position of the tag

	N_anchors = anchors.shape[1]
	N_tags = distance_mat.shape[1]

	A = np.block([np.ones((N_anchors, 1)), -2*anchors.T])
	b = distance_mat**2-(np.linalg.norm(anchors, axis=0, keepdims=True)**2).T

	# Particular soln to unconstrained problem
	# One x_p per tag, so this is a matrix
	x_p_mat = np.linalg.lstsq(A,b, rcond=None)[0]

	# Homogenous soln to unconstrained problem
	# Find the smallest right singular vector of A
	# (The null vector, or the closest null vector)
	vh = np.linalg.svd(A)[2]
	x_h = vh[[-1],:].T

	linear_solns = np.zeros((3, N_tags))
	for i in range(N_tags):
		x_p = x_p_mat[:,[i]]

		quad_a = (x_h[1:3+1]**2).sum()
		quad_b = 2 * (x_p[1:3+1]*x_h[1:3+1]).sum() - x_h[0]
		quad_c = (x_p[1:3+1]**2).sum() - x_p[0]

		quad = np.block([quad_c, quad_b, quad_a])
		t_12 = np.polynomial.polynomial.polyroots(quad)
		t_12 = np.real(t_12)

		soln_0 = x_p + t_12[0]*x_h
		soln_1 = x_p + t_12[1]*x_h
		
		# Pick the solution with the smaller Z value
		if soln_0[3][0] < soln_1[3][0]:
			linear_solns[:,[i]] = soln_0[1:3+1]
		else:
			linear_solns[:,[i]] = soln_1[1:3+1]
	return linear_solns

def rot_mat(theta):
	c, s = np.cos(theta), np.sin(theta)
	return np.array([[c, -s, 0], [s, c, 0], [0,0,1]])

def wrapToPi(a):
	'''
	Wraps angle to [-pi,pi)
	'''
	return ((a+np.pi) % (2*np.pi))-np.pi

# Test functions ---------------------------------
def test_linear_multilateration_min_z():
	anchors = np.array([[0, 0, 10, 10],
						[0, 10, 10, 0],
						[10, 5, 10, 10]])
	dists_1 = np.array([[7.0711],[6.7082],[10.4881],[9.4868]])
	dists_2 = np.array([[9.434],[7.3485],[5.3852],[8.3066]])
	dists = np.block([dists_1, dists_2])
	p = linear_multilateration_min_z(dists, anchors)
	print(p)
	# Should be [3;4;5] for first tag, [6;7;8] for second tag

def test_tag_pair_err_fun():
	robot_pos = np.array([[1],[2],[3],[4]])
	anchor_mat = np.array([[0, 0, 10, 10],
						[0, 10, 10, 0],
						[10, 5, 10, 10]])
	dists_meas = np.array([[7.0711],[6.7082],[10.4881],[9.4868]])
	p_1 = np.array([[1],[0]])
	p_2 = np.array([[-1],[0]])
	err = tag_pair_err_fun(robot_pos, anchor_mat,anchor_mat,
		dists_meas,dists_meas, p_1, p_2)
	print(err)

def test_tag_pair_min_z():
	anchor_mat = np.array([[0, 10, 0, 10],
						   [0, 0, 10, 10],
						   [10, 10, 10, 10]])
	anchor_mat_front = anchor_mat
	anchor_mat_back = anchor_mat
	dists_meas_front = np.array([[10.7703],[11.6619],[9.7980],[10.7703]])
	dists_meas_back = np.array([[10.7703],[9.7980],[11.6619],[10.7703]])
	tag_loc_front = np.array([[-1],[1]])
	tag_loc_back = np.array([[1],[-1]])

	#robot_pos = np.array([[5],[5],[2],[0]])
	#print(tag_pair_err_fun(robot_pos, anchor_mat_front, anchor_mat_back,
	#	dists_meas_front, dists_meas_back, tag_loc_front, tag_loc_back))

	for i in range(10):
		pos,rmse = tag_pair_min_z(anchor_mat_front, anchor_mat_back,
		dists_meas_front, dists_meas_back, tag_loc_front, tag_loc_back)
		print(pos)
		print(rmse)
	# should be [5 5 2 0]

def test_tag_pair_constant_z():
	anchor_mat = np.array([[0, 10, 0, 10],
						   [0, 0, 10, 10],
						   [10, 10, 10, 10]])
	anchor_mat_front = anchor_mat
	anchor_mat_back = anchor_mat
	dists_meas_front = np.array([[10.7703],[11.6619],[9.7980],[10.7703]])
	dists_meas_back = np.array([[10.7703],[9.7980],[11.6619],[10.7703]])
	tag_loc_front = np.array([[-1],[1]])
	tag_loc_back = np.array([[1],[-1]])

	#robot_pos = np.array([[5],[5],[2],[0]])
	#print(tag_pair_err_fun(robot_pos, anchor_mat_front, anchor_mat_back,
	#	dists_meas_front, dists_meas_back, tag_loc_front, tag_loc_back))

	for i in range(10):
		pos,rmse = tag_pair_min_z(anchor_mat_front, anchor_mat_back,
		dists_meas_front, dists_meas_back, tag_loc_front, tag_loc_back, z=2.1)
		print(pos)
		print(rmse)
	# should be [5 5 2 0]

if __name__ == '__main__':
	# test_tag_pair_min_z()
	test_tag_pair_constant_z()
# Test functions ---------------------------------