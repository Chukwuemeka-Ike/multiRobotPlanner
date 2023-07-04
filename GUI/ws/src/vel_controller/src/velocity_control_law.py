#!/usr/bin/env python3
import numpy as np

def control_law(desired_state, position, vel_limit, K):
	'''
	Calculates robot velocity to achieve desired state
	u_world = q_dot_desired - K(q - q_desired)
	u_robot = clip{ rot(-theta)u_world }

	Inputs:
		desired_state: [x; y; theta; x_d; y_d; theta_d] the robot should follow
			(In the world frame)
		position: [x; y; theta] of the robot (e.g. as measured by sensor fusion)
		vel_limit: [x_d_max; y_d_max; theta_d_max]
		K: Feedback proportional gain in (m/s) / m or (rad/s) / rad
			(Can be scalar or diagonal)
	Outputs:
		vel_cmd: [x_d; y_d; theta_d] which should be applied to the robot
			(In the robot frame)
	'''
	error = position - desired_state[0:2+1]
	error[2] = wrapToPi(error[2])

	u_world = desired_state[3:5+1] - (K*np.eye(3)).dot(error)

	vel_cmd = rot_mat(-position[2][0]).dot(u_world)


	vel_cmd[vel_cmd >  vel_limit] =  vel_limit[vel_cmd >  vel_limit]
	vel_cmd[vel_cmd < -vel_limit] = -vel_limit[vel_cmd < -vel_limit]

	return vel_cmd

def control_law_skid_steer_mode(desired_state, position,K):
	'''
	Calculates robot velocity to achieve desired state
	u_world = q_dot_desired - K(q - q_desired)
	u_robot = clip{ rot(-theta)u_world }

	Inputs:
		desired_state: [x; y; theta; x_d; y_d; theta_d] the robot should follow
			(In the world frame)
		position: [x; y; theta] of the robot (e.g. as measured by sensor fusion)
		K: Feedback proportional gain in (m/s) / m or (rad/s) / rad
			(Can be scalar or diagonal)
	Outputs:
		vel_cmd: [x_d; y_d; theta_d] which should be applied to the robot
			(In the WORLD frame)
	'''
	error = position - desired_state[0:2+1]

	vel_cmd = desired_state[3:5+1] - (K*np.eye(3)).dot(error)

	return vel_cmd


def rot_mat(theta):
	c, s = np.cos(theta), np.sin(theta)
	return np.array([[c , -s, 0.],
		             [s ,  c, 0.],
		             [0., 0., 1.]])

def wrapToPi(a):
	'''
	Wraps angle to [-pi,pi)
	'''
	return ((a+np.pi) % (2*np.pi))-np.pi

def test_velocity_controller():
	#desired_state = np.array([[10.],[10.],[10.],[10.],[10.],[10.]])
	desired_state = np.array([[-10.],[0.],[0.],[0.],[0.],[0.]])
	
	vel_limit = np.array([[1.],[1.],[1.]])
	#vel_limit = np.array([[1.],[20.],[np.inf]])
	
	position = np.array([[0.],[0.],[np.pi]])
	#print(rot_mat(position[2][0]))

	print(control_law(desired_state, position, vel_limit, 5))

if __name__ == '__main__':
	test_velocity_controller()
