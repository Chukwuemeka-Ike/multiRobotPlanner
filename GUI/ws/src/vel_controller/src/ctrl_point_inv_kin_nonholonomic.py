#!/usr/bin/env python3

import numpy as np
import rospy

from geometry_msgs.msg import Twist, Pose2D

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

import tf_conversions # quaternion stuff

from velocity_control_law import *

'''
turntable_inv_kin.py
Alex Elias, Burak Aksoy

Calculates the desired skid steer robot velocities (v, w) for a desired velocity of the turntable in world frame (x_dot y_dot)
Uses the constant offset from robot frame to turntable center

Parameters:
	p_x, p_y: Position of the turntable center in the robot frame (meters)
		
	position_feedback_topic_name: Topic for position estimation (e.g. from sensor fusion) to subscribe
	robot_cmd_vel_topic_name: Twist command to robot
	turntable_cmd_vel_topic_name: Desired command for the turntable velocities (x_dot y_dot)

	vel_lim_x: Robot linear velocity limit
	vel_lim_theta: Robot angular velocity limit
'''

class TurntableInvKin:
	def __init__(self):
		rospy.init_node('turntable_inv_kin', anonymous=True)

		self.p_x = rospy.get_param('~p_x')
		self.p_y = rospy.get_param('~p_y')

		self.theta = None
	
		position_feedback_topic_name = rospy.get_param('~position_feedback_topic_name')
		robot_cmd_vel_topic_name = rospy.get_param('~robot_cmd_vel_topic_name')
		turntable_cmd_vel_topic_name =  rospy.get_param('~turntable_cmd_vel_topic_name')

		self.v_lim =  rospy.get_param('~vel_lim_x')
		self.w_lim =  rospy.get_param('~vel_lim_theta')

		# Publisher
		self.cmd_pub = rospy.Publisher(robot_cmd_vel_topic_name, Twist, queue_size=1)

		# Subscribers
		rospy.Subscriber(turntable_cmd_vel_topic_name, Twist, self.turntable_cmd_vel_callback, queue_size=1)
		# Subscribe to Kalman Filter position
		rospy.Subscriber(position_feedback_topic_name, Pose2D, self.robot_pos_callback, queue_size=1)

	def robot_pos_callback(self, data):
		self.theta = data.theta

	def turntable_cmd_vel_callback(self,data):
		if self.theta is not None:
			x_dot = data.linear.x
			y_dot = data.linear.y

			c, s = np.cos(self.theta), np.sin(self.theta)

			v = ( x_dot * (self.p_x * c - self.p_y * s)  +  y_dot*(self.p_x * s + self.p_y * c) ) / self.p_x
			w = ( x_dot * (-s)  +  y_dot*(c) ) / self.p_x

			msg = Twist()
			msg.linear.x = constrain(v,-self.v_lim,self.v_lim)
			msg.angular.z = constrain(w,-self.w_lim,self.w_lim)
			self.cmd_pub.publish(msg)
		else:
			rospy.logwarn("Don't know theta yet")

def constrain(val,min_val,max_val):
	return min(max_val,max(min_val,val))

if __name__ == '__main__':
	TurntableInvKin()
	rospy.spin()