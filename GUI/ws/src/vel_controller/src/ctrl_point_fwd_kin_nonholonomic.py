#!/usr/bin/env python3

import numpy as np
import rospy

from geometry_msgs.msg import Pose2D

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

import tf_conversions # quaternion stuff

from velocity_control_law import *

'''
turntable_fwd_kin.py
Alex Elias, Burak Aksoy

Adds constant offset from robot frame to turntable center
Publishes to TF and Pose2D

Parameters:
	p_x, p_y: Position of the turntable center in the robot frame (meters)
		
	position_feedback_topic_name: Topic for position estimation (e.g. from sensor fusion) to subscribe
	turntable_position_topic_name: Topic for pose2D to publish
	tf_turntable_frame_name: Name used to send tf_frame for turntable to publish to tf
	tf_robot_frame_name: Name of robot tf frame published from sensor fusion
'''

class TurntableFwdKin:
	def __init__(self):
		rospy.init_node('turntable_fwd_kin', anonymous=True)

		self.p_x = rospy.get_param('~p_x')
		self.p_y = rospy.get_param('~p_y')

		self.turntable_vec = np.array([[self.p_x, self.p_y, 1.0]]).T # homogeneous position vector of the turntable center in robot frame
	
		position_feedback_topic_name = rospy.get_param('~position_feedback_topic_name')
		turntable_position_topic_name = rospy.get_param('~turntable_position_topic_name')

		self.tf_turntable_frame_name = rospy.get_param('~tf_turntable_frame_name') 
		self.tf_robot_frame_name = rospy.get_param('~tf_robot_frame_name')

		# Publisher
		self.pos_pub = rospy.Publisher(turntable_position_topic_name, Pose2D, queue_size=1)
		self.tf_broadcaster = tf2_ros.TransformBroadcaster()

		# Subscribe to Kalman Filter position
		rospy.Subscriber(position_feedback_topic_name, Pose2D, self.robot_pos_callback, queue_size=1)

	def robot_pos_callback(self, data):
		# Publish for Pose2D
		T_offset = transform_mat(data.theta, data.x, data.y) # Transform matrix from robot to world
		turntable_vec_world = T_offset.dot(self.turntable_vec) # homogenous pos vec
		
		pos_msg = Pose2D()
		pos_msg.x = turntable_vec_world[0][0]
		pos_msg.y = turntable_vec_world[1][0]
		pos_msg.theta = data.theta
		self.pos_pub.publish(pos_msg)

		# Publish for TF (frame)
		tf_turntable = xyt2TF(np.array([self.p_x,self.p_y,-data.theta]), self.tf_robot_frame_name, self.tf_turntable_frame_name)
		self.tf_broadcaster.sendTransform(tf_turntable)


def transform_mat(theta, t_x,t_y):
	c, s = np.cos(theta), np.sin(theta)
	return np.array([[c , -s, t_x],
		             [s ,  c, t_y],
		             [0., 0., 1.]])

def xyt2TF(xyt, header_frame_id, child_frame_id):
	'''
	Converts a numpy vector [x; y; z; theta]
	into a tf2_msgs.msg.TFMessage message
	'''
	xyt = xyt.flatten()

	t = geometry_msgs.msg.TransformStamped()

	t.header.frame_id = header_frame_id
	t.header.stamp = rospy.Time.now()
	t.child_frame_id = child_frame_id
	t.transform.translation.x = xyt[0]
	t.transform.translation.y = xyt[1]
	t.transform.translation.z = 0

	q = tf_conversions.transformations.quaternion_from_euler(0, 0,xyt[2])
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	return t

if __name__ == '__main__':
	TurntableFwdKin()
	rospy.spin()