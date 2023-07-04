#!/usr/bin/env python3

import numpy as np
import rospy

from geometry_msgs.msg import Twist, Pose2D

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg

from swarm_msgs.msg import State2D # Custom message

import tf_conversions # quaternion stuff

from velocity_control_law import *

'''
closed_loop_velocity_controller.py
Alex Elias, Burak Aksoy

Drives a mobile robot to a desired position/velocity

Parameters:
    feedback_gain_xy: Proportional gain in (m/s) / m
    feedback_gain_theta: Proportional gain in (rad/s) / rad

    cmd_input_type: 'State2D'
        State2D ==> Full state control (e.g. swarm control)
    cmd_input_topic_name: Desired input
        
    position_feedback_topic_name: Topic for position estimation (e.g. from sensor fusion)
    control_cmd_publish_topic_name: Topic for motor control input

    is_skid_steer_mode: True means that you are controlling only x and y, not theta

    vel_lim_x: Velocity limit in X
    vel_lim_y: Velocity limit in Y
    vel_lim_theta: Velocity limit in theta
'''

class Controller:
    def __init__(self):
        rospy.init_node('closed_loop_velocity_controller', anonymous=True)
        
        cmd_input_topic_name = rospy.get_param('~cmd_input_topic_name')
        control_cmd_publish_topic_name = rospy.get_param('~control_cmd_publish_topic_name')
        
        self.is_skid_steer_mode = rospy.get_param('~is_skid_steer_mode', False)
        
        if self.is_skid_steer_mode:
            position_feedback_topic_name = rospy.get_param('~turntable_position_topic_name')
        else:
            position_feedback_topic_name = rospy.get_param('~position_feedback_topic_name')


        if self.is_skid_steer_mode is False:
            vel_lim_x = rospy.get_param('~vel_lim_x')
            vel_lim_y = rospy.get_param('~vel_lim_y')
            vel_lim_theta = rospy.get_param('~vel_lim_theta')
            self.vel_limit = np.array([[vel_lim_x],[vel_lim_y],[vel_lim_theta]])
            self.feedback_gain_theta = rospy.get_param('~feedback_gain_theta')

        self.feedback_gain_xy = rospy.get_param('~feedback_gain_xy')
        
        self.vel_cmd_pub = rospy.Publisher(control_cmd_publish_topic_name, Twist, queue_size=1)
        self.state_pos = np.array([[0.0],[0.0],[0.0]])
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        rospy.Subscriber(cmd_input_topic_name, State2D, self.desired_state_callback, queue_size=1)

        # Subscribe to Filtered position
        rospy.Subscriber(position_feedback_topic_name, nav_msgs.msg.Odometry, self.state_feedback_callback, queue_size=1)


    def desired_state_callback(self, data):
        desired_state = np.zeros((6,1)) 
        desired_state[0] = data.pose.x
        desired_state[1] = data.pose.y
        desired_state[2] = data.pose.theta
        desired_state[3] = data.twist.linear.x
        desired_state[4] = data.twist.linear.y
        desired_state[5] = data.twist.angular.z
        self.process_desired_state(desired_state)

    def process_desired_state(self, desired_state):
        if self.is_skid_steer_mode:
            K = self.feedback_gain_xy
            cmd_vel = control_law_skid_steer_mode(desired_state, self.state_pos,K)
        else:
            K = np.diag([self.feedback_gain_xy, self.feedback_gain_xy, self.feedback_gain_theta])
            cmd_vel = control_law(desired_state, self.state_pos, self.vel_limit, K)

        # Publish commanded velocity
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = cmd_vel[0][0]
        cmd_vel_msg.linear.y = cmd_vel[1][0]
        cmd_vel_msg.angular.z = cmd_vel[2][0]
        self.vel_cmd_pub.publish(cmd_vel_msg)


    def state_feedback_callback(self, data):
        orientations = [data.pose.pose.orientation.x, 
                        data.pose.pose.orientation.y,
                        data.pose.pose.orientation.z,
                        data.pose.pose.orientation.w]
        (roll,pitch,yaw) = tf_conversions.transformations.euler_from_quaternion(orientations)
        
        self.state_pos[0][0] = data.pose.pose.position.x # x
        self.state_pos[1][0] = data.pose.pose.position.y # y
        self.state_pos[2][0] = yaw # theta



if __name__ == '__main__':
    Controller()
    rospy.spin()