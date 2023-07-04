#!/usr/bin/env python3  

"""
Author: Burak Aksoy
Node: uwb_pose_publisher
Description:
    - TODO
    
Parameters:
    - TODO
Subscribes to:
    - TODO
Publishes to:
    - TODO

Broadcasts to:
    - NONE
"""

import rospy

import numpy as np
import math

import std_msgs.msg 
import geometry_msgs.msg

# Local python files
from utilities.multilateration import * # tag_pair_min_z(...)
from utilities.uwb_parsing import * # parse_lec_line(...)

DEBUG_UWB = False

class UWBPosePublisher():
    def __init__(self):
        rospy.init_node('uwb_pose_publisher', anonymous=True)

        # Parameters
        self.tf_world_frame = rospy.get_param("~tf_world_frame", "map")

        self.pub_pose_topic_name = rospy.get_param("~pub_pose_topic_name", "pose_uwb")

        self.sub_uwb_tag_1_topic_name = rospy.get_param('~sub_uwb_tag_1_topic_name', "uwb/tag_1_serial_ranging")
        self.sub_uwb_tag_2_topic_name = rospy.get_param('~sub_uwb_tag_2_topic_name', "uwb/tag_2_serial_ranging")

        uwb_tag_1_id = rospy.get_param('~uwb_tag_1_id')
        uwb_tag_2_id = rospy.get_param('~uwb_tag_2_id')

        self.antenna_offsets = rospy.get_param('~antenna_offsets')

        self.uwb_tag_1_offset = self.antenna_offsets[uwb_tag_1_id]
        self.uwb_tag_2_offset = self.antenna_offsets[uwb_tag_2_id]

        tag_1_x = rospy.get_param('~tag_1_x')
        tag_1_y = rospy.get_param('~tag_1_y')
        self.tag_1_loc = np.array([[tag_1_x],[tag_1_y]])

        tag_2_x = rospy.get_param('~tag_2_x')
        tag_2_y = rospy.get_param('~tag_2_y')
        self.tag_2_loc = np.array([[tag_2_x],[tag_2_y]])

        self.tag_z_height = rospy.get_param('~tag_z_height',0.0)

        self.uwb_meas_std = rospy.get_param('~uwb_meas_std', [0.5, 0.5, 0.2]) # comes from uwb sensor readings data # [x,y,rotz]


        self.covariance_diagonal_max = [std*std for std in self.uwb_meas_std] # least reliable
        self.covariance_diagonal_min = [std*std for std in self.uwb_meas_std] # most_reliable

        # Publishers
        self.pub_pose = rospy.Publisher(self.pub_pose_topic_name, geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
        self.pub_rmse = rospy.Publisher(self.pub_pose_topic_name + '_RMSE', std_msgs.msg.Float32, queue_size=10)

        # Subscribe to UWB tags
        rospy.Subscriber(self.sub_uwb_tag_1_topic_name, std_msgs.msg.String, self.uwb_serial_tag_1_callback, queue_size=5)
        rospy.Subscriber(self.sub_uwb_tag_2_topic_name, std_msgs.msg.String, self.uwb_serial_tag_2_callback, queue_size=5)

        # Class variables:
        # time stamps of last uwb readings
        self.last_uwb_tag_1_time = 0.0
        self.last_uwb_tag_2_time = 0.0

        # Matrix of anchor positions
        self.uwb_tag_1_anchors = 0.
        self.uwb_tag_2_anchors = 0.

        # Vector of distances to each anchor
        self.uwb_tag_1_anchor_dists = 0.
        self.uwb_tag_2_anchor_dists = 0.

        # IDs corresponding to anchors
        self.uwb_tag_1_anchor_ids = 0.
        self.uwb_tag_2_anchor_ids = 0.

        self.uwb_tag_1_valid = False
        self.uwb_tag_2_valid = False

        # Publish rate
        self.expected_uwb_rate = rospy.get_param("~expected_uwb_rate", 10.0)
        self.rate = rospy.Rate(self.expected_uwb_rate)
        self.expected_duration = 1.00/self.expected_uwb_rate # seconds per cycle

        # Max time (in seconds) to consider UWB readings to happen simulataneously
        # Note that UWB readings happen at 10 Hz by default (ie. 0.1 seconds)
        self.uwb_wait_timeout = 0.5*self.expected_duration # (default:  0.05 seconds)
    
        # Start publishing
        rospy.Timer(rospy.Duration(self.expected_duration), self.combine_uwb_readings)


    def uwb_serial_tag_1_callback(self, data):
        if DEBUG_UWB:
            rospy.logwarn("uwb_serial_tag_1_callback")

        valid, anchor_mat, dists, ids = parse_lec_line(data.data)
        # anchor_mat: stores the position vectors of anchors in world frame
        # dists: stores the distances to the anchors from the tag
        # ids: ids of the anchors

        if not valid:
            rospy.logwarn("NOT VALID")
            # rospy.logwarn(str(data))
            self.uwb_tag_1_valid = False
            return

        self.last_uwb_tag_1_time = rospy.Time.now().to_sec() 
        self.uwb_tag_1_anchors = anchor_mat
        self.uwb_tag_1_anchor_dists = dists 
        self.uwb_tag_1_anchor_ids = ids
        self.uwb_tag_1_valid = True


    def uwb_serial_tag_2_callback(self, data):
        if DEBUG_UWB:
            rospy.logwarn("uwb_serial_tag_2_callback")

        valid, anchor_mat, dists, ids = parse_lec_line(data.data)
        
        if not valid:
            rospy.logwarn("NOT VALID")
            # rospy.logwarn(str(data))
            self.uwb_tag_2_valid = False
            return

        self.last_uwb_tag_2_time = rospy.Time.now().to_sec() 
        self.uwb_tag_2_anchors = anchor_mat
        self.uwb_tag_2_anchor_dists = dists
        self.uwb_tag_2_anchor_ids = ids
        self.uwb_tag_2_valid = True


    def combine_uwb_readings(self, event=None):
        if DEBUG_UWB:
            rospy.logwarn("Combining UWB readings!")
            rospy.logwarn("self.last_uwb_tag_2_time - self.last_uwb_tag_1_time")
            rospy.logwarn(str(self.last_uwb_tag_2_time - self.last_uwb_tag_1_time))

        if (self.uwb_tag_1_valid and self.uwb_tag_2_valid):
            # Check the time difference between the last valid uwb readings. They must be "almost" simultanous.
            if abs(self.last_uwb_tag_2_time - self.last_uwb_tag_1_time) < self.uwb_wait_timeout:
                # Ignore the readings if they are too old.
                dt =  rospy.Time.now().to_sec()- min(self.last_uwb_tag_1_time, self.last_uwb_tag_2_time) 
                if dt > 0.25:
                    rospy.logwarn("UWB timesteps are older than 0.25 s | dt = " + str(dt))
                    return

                # Ignore the reading if there are less than 8 readings
                if self.uwb_tag_1_anchor_dists.size + self.uwb_tag_2_anchor_dists.size < 8:
                    rospy.logwarn("Dropping UWB reading | number of readings = " + str(self.uwb_tag_1_anchor_dists.size + self.uwb_tag_2_anchor_dists.size) + " which is less than 8" )
                    return
                
                # Ignore the reading if there are less than 3 readings per tag
                if (self.uwb_tag_1_anchor_dists.size < 3) or (self.uwb_tag_2_anchor_dists.size < 3):
                    rospy.logwarn("Dropping UWB reading | number of readings 1: " + str(self.uwb_tag_1_anchor_dists.size) +"or 2: "+ str(self.uwb_tag_2_anchor_dists.size) + " is less than 3" )
                    return

                # Correct the readings with the offsets
                uwb_tag_1_anchor_offsets = np.array([[self.antenna_offsets[i] for i in self.uwb_tag_1_anchor_ids]]).T
                uwb_tag_2_anchor_offsets = np.array([[self.antenna_offsets[i] for i in self.uwb_tag_2_anchor_ids]]).T
                
                corrected_uwb_tag_1_dists = self.uwb_tag_1_anchor_dists + self.uwb_tag_1_offset + uwb_tag_1_anchor_offsets
                corrected_uwb_tag_2_dists = self.uwb_tag_2_anchor_dists + self.uwb_tag_2_offset + uwb_tag_2_anchor_offsets

                # Multilateration
                uwb_pos, rmse = tag_pair_min_z(self.uwb_tag_1_anchors, self.uwb_tag_2_anchors,
                                               corrected_uwb_tag_1_dists, corrected_uwb_tag_2_dists, 
                                               self.tag_1_loc, self.tag_2_loc, 
                                               self.tag_z_height)

                # Ignore reading if rmse is high than ... meters
                if rmse > 0.15:
                # if rmse > 0.5:
                    rospy.logwarn("Dropping UWB reading | rmse = " + str(rmse) + " is too high" )
                    return

                # Publish RMSE
                self.pub_rmse.publish(data=rmse)
                
                # Publish Pose
                # Also publish the PoseWithCovarianceStamped msgs
                # Create the PoseWithCovarianceStamped msg
                pose_msg = geometry_msgs.msg.PoseWithCovarianceStamped()
                pose_msg.header.frame_id = self.tf_world_frame
                pose_msg.header.stamp = rospy.Time.now()

                pose_msg.pose.pose.position.x = uwb_pos[0]
                pose_msg.pose.pose.position.y = uwb_pos[1]
                pose_msg.pose.pose.position.z = uwb_pos[2]

                # axis: 0,0,1 (z-axis)
                # angle: th
                # corresponding quaternion: w = cos(th/2), (x,y,z) = sin(th/2)*[0,0,1]
                # hence; 
                pose_msg.pose.pose.orientation.x = 0.0
                pose_msg.pose.pose.orientation.y = 0.0
                pose_msg.pose.pose.orientation.z = math.sin(uwb_pos[3]/2.0)
                pose_msg.pose.pose.orientation.w = math.cos(uwb_pos[3]/2.0)

                pose_msg.pose.covariance = self.calculate_covariance(1.0)

                self.pub_pose.publish(pose_msg)
            
            else:
                rospy.logwarn("Time difference btw. UWB tag readings is too much! Difference=" + str(abs(self.last_uwb_tag_2_time - self.last_uwb_tag_1_time)))
        else:
            rospy.logwarn("At least one of the UWB readings are not valid yet..")
    



    def calculate_covariance(self,reliability_score):
        covariance =  np.zeros(36)
    
        covariance[0]  = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[0],self.covariance_diagonal_max[0]) # x
        covariance[7]  = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[1],self.covariance_diagonal_max[1]) # y
        # covariance[14] = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[2],self.covariance_diagonal_max[2]) # z
        # covariance[21] = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[3],self.covariance_diagonal_max[3]) # rot x
        # covariance[28] = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[4],self.covariance_diagonal_max[4]) # rot y
        covariance[35] = self.interpolate_linear(reliability_score,self.covariance_diagonal_min[2],self.covariance_diagonal_max[2]) # rot z
        covariance = list(covariance) # convert to list of 36 floats

        return covariance

    def interpolate_linear(self,score,min,max):
        return min + (1.0-score)*(max-min)

if __name__ == '__main__':
    uwb_pose_publisher = UWBPosePublisher()
    rospy.spin()