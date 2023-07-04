#!/usr/bin/env python3  

"""
Author: Burak Aksoy
Node: wrench_transformer_node
Description:
    get the transform btw. two specified tf frames A and B.
    subscribe to a wrench_stamped message published for frame A. 
    Assuming frames are rigidly linked, transform the wrench from frame A to frame B.
    
Parameters:
    - TODO
Subscribes to:
    - tf2
    - geometry_msgs::WrenchStamped
Publishes to:
    - geometry_msgs::Wrench (My use case needed a non-stamped wrench msg, hence did not make it stamped)

Broadcasts to:
    - NONE
"""

import rospy

import numpy as np
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg 

# # Because of transformations
import tf_conversions
#  tf_conversions.transformations.euler_from_quaternion(Q_eg)
import tf.transformations 

class WrenchTransformer():
    def __init__(self):
        rospy.init_node('wrench_transformer', anonymous=True)

        # Topic name to publish
        self.wrench_topic_name_out = rospy.get_param("~wrench_topic_name_out", "tool_wrench_filtered")
        # Topic name to subsribe
        self.wrench_topic_name_in = rospy.get_param("~wrench_topic_name_in", "tool_wrench_raw")
        # Specified tf frame names
        self.tf_a_frame_name = rospy.get_param("~tf_a_frame_name", "tf_a_link")
        self.tf_b_frame_name = rospy.get_param("~tf_b_frame_name", "tf_b_link")

        # Publisher
        self.pub_wrench = rospy.Publisher(self.wrench_topic_name_out, geometry_msgs.msg.Wrench, queue_size=1)

        # Subscriber
        rospy.Subscriber(self.wrench_topic_name_in, geometry_msgs.msg.WrenchStamped, self.wrench_callback , queue_size=1)
        
        # TF2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.T_a2b = None
        self.is_ok_tf = False

        # To store the applied force on frame a
        self.F_a = [0.,0.,0.,0.,0.,0.]

        # Publish rate
        self.pub_rate = rospy.get_param("~pub_rate", 100.0)
        self.rate = rospy.Rate(self.pub_rate)
        self.expected_duration = 1.00/self.pub_rate # seconds per cycle

        self.wait_timeout = 10.0*self.expected_duration
        self.last_msg_time = 0.0

        # Start control
        rospy.Timer(rospy.Duration(self.expected_duration), self.wrench_transformer)


    def wrench_transformer(self, event=None):
        # Find the transform between the specified joint and the end effector
        self.is_ok_tf = self.look_tfs()

        if not self.is_ok_tf:
            # Do not publish wrench since the transformation could not found
            self.publish_wrench([0.,0.,0.],[0.,0.,0.])
        else:
            # Publish the command to move the end effector to the body joint
            self.transform_and_publish_wrench()

    def look_tfs(self):
        try:
            # returns type geometry_msgs.msg.TransformStamped
            self.T_a2b = self.tfBuffer.lookup_transform(self.tf_a_frame_name, self.tf_b_frame_name,  rospy.Time()) # in base frame 

            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Put a warning which says that the transformation could not found
            rospy.logwarn_once('wrench_transformer: Waiting to find the transformation from %s to %s' 
                            % (self.tf_a_frame_name, self.tf_b_frame_name)) 
            return False

    def transform_and_publish_wrench(self):
        f_a = np.array(self.F_a[0:3]) # force in frame a
        t_a = np.array(self.F_a[3:6]) # torque in frame a

        P_a2b_in_a_x = self.T_a2b.transform.translation.x
        P_a2b_in_a_y = self.T_a2b.transform.translation.y
        P_a2b_in_a_z = self.T_a2b.transform.translation.z
        P_a2b_in_a = np.array([P_a2b_in_a_x,P_a2b_in_a_y,P_a2b_in_a_z])

        qw_cur = self.T_a2b.transform.rotation.w # Scalar part of quaternion
        qx_cur = self.T_a2b.transform.rotation.x
        qy_cur = self.T_a2b.transform.rotation.y
        qz_cur = self.T_a2b.transform.rotation.z
        q_a2b = [qx_cur,qy_cur,qz_cur, qw_cur]
        R_a2b = tf.transformations.quaternion_matrix(q_a2b) # 4x4
        R_b2a = R_a2b[:3,:3].T 

        f_b = np.dot(R_b2a,f_a)
        t_b = np.dot(R_b2a,t_a) - np.dot(R_b2a, np.cross(P_a2b_in_a,f_a))

        self.publish_wrench(f_b,t_b)

    def publish_wrench(self, f,t):
        if not ((rospy.Time.now().to_sec() - self.last_msg_time) > self.wait_timeout):
            wrench_msg = geometry_msgs.msg.Wrench()
            wrench_msg.force.x = f[0]
            wrench_msg.force.y = f[1]
            wrench_msg.force.z = f[2]
            wrench_msg.torque.x = t[0]
            wrench_msg.torque.y = t[1]
            wrench_msg.torque.z = t[2]
            self.pub_wrench.publish(wrench_msg)
        else:
            wrench_msg = geometry_msgs.msg.Wrench()
            wrench_msg.force.x = 0.0
            wrench_msg.force.y = 0.0
            wrench_msg.force.z = 0.0
            wrench_msg.torque.x = 0.0
            wrench_msg.torque.y = 0.0
            wrench_msg.torque.z = 0.0
            self.pub_wrench.publish(wrench_msg)

    """
    def publishWrenchStamped(self, header, wrench):
        wrench_stamped_msg = geometry_msgs.msg.WrenchStamped()
        wrench_stamped_msg.header = header

        # now = rospy.Time.now()
        # rospy.loginfo("WRENCH: Added time delay %i secs %i nsecs", (now.secs - header.stamp.secs), (now.nsecs -header.stamp.nsecs))
        
        wrench_stamped_msg.wrench.force.x = wrench[3]
        wrench_stamped_msg.wrench.force.y = wrench[4]
        wrench_stamped_msg.wrench.force.z = wrench[5]
        wrench_stamped_msg.wrench.torque.x = wrench[0]
        wrench_stamped_msg.wrench.torque.y = wrench[1]
        wrench_stamped_msg.wrench.torque.z = wrench[2]

        self.pub_wrench.publish(wrench_stamped_msg)
    """

    def wrench_callback(self, wrench_stamped_msg):
        F_lin_x = wrench_stamped_msg.wrench.force.x
        F_lin_y = wrench_stamped_msg.wrench.force.y
        F_lin_z = wrench_stamped_msg.wrench.force.z
        F_ang_x = wrench_stamped_msg.wrench.torque.x
        F_ang_y = wrench_stamped_msg.wrench.torque.y
        F_ang_z = wrench_stamped_msg.wrench.torque.z

        self.F_a = [F_lin_x,F_lin_y,F_lin_z,F_ang_x,F_ang_y,F_ang_z]
        self.last_msg_time = rospy.Time.now().to_sec()

if __name__ == '__main__':
    wrenchTransformer = WrenchTransformer()
    rospy.spin()