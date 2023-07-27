#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Test script for deleting a target ticket.
'''
import rospy
import sys

from arm_msgs.msg import Ticket


if __name__ == "__main__":
    rospy.init_node('job_deleter')
    delete_job_pub = rospy.Publisher(
        'delete_job', Ticket, queue_size=100
    )
    # Wait 1 second, or the message won't publish.
    rate = rospy.Rate(1)
    rate.sleep()
    
    msg = Ticket()
    if len(sys.argv) > 1:
        msg.job_id = int(sys.argv[1])
    else:
        msg.job_id = 1
    delete_job_pub.publish(msg)

    # Shutdown the node
    rospy.signal_shutdown('Message published')