#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Test script for starting a target ticket.
'''
import rospy
import sys

from arm_msgs.msg import Ticket


if __name__ == "__main__":
    rospy.init_node('ticket_adder')
    start_ticket_pub = rospy.Publisher(
        'start_ticket', Ticket, queue_size=100
    )
    # Wait 1 second, or the message won't publish.
    rate = rospy.Rate(1)
    rate.sleep()
    
    msg = Ticket()
    if len(sys.argv) > 1:
        msg.ticket_id = int(sys.argv[1])
    else:
        msg.ticket_id = 18
    start_ticket_pub.publish(msg)

    # Shutdown the node
    rospy.signal_shutdown('Message published')
