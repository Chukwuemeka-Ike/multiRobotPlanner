#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Test script for deleting a target ticket.
'''
import rospy

from arm_msgs.msg import Ticket, Tickets


if __name__ == "__main__":
    rospy.init_node('ticket_adder')
    delete_ticket_pub = rospy.Publisher(
        'delete_ticket', Tickets, queue_size=100
    )
    # Wait 1 second, or the message won't publish.
    rate = rospy.Rate(1)
    rate.sleep()
    
    msg = Tickets()
    ticket_ids = [6,7,8,12,28]
    ticket_list = []

    for ticket_id in ticket_ids:
        ticket = Ticket()
        ticket.ticket_id = ticket_id
        ticket_list.append(ticket)

    msg.tickets = ticket_list
    delete_ticket_pub.publish(msg)

    # Shutdown the node
    rospy.signal_shutdown('Message published')