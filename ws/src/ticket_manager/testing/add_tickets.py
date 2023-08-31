#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
'''
import rospy
import time

from arm_msgs.msg import Tickets, Ticket
# from arm_constants.test_jobs_jobless import *
from arm_constants.test_jobs_small import *


def add_tickets(publisher, tickets: dict):
    '''Converts tickets to a Tickets msg and publishes it.'''
    msg = Tickets()
    ticket_list = []
    for ticket_id, ticket in tickets.items():
        ticket_msg = Ticket()
        ticket_msg.ticket_id = ticket_id
        ticket_msg.machine_type = ticket["machine_type"]
        ticket_msg.duration = ticket["duration"]
        ticket_msg.parents = ticket["parents"]
        if len(ticket["parents"]) == 0:
            # try:
            ticket_msg.num_robots = ticket["num_robots"]
            # except:
            #     pass
        ticket_list.append(ticket_msg)

    msg.tickets = ticket_list
    publisher.publish(msg)
    rospy.loginfo(f"Published ticket set.")


if __name__ == "__main__":
    rospy.init_node('ticket_adder')
    add_ticket_pub = rospy.Publisher(
        'add_ticket', Tickets, queue_size=100
    )
    
    # Wait 1 second, or the message won't publish.
    time.sleep(1)

    add_tickets(add_ticket_pub, test_data_1["ticket_add_0"])
    # add_tickets(add_ticket_pub, test_data_1["ticket_add_1"])
    # add_tickets(add_ticket_pub, test_data_1["ticket_add_2"])
    # add_tickets(add_ticket_pub, test_data_1["ticket_add_3"])

    # Shutdown the node
    rospy.signal_shutdown('Message published')
