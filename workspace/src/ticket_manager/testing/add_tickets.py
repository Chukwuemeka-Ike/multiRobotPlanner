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

# Different sets of jobs for testing different functions.
# from arm_constants.test_jobs_anchor import complete_ticket_list
# from arm_constants.test_jobs_demo_physical import complete_ticket_list
from arm_constants.test_jobs_user_study import complete_ticket_list
# from arm_constants.test_jobs_single import complete_ticket_list
# from arm_constants.test_jobs_small import complete_ticket_list
# from arm_constants.test_jobs_tree import complete_ticket_list


def add_tickets(publisher, tickets: dict):
    '''Convert tickets dict to a Tickets msg and publish it.'''
    msg = Tickets()
    ticket_list = []
    for ticket_id, ticket in tickets.items():
        ticket_msg = Ticket()
        ticket_msg.ticket_id = ticket_id
        ticket_msg.machine_type = ticket["machine_type"]
        ticket_msg.duration = ticket["duration"]
        ticket_msg.parents = ticket["parents"]
        if len(ticket["parents"]) == 0:
            ticket_msg.num_robots = ticket["num_robots"]
        ticket_list.append(ticket_msg)

    msg.tickets = ticket_list
    publisher.publish(msg)
    rospy.loginfo(f"Published ticket set.")


if __name__ == "__main__":
    rospy.init_node('ticket_adder')

    add_ticket_pub = rospy.Publisher(
        'add_ticket', Tickets, queue_size=100
    )

    # # Multiply by 60 to change durations to minutes moving forward.
    # for _, ticket in complete_ticket_list.items():
    #     ticket["duration"] = int(ticket["duration"]*60)

    # Wait 1 second, or the message won't publish.
    time.sleep(1)
    add_tickets(add_ticket_pub, complete_ticket_list)

    # Shutdown the node
    rospy.signal_shutdown('Message published')
