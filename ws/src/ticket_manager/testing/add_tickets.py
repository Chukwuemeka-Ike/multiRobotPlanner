import rospy

from arm_msgs.msg import Tickets, Ticket
from arm_constants.test_jobs import *


def add_tickets(publisher, tickets: dict):
    '''Converts tickets to a Tickets msg and publishes it.'''
    msg = Tickets()
    ticket_list = []
    for ticket_id, ticket in tickets.items():
        ticket_msg = Ticket()
        ticket_msg.ticket_id = ticket_id
        ticket_msg.job_id = ticket["job_id"]
        ticket_msg.machine_type = ticket["station_type"]
        ticket_msg.duration = ticket["duration"]
        ticket_msg.parents = ticket["parents"]
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
    rate = rospy.Rate(1)
    rate.sleep()
    # while not rospy.is_shutdown():
    add_tickets(add_ticket_pub, test_data_1["ticket_add_0"])
    # add_tickets(add_ticket_pub, test_data_1["ticket_add_1"])
    # add_tickets(add_ticket_pub, test_data_1["ticket_add_2"])
    # add_tickets(add_ticket_pub, test_data_1["ticket_add_3"])

    # Shutdown the node
    rospy.signal_shutdown('Message published')
