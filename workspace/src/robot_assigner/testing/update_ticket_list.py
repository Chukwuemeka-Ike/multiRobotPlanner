#!/usr/bin/env python3
import rospy

from arm_msgs.msg import Ticket, Tickets

ticket_dict = {
    # Tree Job 2 roots.
    11: {"job_id": 1, "machine_type": 0, "duration": 5, "parents": [], "actual_duration": 5, "num_robots": 3},
    12: {"job_id": 1, "machine_type": 0, "duration": 5, "parents": [], "actual_duration": 5, "num_robots": 2},
    13: {"job_id": 1, "machine_type": 2, "duration": 45.7, "parents": [11], "actual_duration": 45.7},
    14: {"job_id": 1, "machine_type": 3, "duration": 54.8, "parents": [12], "actual_duration": 54.8},
    15: {"job_id": 1, "machine_type": 1, "duration": 89.2, "parents": [13,14], "actual_duration": 89.2},
    16: {"job_id": 1, "machine_type": 3, "duration": 43.1, "parents": [15], "actual_duration": 43.1},
    17: {"job_id": 1, "machine_type": 7, "duration": 38.7, "parents": [16], "actual_duration": 38.7},
    # Tree Job 3 roots.
    1: {"job_id": 0, "machine_type": 0, "duration": 10, "parents": [], "actual_duration": 10, "time_left": 10, "num_robots": 2},
    2: {"job_id": 0, "machine_type": 0, "duration": 5, "parents": [], "actual_duration": 5, "time_left": 5, "num_robots": 1},
    3: {"job_id": 0, "machine_type": 0, "duration": 5, "parents": [], "actual_duration": 7, "time_left": 5, "num_robots": 3},
    4: {"job_id": 0, "machine_type": 2, "duration": 45, "parents": [1], "actual_duration": 55, "time_left": 45},
    5: {"job_id": 0, "machine_type": 2, "duration": 30, "parents": [2], "actual_duration": 25, "time_left": 30},
    6: {"job_id": 0, "machine_type": 3, "duration": 30, "parents": [3], "actual_duration": 28, "time_left": 30},
    7: {"job_id": 0, "machine_type": 2, "duration": 60, "parents": [4,5], "actual_duration": 75, "time_left": 60},
    8: {"job_id": 0, "machine_type": 2, "duration": 30, "parents": [6,7], "actual_duration": 30, "time_left": 30},
    9: {"job_id": 0, "machine_type": 3, "duration": 45, "parents": [8], "actual_duration": 48, "time_left": 45},
    10: {"job_id": 0, "machine_type": 4, "duration": 60, "parents": [9], "actual_duration": 60, "time_left": 60},

    24: {"job_id": 4, "ticket_id": 24, "machine_type": 0, "actual_duration": 0.5, "duration": 0.5, "parents": [], "num_robots": 5},
    25: {"job_id": 4, "ticket_id": 25, "machine_type": 1, "actual_duration": 93.1, "duration": 93.1, "parents": [24]},
    26: {"job_id": 4, "ticket_id": 26, "machine_type": 2, "actual_duration": 51.7, "duration": 51.7, "parents": [25]},
    27: {"job_id": 4, "ticket_id": 27, "machine_type": 3, "actual_duration": 40.5, "duration": 40.5, "parents": [26]},
    28: {"job_id": 4, "ticket_id": 28, "machine_type": 4, "actual_duration": 39, "duration": 39, "parents": [27]},
    29: {"job_id": 4, "ticket_id": 29, "machine_type": 7, "actual_duration": 52.7, "duration": 52.7, "parents": [28]},
}


if __name__ == "__main__":
    rospy.init_node('ticket_adder')
    ticket_list_update_pub = rospy.Publisher(
        'ticket_list_update', Tickets, queue_size=100
    )
    
    # Wait 1 second, or the message won't publish.
    rate = rospy.Rate(1)
    rate.sleep()
    
    msg = Tickets()
    ticket_list = []
    for ticket_id, ticket in ticket_dict.items():
        ticket_msg = Ticket()
        ticket_msg.job_id = ticket["job_id"]
        ticket_msg.ticket_id = ticket_id
        ticket_msg.parents = ticket["parents"]
        ticket_msg.duration = ticket["duration"]
        ticket_msg.machine_type = ticket["machine_type"]
        if len(ticket_msg.parents) == 0:
            ticket_msg.num_robots = ticket["num_robots"]
        ticket_list.append(ticket_msg)

    msg.tickets = ticket_list
    ticket_list_update_pub.publish(msg)
    rospy.loginfo(f"Published ticket set.")

    # Shutdown the node
    rospy.signal_shutdown('Message published')
