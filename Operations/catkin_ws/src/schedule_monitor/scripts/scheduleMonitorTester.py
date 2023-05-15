#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    This script tests the schedule monitor node by adding and removing tickets
    on a manually set schedule. It simulates how the signals might come in
    during a normal daily operation.
'''

import rospy

from schedule_monitor_msgs.msg import Ticket, Tickets
# from tickets import *
from constants.jobs import anchor_jobs
from utils.job_utils import convert_job_list_to_task_list

complete_ticket_list = convert_job_list_to_task_list(anchor_jobs)
for ticket_id, ticket in complete_ticket_list.items():
    ticket["actual_duration"] = ticket["duration"]
initial_ticket_list = complete_ticket_list
last_ticket_list = {}


class ScheduleMonitorTester:
    '''.'''
    def __init__(self):
        self.ticket_started_sub = rospy.Subscriber(
            'ticket_started', Ticket, self.start_ticket_timer_callback
        )
        self.add_ticket_pub = rospy.Publisher(
            'add_ticket', Tickets, queue_size=100
        )
        self.end_ticket_pub = rospy.Publisher(
            'end_ticket', Ticket, queue_size=100
        )

        # self.run()

    def end_ticket(self, ticket_id, event):
        '''.'''
        msg = Ticket()
        msg.id = ticket_id
        msg.machine_type = complete_ticket_list[ticket_id]["station_type"]
        msg.duration = complete_ticket_list[ticket_id]["duration"]
        msg.related = complete_ticket_list[ticket_id]["parents"]
        self.end_ticket_pub.publish(msg)
        rospy.loginfo(f"Tester: Ending ticket {ticket_id}.")

    def start_ticket_timer_callback(self, msg):
        '''.'''
        ticket_id = msg.id
        # self.end_ticket_id = ticket_id
        ticket_actual_duration_in_mins = \
            complete_ticket_list[ticket_id]["actual_duration"]
        rospy.Timer(
            rospy.Duration(ticket_actual_duration_in_mins), #60*
            lambda event: self.end_ticket(ticket_id, event),
            oneshot=True
        )

    def add_tickets(self, tickets: dict):
        '''Converts tickets to a Tickets msg and publishes it.'''
        msg = Tickets()
        ticket_list = []
        for ticket_id, ticket in tickets.items():
            ticket_msg = Ticket()
            ticket_msg.id = ticket_id
            ticket_msg.machine_type = ticket["station_type"]
            ticket_msg.duration = ticket["duration"]
            ticket_msg.related = ticket["parents"]
            ticket_list.append(ticket_msg)

        msg.tickets = ticket_list
        self.add_ticket_pub.publish(msg)

    # def run(self):
    #     '''.'''

    def add_initial_tix(self, event):
        '''.'''
        self.add_tickets(initial_ticket_list)

    def add_ticket_8(self, event):
        '''.'''
        tix = {}
        tix[8] = complete_ticket_list[8]
        self.add_tickets(tix)

    def add_last_tix(self, event):
        '''.'''
        self.add_tickets(last_ticket_list)

if __name__ == '__main__':
    rospy.init_node('schedule_monitor_tester')
    sMT = ScheduleMonitorTester()
    rospy.Timer(
        rospy.Duration(0.5),
        sMT.add_initial_tix,
        oneshot=True
    )
    # print(complete_ticket_list)
    # rospy.Timer(
    #     rospy.Duration(10),
    #     sMT.add_ticket_8,
    #     oneshot=True
    # )
    # rospy.Timer(
    #     rospy.Duration(15),
    #     sMT.add_last_tix,
    #     oneshot=True
    # )
    rospy.spin()