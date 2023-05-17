#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
'''

import rospy
import time

from std_msgs.msg import String
from schedule_monitor_msgs.msg import Ticket, Tickets
from schedule_monitor_msgs.srv import Schedule, ScheduleRequest

from constants.stations import *
from constants.locations import station_locations, stations

from utils.display_utils import display_solution_stats, display_solver_information,\
    display_task_list
from utils.map_utils import *


class TrajectoryPlanner:
    '''.'''

    def __init__(self):
        '''.'''
        rospy.init_node('trajectory_planner')
        # rospy.on_shutdown()

        self.robot_assignments = {} # Job_id's to robot_id's.
        self.next_destinations = {} # Job_id's next destination.
        self.new_schedule_sub = rospy.Subscriber(
            "new_schedule", String, self.new_schedule_callback
        )
        self.end_ticket_sub = rospy.Subscriber(
            "end_ticket", Ticket, self.end_ticket_message_callback
        )
        self.ticket_dict = {}

    def update_next_destinations(self):
        '''Checks if the next destinations are correct. Updates if not.'''

    def new_schedule_callback(self, msg):
        '''Called when a new schedule is created.'''
        self.schedule_client()

    def end_ticket_message_callback(self, msg):
        '''Triggered when a ticket ends.'''
        self.schedule_client()

    def schedule_client(self):
        '''.'''
        rospy.wait_for_service('schedule_service')
        try:
            schedule = rospy.ServiceProxy('schedule_service', Schedule)
            response = schedule()
            self.add_tickets_to_task_list(response.tickets)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}.')
    
    def add_tickets_to_task_list(self, ticket_list):
        '''Adds the list of tickets to the task_list and waiting.'''
        for ticket in ticket_list:
            # Create the ticket dictionary.
            tix = {}
            tix["job_id"] = ticket.job_id
            tix["station_type"] = ticket.machine_type
            tix["duration"] = ticket.duration
            tix["parents"] = list(ticket.parents)
            tix["start"] = ticket.start
            tix["end"] = ticket.end
            tix["station_num"] = ticket.station_num

            # Add the ticket to task_list and waiting set.
            self.ticket_dict[ticket.ticket_id] = tix

if __name__ == '__main__':
    TP = TrajectoryPlanner()
    TP.schedule_client()
    # display_task_list(TP.ticket_dict)
    rospy.spin()