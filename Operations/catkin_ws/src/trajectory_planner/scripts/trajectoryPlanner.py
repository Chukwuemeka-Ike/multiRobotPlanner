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
from trajectory_planner_msgs.msg import Desired_Locations

from constants.stations import *
from constants.locations import station_locations, stations

from utils.display_utils import display_solution_stats, display_solver_information,\
    display_task_list
from utils.job_utils import get_immediate_child
from utils.map_utils import *


# Distance in ft. Time in seconds.
bounds = [0, 150, 0, 42]
spaceStep = 3

# Set up the grid.
grid = create_grid_map(bounds, spaceStep)
numStates = grid.size
print(f"Number of possible states (grid size): {numStates}")
print(f"Grid shape: {grid.shape}")


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
        self.path_pub = rospy.Publisher(
            "desired_path", Desired_Locations, queue_size=100
        )
        self.ticket_dict = {}

    def update_next_destinations(self, child_ticket_id):
        '''Checks if the next destinations are correct. Updates if not.'''
        job_id = self.ticket_dict[child_ticket_id]["job_id"]
        if job_id not in self.next_destinations:
            self.next_destinations[job_id] = \
                self.ticket_dict[child_ticket_id]["station_num"]
        elif self.next_destinations[job_id] == self.ticket_dict[child_ticket_id]["station_num"]:
            pass
        else:
            # Need to 
            rospy.loginfo(f"Updating the next destination for job {job_id}.")
            self.next_destinations[job_id] = \
                self.ticket_dict[child_ticket_id]["station_num"]

    def new_schedule_callback(self, msg):
        '''Called when a new schedule is created.'''
        self.request_schedule()

    def get_station_coordinates(self, station_num: int, ):
        '''Gets.'''
        station = station_locations[station_num]
        station_neighbors = get_station_neighbors(station, grid, stations)
        return get_coordinates(
            station_neighbors[0], grid, spaceStep
        )

    def get_swarm_location(self, ticket_id):
        '''Gets the current location of the swarm.'''
        # Currently just uses the location of the ticket.
        # TODO: Change this to use the location of the swarm.
        ticket_station_num = self.ticket_dict[ticket_id]["station_num"]
        
        ticket_coordinates = self.get_station_coordinates(ticket_station_num)
        return ticket_coordinates


    def end_ticket_message_callback(self, msg):
        '''Triggered when a ticket ends.'''
        self.request_schedule()
        ticket_id = msg.ticket_id
        child_ticket_id = get_immediate_child(ticket_id, self.ticket_dict)
        # TODO: If there is no child, do we just go back to base? Just chill out?
        self.update_next_destinations(child_ticket_id)

        path_msg = Desired_Locations()
        path_msg.job_id = self.ticket_dict[ticket_id]["job_id"]
        
        ticket_coordinates = self.get_station_coordinates(
            self.ticket_dict[ticket_id]["station_num"]
        )
        # TODO: If there is no child, do we just go back to base? Just chill out?
        child_coordinates = self.get_station_coordinates(
            self.ticket_dict[child_ticket_id]["station_num"]
        )

        rospy.loginfo(f"TP: Ticket Coordinates: {ticket_coordinates}. Ticket ID: {ticket_id}.")
        rospy.loginfo(f"TP: Child Coordinates: {child_coordinates}. Child ID: {child_ticket_id}.")
        
        path_msg.start.position.x = ticket_coordinates[0]
        path_msg.start.position.y = ticket_coordinates[1]
        path_msg.goal.position.x = child_coordinates[0]
        path_msg.goal.position.y = child_coordinates[1]

        self.path_pub.publish(path_msg)

    def request_schedule(self):
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
    # TP.request_schedule()
    # display_task_list(TP.ticket_dict)
    rospy.spin()