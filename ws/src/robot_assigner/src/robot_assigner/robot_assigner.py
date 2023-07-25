#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
'''
import rospy

from arm_msgs.msg import Ticket, Tickets
from arm_msgs.srv import TicketList, TicketListRequest

from arm_utils.conversion_utils import convert_ticket_list_to_task_dict,\
    convert_task_list_to_job_list
from arm_utils.job_utils import get_all_children_from_task_list,\
    get_all_job_start_points


log_tag = "Robot Assigner"


class RobotAssigner():
    '''Robot assigner class. Generates and maintains assignments for
    robots to jobs.

    Assignments are given from robot to the starting point(s) of a job,
    and those assignments are inherited by the children of the start point(s).
    '''

    def __init__(self) -> None:
        '''.'''
        # Spawn the ROS node and announce its startup.
        rospy.init_node('robot_assigner')
        rospy.on_shutdown(self.shutdown_robot_assigner)
        rospy.loginfo(f"{log_tag}: Node started.")

        # Main data structures for the node.
        # Assignments is a dictionary with job IDs as keys and dictionaries
        # of the jobs' task-robot assignments as values.
        # {1: {1: [1,2], 2: [3], 3: [4,5,6], 4: [1,2,3], 5: [1,2,3,4,5,6]}}.
        self.assignments = {}

        # Robot pool - the set of available and occupied robots.
        # Both are just lists of IDs
        self.available = []
        self.occupied = {}

        # Set of jobs with assigned robots. Dictionary of job IDs with list of
        # assigned IDs.
        self.jobs_with_assigned_robots = {}

        # Ticket dict. Allows us compare the previous with the current.
        self.ticket_dict = {}

        # Subscriber for ticket list updates. Ticket Manager is the publisher.
        self.ticket_list_update_sub = rospy.Subscriber(
            'ticket_list_update', Tickets, self.update_robot_assignments
        )

        # Service for robot assignments. Provides the assignment for a
        # specified task.
        self.robot_assignment_service = rospy.Service(
            'robot_assignment_service',
            RobotAssignment,
            self.send_robot_assignment
        )

    def startup_assigner(self):
        '''.'''
        # Try to load saved assignments if they exist.
        # Get the available robot IDs.
        # Cross-check them against the saved assignments

    def shutdown_robot_assigner(self):
        '''Gracefully shutdown ticket manager.'''
        # TODO: Save the current assignments in case of incorrect shutdowns
        # and to allow us pick up the next shift.
        rospy.loginfo(f"{log_tag}: Node shutdown.")

    def update_robot_assignments(self, msg):
        '''Updates the robot assignments when the ticket list is updated.'''
        # Convert the ticket list to a job list.
        self.task_dict = convert_ticket_list_to_task_dict(msg.tickets)
        self.job_list = convert_task_list_to_job_list(self.task_dict)

        # Go through each job and save their starting point IDs.
        start_points = get_all_job_start_points(self.job_list, self.task_dict)

        num_robots_needed = []

        # Set of job IDs which we can' assign robots to in first pass.
        second_pass_jobs = []

        # Iterate through the job list.
        for job_id, job_start_points in start_points.items():
            # Count how many robots are needed for the job.
            job_num_robots = 0

            for starter in job_start_points:
                job_num_robots += self.task_dict[starter]["num_robots"]

            if job_num_robots <= len(self.available):
                job_assignments = self.assign_robots_to_job(
                    job_num_robots, job_start_points
                )
            else:
                second_pass_jobs.append(job_id)

            # Append the number needed for the job
            num_robots_needed.append(job_num_robots)

    def assign_robots_to_job(self, job_num_robots: int, job_start_points: list):
        '''Assigns robots to the jobs.
        
        Assigns the number of robots needed from the available set. Starts with
        the job's starting points, and assigns the same IDs to all the
        children. That way, the same number of robots is assigned at every
        level of the tree.

        Args:
            job_num_robots
            job_start_points
        '''
        job_assignments = {}
        for start_point in job_start_points:
            # Get the number of robots needed for the start point.
            num_robots = self.task_dict[start_point]["num_robots"]

            # Get num_robots from available set and remove them from it.
            assigned_robots = self.available[:num_robots]
            for robot in assigned_robots:
                self.available.remove(robot)

            # Get all the children of the current start point.
            linear_job = [start_point]
            get_all_children_from_task_list(
                start_point, self.task_dict, linear_job
            )

            # Add the assignments to each task in that branch.
            for task in linear_job:
                # Add the assignments to that task if it's already in 
                # the dictionary. Otherwise, create a new element.
                if task in job_assignments:
                    job_assignments[task] += assigned_robots
                else:
                    job_assignments[task] = assigned_robots

        # Return the assignments
        return job_assignments