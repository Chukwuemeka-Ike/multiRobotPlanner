#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Robot Assigner class definition.
'''
import rospy

from std_msgs.msg import String

from arm_msgs.msg import StringList
from arm_msgs.srv import FleetInformation, FleetInformationRequest,\
    FleetInformationResponse, RobotAssignments, RobotAssignmentsRequest,\
    RobotAssignmentsResponse, TicketList, TicketListRequest

from arm_utils.conversion_utils import convert_ticket_list_to_task_dict,\
    convert_task_list_to_job_list
from arm_utils.job_utils import get_all_children_from_task_list,\
    get_all_job_start_points, get_job_last_ticket_status, has_job_started


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

        # Get the number of robots in the fleet.
        self.fleet_size = rospy.get_param("fleet_size")

        # Robot pool - the sets of available and occupied robots.
        # Available is [IDs], and occupied is {id: job_id}.
        self.available = list(range(1, self.fleet_size+1))
        self.occupied = {}

        # Get the per-robot parameters from assigner_params.yaml.
        self.robot_name = rospy.get_param("robot_name")
        self.robot_command_topic = rospy.get_param("robot_command_topic")
        self.robot_frame_command_topic = rospy.get_param("robot_frame_command_topic")
        self.real_robot_frame_name = rospy.get_param("real_robot_frame_name")
        self.virtual_robot_frame_name = rospy.get_param("virtual_robot_frame_name")
        self.robot_desired_state_topic = rospy.get_param("robot_desired_state_topic")
        self.robot_node_names = rospy.get_param("robot_node_names")

        # Create lists for each parameter.
        # The number of robots determines the length of each list.
        self.robot_names = [
            self.robot_name + str(i) for i in self.available
        ]
        self.robot_command_topics = [
            self.robot_command_topic.replace("/d_", f"/d{i}") for i in self.available
        ]
        self.robot_frame_command_topics = [
            self.robot_frame_command_topic.replace("robot_", f"robot_{i}_") for i in self.available
        ]
        self.real_robot_frame_names = [
            self.real_robot_frame_name.replace("d_", f"d{i}_") for i in self.available
        ]
        self.virtual_robot_frame_names = [
            self.virtual_robot_frame_name + str(i) for i in self.available
        ]
        self.robot_desired_state_topics = [
            self.robot_desired_state_topic.replace("/d_", f"/d{i}") for i in self.available
        ]

        # print(f"Desired state topics: {self.robot_desired_state_topics}")
        # These topics are constant across robots and teams.
        self.tf_changer_topic = rospy.get_param("tf_changer_topic")
        self.robot_enable_status_topic = rospy.get_param("robot_enable_status_topic")

        # Get the per-team parameters.
        # Whenever a ticket's assignments are requested, these are also sent.
        self.team_command_topic = rospy.get_param("team_command_topic")
        self.team_frame_command_topic = rospy.get_param("team_frame_command_topic")
        self.team_footprint_topic = rospy.get_param("team_footprint_topic")
        self.team_tf_frame_name = rospy.get_param("team_tf_frame_name")

        # Assignments is a dictionary with job IDs as keys and dictionaries
        # of the jobs' ticket-robot assignments as values.
        # {job_id: {ticket_id: [robot_ids], ...}, ...}.
        # {1: {1: [1,2], 2: [3], 3: [4,5,6], 4: [1,2,3], 5: [1,2,3,4,5,6]}}.
        self.assignments = {}

        # Set of jobs with assigned robots. Dictionary of job IDs with list of
        # assigned IDs. Makes it easier to free robot IDs.
        self.job_assigned_ids = {}

        # Ticket dict and job list. These come from the Ticket Manager.
        self.all_tickets = {}
        self.job_list = []

        # Team. Dictionary mapping tickets to the team IDs that are associated
        # with them. When we send this information over, it is the lowest ID
        # that is given to that team.
        # {ticket_id: [team_ids], ...}.
        self.teams = {}

        # Lowest team ID that can be given to a new team.
        self.minTeamID = 1

        # Subscriber for ticket list updates. Ticket Manager is the publisher.
        self.ticket_list_update_sub = rospy.Subscriber(
            'ticket_list_update', String, self.request_ticket_list
        )

        # Service for robot assignments. Provides the assignment for a
        # specified ticket.
        self.robot_assignments_service = rospy.Service(
            'robot_assignments_service',
            RobotAssignments,
            self.send_robot_assignments
        )

        # Service for fleet information. Provides info about all robots.
        self.fleet_information_service = rospy.Service(
            'fleet_information_service',
            FleetInformation,
            self.send_fleet_information
        )

        # Request the ticket list on startup.
        self.request_ticket_list(None)

        rospy.spin()

    def startup_robot_assigner(self):
        '''.'''
        # TODO.
        # Try to load saved assignments if they exist.
        # Get the available robot IDs.
        # Cross-check them against the saved assignments.

    def shutdown_robot_assigner(self):
        '''Gracefully shutdown ticket manager.'''
        # TODO: Save the current assignments in case of incorrect shutdowns
        # and to allow us pick up the next shift.
        rospy.loginfo(f"{log_tag}: Node shutdown.")

    def send_fleet_information(self, _: FleetInformationRequest):
        '''Sends information about all the individual robots in the fleet.

        The fleet information is the topics, names, and frames for all robots,
        so other nodes can get all topics at once and subsequently index using
        relevant IDs.
        '''
        robot_node_names = []
        for idx in range(len(self.robot_names)):

            nodes = StringList()
            nodes.string_list = self.robot_node_names[idx]
            robot_node_names.append(nodes)

        return FleetInformationResponse(
            self.fleet_size,
            self.robot_names,
            self.robot_command_topics,
            self.robot_frame_command_topics,
            self.real_robot_frame_names,
            self.virtual_robot_frame_names,
            self.robot_desired_state_topics,
            robot_node_names,
            self.tf_changer_topic,
            self.robot_enable_status_topic
        )

    def send_robot_assignments(self, request: RobotAssignmentsRequest):
        '''Sends info about the robots and team assigned to a requested ticket ID.

        If the job_id isn't in assignments, there are no assigned robots, and
        we return empty lists and strings.
        '''
        ticket_id = request.ticket_id
        job_id = self.all_tickets[ticket_id]["job_id"]

        # Check if the job has any assignments.
        if job_id in self.assignments:
            assigned_ids = self.assignments[job_id][ticket_id]
            # Sort so the Operator GUI shows them in numeric order.
            assigned_ids.sort()
        else:
            assigned_ids = []

        num_assigned_robots = len(assigned_ids)

        # Robot IDs start at 1, so subtract 1 to get the indices for
        # accessing their info from the robot info lists.
        indices = [id-1 for id in assigned_ids]
        robot_names = []
        robot_frame_command_topics = []
        robot_command_topics = []
        virtual_robot_frame_names = []
        real_robot_frame_names = []
        robot_node_names = []
        robot_desired_state_topics = []

        for idx in indices:
            robot_names.append(self.robot_names[idx])
            robot_frame_command_topics.append(self.robot_frame_command_topics[idx])
            robot_command_topics.append(self.robot_command_topics[idx])
            virtual_robot_frame_names.append(self.virtual_robot_frame_names[idx])
            real_robot_frame_names.append(self.real_robot_frame_names[idx])
            robot_desired_state_topics.append(self.robot_desired_state_topics[idx])

            nodes = StringList()
            nodes.string_list = self.robot_node_names[idx]
            robot_node_names.append(nodes)

        team_id = 0
        team_command_topic = ""
        team_frame_command_topic = ""
        team_footprint_topic = ""
        team_tf_frame_name = ""
        if ticket_id in self.teams:
            team_id = min(self.teams[ticket_id])
            team_command_topic = self.team_command_topic.replace("team_", f"team_{team_id}_")
            team_frame_command_topic = self.team_frame_command_topic.replace("team_", f"team_{team_id}_")
            team_footprint_topic = self.team_footprint_topic.replace("team_", f"team_{team_id}_")
            team_tf_frame_name = self.team_tf_frame_name.replace("team_", f"team_{team_id}_")

        return RobotAssignmentsResponse(
            num_assigned_robots,
            assigned_ids,
            team_id,
            team_command_topic,
            team_frame_command_topic,
            team_footprint_topic,
            team_tf_frame_name,
            # "desired_swarm_vel",
            # "just_swarm_vel",
            # "/swarm_footprint",
            # "/swarm_frame"
        )

    def request_ticket_list(self, _):
        '''Requests the current ticket list from the ticket_service.

        Called whenever a ticket_list_update message is received.
        '''
        rospy.wait_for_service('ticket_service')

        try:
            # TicketListRequest() is empty.
            request = TicketListRequest()
            ticket_list = rospy.ServiceProxy('ticket_service', TicketList)
            response = ticket_list(request)

            # Convert the ticket list to a job list and ticket dictionary.
            self.all_tickets = convert_ticket_list_to_task_dict(response.all_tickets)
            self.job_list = convert_task_list_to_job_list(self.all_tickets)

            self.waiting = response.waiting
            self.ready = response.ready
            self.ongoing = response.ongoing
            self.done = response.done

            self.update_robot_assignments()
        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Ticket list request failed: {e}.')

    def update_robot_assignments(self):
        '''Updates the robot assignments when the ticket list is updated.'''
        # Note that the ticket dict, job list, and start points only have
        # information about tickets that are still in the ticket manager.
        # Anything that was deleted in the most recent update won't be in
        # these and has to be taken care of.

        # Holds the jobs without assignments that will be passed to the
        # new job assignment function.
        unassigned_jobs = []

        # Deleted job.
        # Check for tickets that have been deleted. If one is deleted, we
        # remove the whole job assignment (single tickets can't be deleted
        # without deleting the whole job).
        deleted_jobs = []
        for job_id, ticket_assignments in self.assignments.items():
            for ticket_id, _ in ticket_assignments.items():
                if ticket_id not in self.all_tickets:
                    deleted_jobs.append(job_id)
                    break

        for job_id in deleted_jobs:
            self.release_assigned_robots(job_id)

        # Look through the job list for jobs with pre-existing assignments.
        for job in self.job_list:
            any_ticket = job[0]
            if any_ticket["job_id"] in self.assignments:
                # If job is done, release the robots.
                # Check the status of the last ticket in the job.
                job_status = get_job_last_ticket_status(job, self.all_tickets)
                if job_status == "Done":
                    self.release_assigned_robots(any_ticket["job_id"])
                    continue

                # Edited jobs.
                # Check the number of robots needed still matches assigned.
                self.update_existing_job_assigned_number(job)

                # Check that the assigned robots are still functional.
                # TODO.
            else:
                # Only try to assign tickets to the job if it hasn't been started yet.
                if not has_job_started(job, self.all_tickets):
                    unassigned_jobs.append(job)
                else:
                    print(f"Job {any_ticket['job_id']} has started.")

        # Attempt to assign robots to the new and unstarted jobs.
        # print(unassigned_jobs)
        self.new_job_robot_assignments(unassigned_jobs)

        # Print assignments and sets.
        for job_id, assignments in self.assignments.items():
            print(f"Job ID: {job_id}: {assignments}")
        print(f"Available: {self.available}")
        print(f"Occupied: {self.occupied}")
        print(f"Teams: {self.teams}")
        print()

    def release_assigned_robots(self, job_id: int):
        '''Releases the robots assigned to job_id.

        Args:
            job_id: id of the job whose assignments we are releasing.
        '''
        # Remove the robots from occupied and place them back in available.
        for robot in self.job_assigned_ids[job_id]:
            del(self.occupied[robot])
        self.available += self.job_assigned_ids[job_id].copy()

        # Remove the tickets from teams.
        for ticket_id, _ in self.assignments[job_id].items():
            del(self.teams[ticket_id])

        # Delete the job from assignments and job_assigned_ids.
        del(self.assignments[job_id])
        del(self.job_assigned_ids[job_id])

    def update_existing_job_assigned_number(self, job: list):
        '''Updates the number of robots in an existing job.

        Takes care of edits made to the number of robots needed for a job.
        If more robots are needed, it adds them to the relevant branches.
        If fewer robots are needed, it removes the difference from the
        relevant branches.

        Currently only implemented for if changes are made to top-level
        tickets.

        Args:
            job: list of tickets within that job.
        '''
        # TODO: Decide whether to allow changes to lower-level tickets.

        # Check how many robots were already assigned to the job.
        job_id = job[0]["job_id"]
        old_num_assigned = len(self.job_assigned_ids[job_id])

        # Get the start points of the job.
        start_points = get_all_job_start_points([job], self.all_tickets)

        # Count how many robots are now needed for the job.
        new_num_needed = 0
        for starter in start_points[job_id]:
            new_num_needed += self.all_tickets[starter]["num_robots"]

        # If the number needed is still correct, exit the function.
        # Otherwise, make the changes to the relevant branches.
        if old_num_assigned - new_num_needed == 0:
            return
        else:
            for starter in start_points[job_id]:
                old_num = len(self.assignments[job_id][starter])
                new_num = self.all_tickets[starter]["num_robots"]
                diff_num = old_num - new_num

                # Get the whole branch.
                linear_job = [starter]
                get_all_children_from_task_list(
                    starter, self.all_tickets, linear_job
                )

                if diff_num > 0:
                    # Remove the diff from the whole branch.
                    remove_ids = self.assignments[job_id][starter][:diff_num]

                    for ticket_id in linear_job:
                        for id in remove_ids:
                            self.assignments[job_id][ticket_id].remove(id)

                    # Remove the IDs from occupied and job_assigned_ids.
                    for id in remove_ids:
                        self.job_assigned_ids[job_id].remove(id)
                        del(self.occupied[id])

                    # Add the IDs back to available.
                    self.available += remove_ids.copy()
                elif diff_num < 0:
                    # Add up to the diff to the branch.
                    if diff_num <= len(self.available):
                        add_ids = self.available[:diff_num]
                    else:
                        add_ids = self.available[:]

                    for ticket_id in linear_job:
                        for id in add_ids:
                            self.assignments[job_id][ticket_id].append(id)

                    # Add the IDs to occupied and job_assigned_ids, and remove
                    # the IDs from available.
                    for id in add_ids:
                        self.job_assigned_ids[job_id].append(id)
                        self.occupied[id] = job_id
                        self.available.remove(id)

    def new_job_robot_assignments(self, job_list):
        '''Attmepts to assign robots to jobs if they are new or still ongoing.

        Args:
            job_list: list of jobs. Each job is list of tix [[{}, ...], ...].
        '''
        start_points = get_all_job_start_points(job_list, self.all_tickets)

        # Set of job IDs which we can't assign robots to in the first pass.
        second_pass_jobs = []
        second_pass_num_robots = []

        # Iterate through the list of starting points.
        for job_id, job_start_points in start_points.items():
            # Count how many robots are needed for the job.
            job_num_robots = 0

            for starter in job_start_points:
                job_num_robots += self.all_tickets[starter]["num_robots"]

            # print(f"Job ID: {job_id}. Start Points: {job_start_points}."
            #       f"Num robots: {job_num_robots}.")

            if job_num_robots <= len(self.available):
                self.assign_robots_to_job(job_id, job_start_points)
            else:
                # If we can't assign robots to the job, 
                # Append the number needed for the job.
                second_pass_jobs.append(job_id)
                second_pass_num_robots.append(job_num_robots)

        # Second pass. Attempt to assign the remaining robots to a job if
        # there are at least 75% of the job's required number.
        num_robots_left = len(self.available)
        percentages = []
        for idx in range(len(second_pass_jobs)):
            percentages.append(num_robots_left/second_pass_num_robots[idx])

        largest_percentage, largest_idx = 0, 0
        for idx in range(len(second_pass_jobs)):
            if percentages[idx] >= largest_percentage:
                largest_percentage = percentages[idx]
                largest_idx = idx
        # print(f"Largest percentage: {largest_percentage}.")
        if largest_percentage >= 0.75:
            job_id = second_pass_jobs[largest_idx]
            self.assign_robots_to_job(job_id, start_points[job_id])

    def assign_robots_to_job(self, job_id: int, job_start_points: list) -> None:
        '''Assigns robots to the specified job.

        Assigns the number of robots needed from the available set. Starts with
        the job's starting points, and assigns the same IDs to all the
        children. That way, the same number of robots is assigned at every
        level of the tree.

        Also assigns team IDs to each level of the job tree.

        Args:
            job_id:
            job_start_points:
        '''
        print(f"Assigning robots to {job_id}.")
        job_assignments = {}
        assigned_ids = []
        for start_point in job_start_points:
            # Get the number of robots needed for the start point.
            num_robots = self.all_tickets[start_point]["num_robots"]

            # Get num_robots from available set.
            # We might have fewer because of the 75% requirement.
            if num_robots <= len(self.available):
                assigned_robots = self.available[:num_robots]
            else:
                assigned_robots = self.available[:]

            # Transfer the robot IDs from available to occupied.
            for robot in assigned_robots:
                self.available.remove(robot)
                self.occupied[robot] = job_id

            # Get all the children of the current start point.
            linear_job = [start_point]
            get_all_children_from_task_list(
                start_point, self.all_tickets, linear_job
            )
            # print(f"Start {start_point}: {num_robots}. "
            #       f"Assigned: {assigned_robots}. "
            #       f"Job branch: {linear_job}.")

            # Create a team ID for the robots assigned to the branch.
            team_id = self.minTeamID
            self.minTeamID += 1

            # Add the assignments to each ticket in that branch.
            for ticket in linear_job:
                # Add the assignments to that ticket if it's already in 
                # the dictionary. Otherwise, create a new element.
                if ticket in job_assignments:
                    for robot in assigned_robots:
                        if robot not in job_assignments[ticket]:
                            job_assignments[ticket].append(robot)
                else:
                    job_assignments[ticket] = assigned_robots.copy()

                # Add the team ID to that ticket's list if it's already
                # in the dict. Otherwise, create a new entry.
                if ticket in self.teams:
                    self.teams[ticket].append(team_id)
                else:
                    self.teams[ticket] = [team_id]

            # Add the assignments to the assigned_ids.
            assigned_ids += assigned_robots.copy()

        # Add the job assignments to the assignments dictionary.
        self.assignments[job_id] = job_assignments
        self.job_assigned_ids[job_id] = assigned_ids

    def on_robot_dropout(self, msg) -> None:
        '''Remove the robot from assignments and attempt to assign a new one.'''
        remove_id = msg.robot_id
        
        if remove_id in self.available:
            self.available.remove(remove_id)
            return
        elif remove_id not in self.occupied:
            return
        job_id = self.occupied[remove_id]

        # Remove it from job_assigned_ids and occupied, but don't add
        # it to available.
        self.job_assigned_ids[job_id].remove(remove_id)
        del(self.occupied[remove_id])

        # Get a new robot from available if possible.
        add_id = None
        if len(self.available) > 0:
            add_id = self.available[0]
            self.available.remove(add_id)
            self.occupied[add_id] = job_id
            self.job_assigned_ids[job_id].append(add_id)

        # Remove the robot ID everywhere it was placed in that job.
        # If a new robot is available, put it in the same spots.
        for _, ticket_assignments in self.assignments[job_id].items():
            if remove_id in ticket_assignments:
                ticket_assignments.remove(remove_id)
            if add_id is not None:
                ticket_assignments.append(add_id)