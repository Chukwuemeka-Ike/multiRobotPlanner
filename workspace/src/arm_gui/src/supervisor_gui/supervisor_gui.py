#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Supervisor GUI class designed using PyQt5.
'''
import os
import pandas as pd
import rospy
import rospkg

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.backends.backend_qtagg import \
    NavigationToolbar2QT as NavigationToolbar

from typing import Tuple

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import *

from std_msgs.msg import UInt32

from arm_msgs.msg import Ticket, Tickets
from arm_msgs.srv import MachinesOverview, MachinesOverviewRequest,\
    RobotAssignments, RobotAssignmentsRequest,\
    TicketList, TicketListRequest, TicketLog, TicketLogRequest

from arm_utils.conversion_utils import convert_ticket_list_to_task_dict,\
        convert_task_list_to_schedule, convert_task_list_to_job_list,\
        convert_list_of_int_lists_to_list_of_lists
from arm_utils.draw_utils import draw_no_schedule, draw_schedule
from arm_utils.job_utils import get_job_id_ticket_ids, get_leaf_locations
from arm_utils.sched_utils import *

from gui_common.dialogs import ImportTicketsDialog, NewTicketDialog, EditTicketDialog, EditJobDialog
from gui_common.gui_elements import FixedWidthLabel, MapWidget, SaveOnlyNavigationToolbar
from gui_common.gui_utils import clear_layout, float_minutes_to_minutes_seconds


log_tag = "Supervisor GUI"


class SupervisorGUI(QMainWindow):
    '''Supervisor GUI class.'''

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Supervisor GUI")

        rospy.init_node("supervisor_gui")
        rospy.on_shutdown(self.shutdown_gui)
        rospy.loginfo(f"{log_tag}: Node started.")
        rospy.loginfo(f"{log_tag}: Waiting for services from: "
                      "Machine Manager, Robot Assigner, and Ticket Manager"
        )

        # RViz configuration location.
        rviz_folder = os.path.join(
            rospkg.RosPack().get_path("arm_gui"), 'rviz'
        )
        self.rviz_path = rospy.get_param(
            "gui_rviz_path", os.path.join(rviz_folder, "config.rviz")
        )

        # Publishers for ticket management. Add/edit/remove ticket/job.
        self.add_ticket_pub = rospy.Publisher(
            "add_ticket", Tickets, queue_size=10
        )
        self.edit_ticket_pub = rospy.Publisher(
            "edit_ticket", Tickets, queue_size=10
        )
        self.delete_job_pub = rospy.Publisher(
            "delete_job", UInt32, queue_size=10
        )

        # Machine overview information.
        # self.machine_type_names = machine_type_names
        self.machine_ids = []
        self.machine_type_names = []
        self.grouped_machine_ids = []
        self.machine_type_indices = []
        self.machine_type_abvs = []

        # Ticket dictionary and its subsets (waiting, ready, ongoing, done).
        self.all_tickets = {}
        self.waiting = []
        self.ready = []
        self.ongoing = []
        self.done = []
        self.ticket_log = {}

        # List of jobs, where each job is a list of tickets.
        # job_list: [[{}, ...], ...]
        self.job_list = []

        # Dictionary of each job's ticket IDs.
        # {job_id: [ticket_id, ...], ...}
        self.job_ticket_ids = {}

        # The minimum ticket ID that can be used.
        # Updated every time a ticket list is requested.
        self.min_ticket_id = 1

        # Robots assigned to every ticket
        self.robot_assignments = {}

        # Set the central widget and window layout.
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.overallLayout = QHBoxLayout(self.centralWidget)
        self.centralWidget.setLayout(self.overallLayout)

        # Resize, and center the window.
        self.resize(1500, 1200)
        self.center_window()

        # Create the ui and status bar.
        self.create_ui()
        self.create_status_bar()

        # Request info once to populate the necessary variables.
        self.update_gui()

        # Show the window.
        self.show()

        # Set the update interval for the GUI in milliseconds.
        self.update_interval = 1000
        self.updateTimer = QTimer()
        self.updateTimer.timeout.connect(self.update_gui)
        self.updateTimer.setInterval(self.update_interval)
        self.updateTimer.start()

    def center_window(self) -> None:
        '''Center the window on screen.'''
        # Get the screen geometry.
        screenGeometry = QDesktopWidget().screenGeometry()

        # Calculate the center position for the window.
        x = (screenGeometry.width() - self.width()) // 2
        y = (screenGeometry.height() - self.height()) // 2

        # Move the window to the center position.
        self.move(x, y)

    def show(self) -> None:
        '''Override the original show function.
        
        If the initial window size is larger than the screen,
        maximize the window instead.
        '''
        screenGeometry = QDesktopWidget().screenGeometry()
        if screenGeometry.width() <= self.width() or \
            screenGeometry.height() <= self.height():
            self.showMaximized()
        # Call the base class method.
        super().show()

    def update_gui(self) -> None:
        '''Operations to keep the GUI updated. Triggered by a QTimer.'''
        self.request_ticket_list()
        self.request_ticket_log()
        self.request_machine_overview()
        self.request_robot_assignments()
        self.update_job_list_layout()
        self.update_schedule_display()

    def shutdown_gui(self) -> None:
        '''Announce GUI shutdown.'''
        rospy.loginfo(f"{log_tag}: Node shutdown.")

    def create_ui(self) -> None:
        '''Create the basic UI.'''

        # Create the matplotlib canvases for the schedule and the RViz widget.
        self.mapWidget = MapWidget(self.rviz_path)
        self.miniScheduleCanvas, self.miniScheduleAxes = \
            self.create_schedule_canvas()
        self.fullScheduleCanvas, self.fullScheduleAxes = \
            self.create_schedule_canvas()
        NavigationToolbar

        # Tabs for going between map and schedule, job list, and full schedule.
        self.tabs = QTabWidget()
        tab1 = QWidget()
        tab2 = QWidget()
        tab3 = QWidget()

        self.tabs.addTab(tab1, "Map & Schedule")
        self.tabs.addTab(tab2, "Job List")
        self.tabs.addTab(tab3, "Full Schedule")

        # Create the job list layout.
        self.create_job_list_layout()

        # Vertical layout for the map and schedule.
        splitter = QSplitter()
        splitter.setOrientation(Qt.Vertical)
        splitter.addWidget(self.mapWidget)
        splitter.addWidget(self.miniScheduleCanvas)

        # Add widgets and spacer to the layout.
        mapSchedLayout = QVBoxLayout(tab2)
        mapSchedLayout.addWidget(splitter)

        fullMapLayout = QVBoxLayout(tab3)
        # # Full schedule needs a toolbar, so we can zoom in and out.
        # fullMapLayout.addWidget(NavigationToolbar(self.fullScheduleCanvas, tab3))
        fullMapLayout.addWidget(SaveOnlyNavigationToolbar(self.fullScheduleCanvas, tab3))
        fullMapLayout.addWidget(self.fullScheduleCanvas)

        # Set the tabs' layouts.
        tab1.setLayout(mapSchedLayout)
        tab2.setLayout(self.jobScrollLayout)
        tab3.setLayout(fullMapLayout)

        self.create_ticket_management_layout()

        # Add the tabs and ticket management layouts to the window's layout.
        self.overallLayout.addWidget(self.tabs)
        self.overallLayout.addWidget(self.ticketManagementGroupbox)

    def create_status_bar(self) -> None:
        '''Create a simple status bar.'''
        self.status_bar = QStatusBar(self.centralWidget)
        self.status_bar.showMessage("Supervisor GUI")
        self.setStatusBar(self.status_bar)

    def create_ticket_management_layout(self) -> None:
        '''Create the ticket management layout.'''
        buttonLayout = QVBoxLayout()

        self.ticketManagementGroupbox = QGroupBox("Ticket Management")

        # Buttons for bulk adding, single adding, and editing tickets;
        # and editing jobs.
        importTixButton = QPushButton("Bulk Add Tickets")
        addTicketButton = QPushButton("Add Ticket")
        editTicketButton = QPushButton("Edit Ticket")
        editJobButton = QPushButton("Edit Job")

        # Methods when each button is clicked.
        importTixButton.clicked.connect(self.create_bulk_add_tix_dialog)
        addTicketButton.clicked.connect(self.create_add_ticket_dialog)
        editTicketButton.clicked.connect(self.create_edit_ticket_dialog)
        editJobButton.clicked.connect(self.create_edit_job_dialog)

        # Add the buttons to the layout.
        buttonLayout.addWidget(importTixButton)
        buttonLayout.addWidget(addTicketButton)
        buttonLayout.addWidget(editTicketButton)
        buttonLayout.addWidget(editJobButton)

        # Add stretch, so buttons stay at the top when resizing.
        buttonLayout.addStretch()

        # Set the layout on the group box.
        self.ticketManagementGroupbox.setLayout(buttonLayout)
        self.ticketManagementGroupbox.setCheckable(False)

    def create_job_list_layout(self) -> None:
        '''Create the layout for the job list tab.'''
        # Create a layout to hold the scroll area.
        self.jobScrollLayout = QVBoxLayout()

        # Create the scroll area, scroll widget, and the job list layout.
        self.jobScrollArea = QScrollArea()
        self.jobScrollWidget = QWidget()
        self.jobListLayout = QVBoxLayout()

        self.jobScrollWidget.setLayout(self.jobListLayout)

        self.jobScrollArea.setWidget(self.jobScrollWidget)
        self.jobScrollArea.setWidgetResizable(True)
        self.jobScrollArea.setMaximumHeight(self.height())

        self.jobScrollLayout.addWidget(self.jobScrollArea)

    def create_schedule_canvas(self) -> Tuple[FigureCanvas, plt.Axes]:
        '''Creates the canvas for the schedule.'''
        # figure, ax = plt.subplots()
        # scheduleCanvas = FigureCanvas(figure)
        scheduleCanvas = FigureCanvas(plt.figure())
        ax = scheduleCanvas.figure.subplots()

        return scheduleCanvas, ax

    def draw_no_schedules(self, current_time: int) -> None:
        '''Draws empty graphs with the current time and windows.'''
        # Clear the old figures.
        self.miniScheduleAxes.clear()
        self.fullScheduleAxes.clear()

        draw_no_schedule(self.miniScheduleAxes, current_time)
        draw_no_schedule(self.fullScheduleAxes, current_time)

        # Refresh the canvases.
        self.miniScheduleCanvas.draw()
        self.fullScheduleCanvas.draw()

    def draw_schedules(self, schedule: pd.DataFrame, current_time: int) -> None:
        '''Draw the given schedule.'''
        # # Store the current view limits in case the user is interacting.
        # current_xlim = self.fullScheduleAxes.get_xlim()
        # current_ylim = self.fullScheduleAxes.get_ylim()

        # print(current_xlim)
        # print(current_ylim)

        # Clear the old figures.
        self.miniScheduleAxes.clear()
        self.fullScheduleAxes.clear()

        draw_schedule(
            schedule,
            self.ongoing,
            self.machine_type_indices,
            self.machine_type_abvs,
            self.miniScheduleAxes,
            current_time,
            False
        )

        draw_schedule(
            schedule,
            self.ongoing,
            self.machine_type_indices,
            self.machine_type_abvs,
            self.fullScheduleAxes,
            current_time,
            True
        )

        # Refresh the canvases.
        self.miniScheduleCanvas.draw()
        self.fullScheduleCanvas.draw()

        # self.fullScheduleAxes.set_xlim(current_xlim)
        # self.fullScheduleAxes.set_ylim(current_ylim)

    def create_bulk_add_tix_dialog(self) -> None:
        '''Create a dialog for importing a set of new tickets.'''
        importTicketsDialog = ImportTicketsDialog(
            self, self.min_ticket_id, self.machine_type_names
        )
        importTicketsDialog.dataEntered.connect(self.process_bulk_added_tix)

    def process_bulk_added_tix(self, data) -> None:
        '''Process the imported tickets.

        Gets the ticket IDs, machine types, parents, and durations, and
        publishes the ticket list for the ticket manager.
        '''
        ticket_list = []
        for ticket_row in data:
            ticket = Ticket()
            ticket.ticket_id = ticket_row[0]
            ticket.parents = ticket_row[1]
            ticket.duration = ticket_row[2]
            ticket.machine_type = ticket_row[3]

            if len(ticket.parents) == 0:
                ticket.num_robots = ticket_row[4]

            ticket_list.append(ticket)

        # Create the Tickets msg and publish it.
        msg = Tickets()
        msg.tickets = ticket_list
        self.add_ticket_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Imported new ticket(s).")

    def create_add_ticket_dialog(self) -> None:
        '''Create a dialog for creating a new ticket.'''
        newTicketDialog = NewTicketDialog(
            self, self.min_ticket_id, self.job_ticket_ids,
            self.machine_type_names
        )
        newTicketDialog.dataEntered.connect(self.process_added_ticket)

    def process_added_ticket(self, data) -> None:
        '''Process the ticket that was added.'''
        ticket = Ticket()
        ticket.ticket_id = data[0]
        ticket.parents = data[1]
        ticket.duration = data[2]
        ticket.machine_type = data[3]

        if len(ticket.parents) == 0:
            ticket.num_robots = data[4]

        # Create the Tickets() msg and publish it.
        msg = Tickets()
        msg.tickets = [ticket]
        self.add_ticket_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Added new ticket.")

    def create_edit_ticket_dialog(self) -> None:
        '''Create a dialog for editing a specific ticket.'''
        editTicketDialog = EditTicketDialog(
            self, self.job_ticket_ids, self.all_tickets, self.ongoing,
            self.machine_type_names
        )
        editTicketDialog.dataEntered.connect(self.process_edited_ticket)

    def process_edited_ticket(self, data) -> None:
        '''Process the edits made to the ticket.'''
        ticket = Ticket()
        ticket.ticket_id = data[0]
        ticket.duration = data[1]
        ticket.machine_type = data[2]

        if len(self.all_tickets[ticket.ticket_id]["parents"]) == 0:
            ticket.num_robots = data[3]

        # Create the Tickets msg and publish it.
        msg = Tickets()
        msg.tickets = [ticket]
        self.edit_ticket_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Published edited ticket.")

    def create_edit_job_dialog(self) -> None:
        '''Create a dialog for editing a specific job.'''
        editJobDialog = EditJobDialog(
            self, self.job_ticket_ids, self.all_tickets, self.ongoing,
            self.machine_type_names
        )
        editJobDialog.dataEntered.connect(self.process_edited_job)
        editJobDialog.deleteJobID.connect(self.process_deleted_job)

    def process_edited_job(self, data) -> None:
        '''Process the edits made to the job.

        Args:
            data - list of edited tickets. Each ticket contains the
                    ticket_id, parents, duration, and machine_type_num.
        '''
        ticket_list = []
        edited_ticket_ids = []
        for ticket_row in data:
            ticket = Ticket()
            ticket.ticket_id = ticket_row[0]
            ticket.parents = ticket_row[1]
            ticket.duration = ticket_row[2]
            ticket.machine_type = ticket_row[3]

            if len(ticket.parents) == 0:
                ticket.num_robots = ticket_row[4]

            ticket_list.append(ticket)
            edited_ticket_ids.append(ticket_row[0])

        # Create the Tickets msg and publish it.
        msg = Tickets()
        msg.tickets = ticket_list
        self.edit_ticket_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Edited ticket(s) {edited_ticket_ids}.")

    def process_deleted_job(self, deleted_job_id) -> None:
        '''Deletes the specified job.'''
        deleted_ticket_ids = []
        for ticket_id in self.job_ticket_ids[deleted_job_id]:
            ticket = Ticket()
            ticket.ticket_id = ticket_id
            ticket.job_id = deleted_job_id
            deleted_ticket_ids.append(ticket_id)

        # Create the Tickets msg and publish it.
        msg = UInt32()
        msg.data = deleted_job_id
        self.delete_job_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Deleted job {deleted_job_id} with tickets "
                      f"{deleted_ticket_ids}."
        )

    def request_ticket_list(self):
        '''Request the current ticket list from the ticket_service.'''
        rospy.wait_for_service('ticket_service', timeout=10)
        try:
            # TicketListRequest() is empty.
            request = TicketListRequest()
            ticket_list = rospy.ServiceProxy('ticket_service', TicketList)
            response = ticket_list(request)

            self.all_tickets = convert_ticket_list_to_task_dict(response.all_tickets)
            self.waiting = response.waiting
            self.ready = response.ready
            self.ongoing = response.ongoing
            self.done = response.done

            # Set the minimum ticket ID based on the response.
            if len(self.all_tickets) != 0 :
                self.min_ticket_id = max(self.all_tickets.keys())+1

            # Convert all_tickets to a list of lists, where each sub-list is a job.
            # Good for iterating when refreshing job list.
            self.job_list = convert_task_list_to_job_list(self.all_tickets)

            # Get each job's ticket IDs in a dict of lists.
            self.job_ticket_ids = get_job_id_ticket_ids(self.job_list)
        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Ticket list request failed: {e}.')

    def request_machine_overview(self):
        '''.'''
        rospy.wait_for_service('machine_overview_service', timeout=10)
        try:
            # MachinesOverviewRequest() is empty.
            request = MachinesOverviewRequest()
            machines_overview = rospy.ServiceProxy('machine_overview_service', MachinesOverview)
            response = machines_overview(request)
            self.machine_ids = response.machine_ids
            grouped_machine_ids = response.grouped_machine_ids
            self.grouped_machine_ids = \
                convert_list_of_int_lists_to_list_of_lists(grouped_machine_ids)
            self.machine_type_names = response.machine_type_names
            self.machine_type_indices = response.machine_type_indices
            self.machine_type_abvs = response.machine_type_abvs

            # print(f"IDs: {self.machine_ids}")
            # print(f"Grouped IDs: {self.grouped_machine_ids}")
            # print(f"Names: {self.machine_type_names}")

        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Ticket list request failed: {e}.')

    def request_assigned_robot_information(self, ticket_id: int) -> list:
        '''Requests info about the robots assigned to the ticket.'''
        rospy.wait_for_service('robot_assignments_service', timeout=10)
        try:
            request = RobotAssignmentsRequest()
            request.ticket_id = ticket_id
            robot_assignments = rospy.ServiceProxy(
                'robot_assignments_service',
                RobotAssignments
            )
            response = robot_assignments(request)
            assigned_robot_ids = response.assigned_robot_ids
            return list(assigned_robot_ids)
        except rospy.ServiceException as e:
            rospy.logerr(f"{log_tag}: Robot assignment request failed: {e}.")

    def request_robot_assignments(self):
        '''Requests the robot assignments for every ticket in all_tickets.'''
        for ticket_id, _ in self.all_tickets.items():
            assigned_robot_ids = \
                self.request_assigned_robot_information(ticket_id)
            self.robot_assignments[ticket_id] = assigned_robot_ids

    def request_ticket_log(self) -> None:
        '''.'''
        rospy.wait_for_service('ticket_log_service', timeout=10)
        try:
            request = TicketLogRequest()
            ticket_log = rospy.ServiceProxy('ticket_log_service', TicketLog)
            response = ticket_log(request)
            self.ticket_log = convert_ticket_list_to_task_dict(
                response.ticket_log
            )
        except rospy.ServiceException as e:
            print(f"{log_tag}: Ticket log request failed: {e}.")

    def update_job_list_layout(self):
        '''Update the list of jobs and their ticket states.'''
        # TODO: Fix the inconsistent labelWidth btw titleLayout and jobLayouts.

        # Clear the layout first.
        clear_layout(self.jobListLayout)
        labelWidth = self.tabs.width()//10 # Currently 9 columns.
        # print(f"Update width: {labelWidth}")

        titleLayout = QHBoxLayout()
        titleFrame = QFrame()
        titleFrame.setLayout(titleLayout)
        titleFrame.setFrameStyle(QFrame.Box | QFrame.Plain)
        titleFrame.setStyleSheet("background-color: darkgray;")
        titleLayout.addWidget(FixedWidthLabel("<h3>Job ID</h3>", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("<h3>Ticket ID</h3>", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("<h3>Parents</h3>", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("<h3>Duration</h3>", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("<h3>Machine Type</h3>", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("<h3>Number of Robots</h3>", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("<h3>Assigned Robots</h3>", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("<h3>Status</h3>", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("<h3>Time Left</h3>", labelWidth))
        # self.jobListLayout.addLayout(titleLayout)
        self.jobListLayout.addWidget(titleFrame)

        # Iterate through the jobs and display each in its own layout.
        for job in self.job_list:
            # Create the jobLayout and put a frame around it to give it a
            # border. Should make the list slightly easier to read.
            jobLayout = QHBoxLayout()
            jobFrame = QFrame()
            jobFrame.setLayout(jobLayout)
            jobFrame.setFrameStyle(QFrame.Box | QFrame.Plain)

            # Job ID.
            jobIDLayout = QVBoxLayout()
            jobIDLayout.addWidget(FixedWidthLabel(
                f"<h2>{job[0]['job_id']}</h2>", labelWidth)
            )
            jobIDLayout.addStretch()
            jobLayout.addLayout(jobIDLayout)
            # jobLayout.addWidget(FixedWidthLabel(
            #     f"<h2>{job[0]['job_id']}</h2>", labelWidth)
            # )

            # Ticket information.
            ticketListLayout = QVBoxLayout()
            for ticket in job:
                ticket_id = ticket['ticket_id']

                # Need a widget, so we can color it according to ticket status.
                ticketWidget = QWidget()

                # Number of robots needed for a ticket is the sum of all its
                # top-level parents' num_robots.
                top_level_parents = []
                get_leaf_locations(ticket_id, self.all_tickets, top_level_parents)
                num_robots_needed = sum(
                    self.all_tickets[id]["num_robots"] for id in top_level_parents
                )

                # The layout holding the labels.
                ticketLayout = QHBoxLayout()
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket_id}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{list(ticket['parents'])}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['duration']: .2f}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{self.machine_type_names[ticket['machine_type']]}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{num_robots_needed}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{self.robot_assignments[ticket_id]}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['status']}", labelWidth))
                # ticketLayout.addWidget(FixedWidthLabel(f"{ticket['time_left']: .2f}", labelWidth))
                time_left = float_minutes_to_minutes_seconds(ticket['time_left'])
                ticketLayout.addWidget(FixedWidthLabel(f"{time_left}", labelWidth))

                ticketWidget.setLayout(ticketLayout)

                # Set the color of the ticket based on its status.
                if ticket['status'] == "Ongoing":
                    ticketWidget.setStyleSheet("background-color: lightgreen;")
                elif ticket['status'] == "Done":
                    ticketWidget.setStyleSheet("background-color: lightblue;")

                # Add the ticket to the ticket list.
                ticketListLayout.addWidget(ticketWidget)
            ticketListLayout.addStretch()

            jobLayout.addLayout(ticketListLayout)
            self.jobListLayout.addWidget(jobFrame)
        self.jobListLayout.addStretch()

    def update_schedule_display(self):
        '''Update the schedule shown to the user.'''
        # TODO: This logic probably should be in Ticket Manager and SG
        # just requests an absolute ticket list over a service. This is
        # less work for crunch time dev.
        absolute_ticket_dict = {}
        current_time = int(rospy.get_time())
        for ticket_id, ticket in self.all_tickets.items():
            ticket_copy = ticket.copy()
            ticket_copy["start"] += current_time
            ticket_copy["end"] += current_time

            if ticket_id in self.ticket_log:
                ticket_copy["start"] = self.ticket_log[ticket_id]["start"]
                if ticket_id in self.done:
                    ticket_copy["end"] = self.ticket_log[ticket_id]["end"]
            absolute_ticket_dict[ticket_id] = ticket_copy.copy()

        schedule = convert_task_list_to_schedule(
            absolute_ticket_dict, self.machine_type_names
        )

        # Check if the schedule is real.
        if schedule is not None and schedule["end"].max() != 0:
            self.draw_schedules(schedule, current_time)
        else:
            self.draw_no_schedules(current_time)
