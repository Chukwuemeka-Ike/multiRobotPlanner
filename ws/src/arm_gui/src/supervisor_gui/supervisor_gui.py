#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    The Supervisor GUI class that runs using PyQt5.
'''
import json
import os
import pandas as pd

import rospy
import rospkg

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import *

from arm_constants.machines import machine_type_names
from arm_msgs.msg import Ticket, Tickets
from arm_msgs.srv import TicketList, TicketListRequest

# from arm_utils.display_utils import *
from arm_utils.conversion_utils import convert_ticket_list_to_task_dict,\
      convert_task_list_to_schedule, convert_task_list_to_job_list
from arm_utils.draw_utils import draw_tree_schedule
from arm_utils.job_utils import get_job_id_ticket_ids
from arm_utils.sched_utils import *

from gui_common.dialogs import ImportTicketsDialog, NewTicketDialog, EditTicketDialog, EditJobDialog
from gui_common.gui_elements import FixedWidthLabel
from gui_common import map_viz


# *****************************************************************************
schedule = pd.read_csv(f"~/arm/ws/src/arm_gui/src/gui_common/CPSATScheduleD.csv", index_col=0)
# schedule.insert(0, 't', schedule.index)
# schedule.reset_index(drop=True, inplace=True)
# schedule.index.name = 't'
# print(schedule.head())

# Lists get converted to strings when saved. This converts the string back.
schedule["parents"] = schedule["parents"].apply(lambda x:json.loads(x))
schedule["start"] = schedule["start"].apply(lambda x:int(x))
schedule["end"] = schedule["end"].apply(lambda x:int(x))


log_tag = "Supervisor GUI"


class SupervisorGUI(QMainWindow):
    '''Supervisor GUI class.'''

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Supervisor GUI")

        rospy.init_node("supervisor_gui")
        rospy.on_shutdown(self.shutdown_gui)
        rospy.loginfo(f"{log_tag}: Node started.")

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
        self.end_ticket_pub = rospy.Publisher(
            "end_ticket", Ticket, queue_size=10
        )
        self.delete_job_pub = rospy.Publisher(
            "delete_job", Ticket, queue_size=10
        )

        # Ticket list and subsets - waiting, ready, ongoing, done.
        self.all_tickets = {}
        self.waiting = []
        self.ready = []
        self.ongoing = []
        self.done = []

        # List of jobs, where each job is a list of tickets.
        # job_list: [[{}, ...], ...]
        self.job_list = []

        # Dictionary of each job's ticket IDs.
        # {job_id: [ticket_id, ...], ...}
        self.job_ticket_ids = {}

        # The minimum ticket ID that can be used.
        # Updated every time a ticket list is requested.
        self.min_ticket_id = 0

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

        # Request the task list once to populate the necessary variables.
        self.update_gui()

        self.show()

        # Set the update interval for the GUI in milliseconds.
        self.update_interval = 1000
        self.updateTimer = QTimer()
        self.updateTimer.timeout.connect(self.update_gui)
        self.updateTimer.setInterval(self.update_interval)
        self.updateTimer.start()

    def center_window(self):
        '''.'''
        # Get the screen geometry.
        screenGeometry = QDesktopWidget().screenGeometry()

        # Calculate the center position for the window.
        x = (screenGeometry.width() - self.width()) // 2
        y = (screenGeometry.height() - self.height()) // 2

        # Move the window to the center position.
        self.move(x, y)

    def show(self):
        '''Override the original show function.
        
        If the initial window size is larger than the screen,
        maximize the window instead.
        '''
        screenGeometry = QDesktopWidget().screenGeometry()
        if screenGeometry.width() <= self.width() or \
            screenGeometry.height() <= self.height():
            self.showMaximized()
        else:
            # Call the base class method.
            super().show()

    def update_gui(self):
        '''Operations to keep the GUI updated. Triggered by a QTimer.'''
        self.request_ticket_list()
        self.update_job_list_layout()
        self.update_schedule_display()

    def shutdown_gui(self):
        '''Announce GUI shutdown.'''
        rospy.loginfo(f"{log_tag}: Node shutdown.")

    def create_ui(self):
        '''Create the basic UI.'''

        # Create the matplotlib canvas for the schedule and the RViz widget.
        self.create_map_widget()
        self.create_schedule_canvas()

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
        mapSchedLayout = QVBoxLayout(self.tabs)
        mapSchedLayout.addWidget(self.mapWidget)
        mapSchedLayout.addWidget(self.scheduleCanvas)

        # Set the tabs' layouts.
        tab1.setLayout(mapSchedLayout)
        tab2.setLayout(self.jobScrollLayout)

        self.create_ticket_management_layout()

        # Add the tabs and ticket management layouts to the window's layout.
        self.overallLayout.addWidget(self.tabs)
        self.overallLayout.addWidget(self.ticketManagementGroupbox)

    def create_status_bar(self):
        '''Create a simple status bar.'''
        self.status_bar = QStatusBar(self.centralWidget)
        self.status_bar.showMessage("Supervisor GUI")
        self.setStatusBar(self.status_bar)

    def create_ticket_management_layout(self):
        '''Create the ticket management layout.'''
        buttonLayout = QVBoxLayout()

        self.ticketManagementGroupbox = QGroupBox("Ticket Management")
        
        # Buttons for bulk adding, adding, and editing tickets;
        # and editing jobs.
        importTixButton = QPushButton("Bulk Add Tickets")
        addTicketButton = QPushButton("Add Ticket")
        editTicketButton = QPushButton("Edit Ticket")
        editJobButton = QPushButton("Edit Job")

        # Methods when each button is clicked.
        importTixButton.clicked.connect(self._createImportTixDialog)
        addTicketButton.clicked.connect(self._createNewTicketDialog)
        editTicketButton.clicked.connect(self._createEditTicketDialog)
        editJobButton.clicked.connect(self._createEditJobDialog)

        # Add the buttons to the layout.
        buttonLayout.addWidget(importTixButton)
        buttonLayout.addWidget(addTicketButton)
        buttonLayout.addWidget(editTicketButton)
        buttonLayout.addWidget(editJobButton)

        # Add stretch, so buttons stay at the top when resizing.
        buttonLayout.addStretch()

        # Set the layout on the group box
        self.ticketManagementGroupbox.setLayout(buttonLayout)
        self.ticketManagementGroupbox.setCheckable(False)

    def create_job_list_layout(self):
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
        
        # # Layout for the column names.
        # labelWidth = self.tabs.width()//8 # Currently 7 columns.
        # print(f"Create width: {labelWidth}")

        # self.jobScrollLayout.addLayout(titleLayout)
        self.jobScrollLayout.addWidget(self.jobScrollArea)

        # # Update the job list and add the elements to the layout.
        # self.update_job_list_layout()

    def create_map_widget(self):
        '''Create the map widget for rviz visualization.'''
        self.mapWidget, top_button, side_button = map_viz.create_map_widget(
            self.rviz_path
        )
        top_button.clicked.connect(self._onTopButtonClick)
        side_button.clicked.connect(self._onSideButtonClick)
        self.mapManager = self.mapWidget.frame.getManager()

    def create_schedule_canvas(self):
        '''Creates the canvas we will draw the schedule on.'''
        figure = plt.figure()

        self.scheduleCanvas = FigureCanvas(figure)
        self.ax = figure.add_subplot(111)

        # Draw the placeholder schedule.
        self._drawSchedule(schedule)

    def _drawSchedule(self, schedule):
        '''.'''
        # Clear the old figure.
        self.ax.clear()

        draw_tree_schedule(schedule, self.ax)

        # Refresh the canvas.
        self.scheduleCanvas.draw()

    def _createImportTixDialog(self):
        '''Create a dialog for importing a set of new tickets.'''
        importTicketsDialog = ImportTicketsDialog(self, self.min_ticket_id)
        importTicketsDialog.dataEntered.connect(self._processImportedTickets)

    def _processImportedTickets(self, data):
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

            ticket_list.append(ticket)

        # Create the Tickets msg and publish it.
        msg = Tickets()
        msg.tickets = ticket_list
        self.add_ticket_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Imported new ticket(s).")

    def _createNewTicketDialog(self):
        '''Create a dialog for creating a new ticket.'''
        newTicketDialog = NewTicketDialog(
            self, self.min_ticket_id, self.job_ticket_ids
        )
        newTicketDialog.dataEntered.connect(self._processAddedTicket)

    def _processAddedTicket(self, data):
        '''Process the ticket that was added.'''
        ticket = Ticket()
        ticket.ticket_id = data[0]
        ticket.parents = data[1]
        ticket.duration = data[2]
        ticket.machine_type = data[3]

        # Create the Tickets() msg and publish it.
        msg = Tickets()
        msg.tickets = [ticket]
        self.add_ticket_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Added new ticket.")

    def _createEditTicketDialog(self):
        '''Create a dialog for editing a specific ticket.'''
        editTicketDialog = EditTicketDialog(
            self, self.job_ticket_ids, self.all_tickets, self.ongoing
        )
        editTicketDialog.dataEntered.connect(self._processEditedTicket)

    def _processEditedTicket(self, data):
        '''Process the edits made to the ticket.'''
        ticket = Ticket()
        ticket.ticket_id = data[0]
        ticket.duration = data[1]
        ticket.machine_type = data[2]

        self.edit_ticket_pub.publish(ticket)
        rospy.loginfo(f"{log_tag}: Published edited ticket.")

    def _createEditJobDialog(self):
        '''Create a dialog for editing a specific job.'''
        editJobDialog = EditJobDialog(
            self, self.job_ticket_ids, self.all_tickets, self.ongoing
        )
        editJobDialog.dataEntered.connect(self._processEditedJob)
        editJobDialog.deleteJobID.connect(self._processDeletedJob)

    def _processEditedJob(self, data):
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

            ticket_list.append(ticket)
            edited_ticket_ids.append(ticket_row[0])

        # Create the Tickets msg and publish it.
        msg = Tickets()
        msg.tickets = ticket_list
        self.edit_ticket_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Edited ticket(s) {edited_ticket_ids}.")

    def _processDeletedJob(self, deleted_job_id):
        '''Deletes the specified job.'''
        ticket_list = []
        deleted_ticket_ids = []
        for ticket_id in self.job_ticket_ids[deleted_job_id]:
            ticket = Ticket()
            ticket.ticket_id = ticket_id
            ticket.job_id = deleted_job_id

            ticket_list.append(ticket)
            deleted_ticket_ids.append(ticket_id)

        # Create the Tickets msg and publish it.
        msg = Tickets()
        msg.tickets = ticket_list
        self.delete_job_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Deleted job {deleted_job_id} with tickets "
                      f"{deleted_ticket_ids}."
        )

    def _onTopButtonClick(self):
        '''.'''
        self._switchToView("Top View")

    def _onSideButtonClick(self):
        '''.'''
        self._switchToView("Side View")

    def _switchToView(self, view_name):
        '''Looks for view_name in the saved views in the config.'''
        view_man = self.mapManager.getViewManager()
        for i in range(view_man.getNumViews()):
            if view_man.getViewAt(i).getName() == view_name:
                view_man.setCurrentFrom(view_man.getViewAt(i))
                return
        print(f"Could not find view named {view_name}")

    def request_ticket_list(self):
        '''Request the current ticket list from the ticket_service.'''
        # TODO: Waiting for service blocks the GUI from startup the way it's
        # currently set up. But waiting for service might be safer overall.
        # Make a decision.
        # rospy.wait_for_service('ticket_service')

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

    def update_job_list_layout(self):
        '''Update the list of jobs and their ticket states.'''
        # TODO: Fix the inconsistent labelWidth btw titleLayout and jobLayouts.

        # Clear the layout first.
        self.clear_layout(self.jobListLayout)
        labelWidth = self.tabs.width()//8 # Currently 7 columns.
        # print(f"Update width: {labelWidth}")
        titleLayout = QHBoxLayout()
        titleLayout.addWidget(FixedWidthLabel("Job", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("Ticket", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("Parents", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("Duration", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("Machine Type", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("Status", labelWidth))
        titleLayout.addWidget(FixedWidthLabel("Time Left", labelWidth))
        self.jobListLayout.addLayout(titleLayout)

        # Iterate through the jobs and display each in its own layout.
        for job in self.job_list:
            jobLayout = QHBoxLayout()

            # Job ID.
            jobIDLayout = QVBoxLayout()
            jobIDLayout.addWidget(FixedWidthLabel(f"{job[0]['job_id']}", labelWidth))
            jobIDLayout.addStretch()
            jobLayout.addLayout(jobIDLayout)

            # Ticket information.
            ticketListLayout = QVBoxLayout()
            for ticket in job:
                # Need a widget, so we can color it according to ticket status.
                ticketWidget = QWidget()

                # The layout holding the labels.
                ticketLayout = QHBoxLayout()
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['ticket_id']}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['parents']}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['duration']: .2f}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{machine_type_names[ticket['machine_type']]}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['status']}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['time_left']: .2f}", labelWidth))

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
            self.jobListLayout.addLayout(jobLayout)
        self.jobListLayout.addStretch()

    def clear_layout(self, layout):
        '''Clears the specified layout.

        Need to do so recursively to clean everything properly.
        '''
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
            elif isinstance(item, QLayout):
                self.clear_layout(item)
            else:
                layout.removeItem(item)

    def update_schedule_display(self):
        '''Update the schedule shown to the user.'''
        # print(self.all_tickets[38])
        schedule = convert_task_list_to_schedule(self.all_tickets, machine_type_names)
        # print(schedule)

        # Check if schedule
        if schedule is not None and schedule["end"].max() != 0:
            self._drawSchedule(schedule)