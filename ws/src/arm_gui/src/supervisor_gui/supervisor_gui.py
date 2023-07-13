#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    The Supervisor GUI class that runs using PyQt5.
'''
import json
import pandas as pd
import os
import sys
import rospy
import rospkg

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import *
from rviz import bindings as rviz

from arm_msgs.msg import Ticket, Tickets
from arm_msgs.srv import TicketList, TicketListRequest
from gui_common.dialogs import ImportTicketsDialog, NewTicketDialog, EditTicketDialog, RemoveTicketDialog
from gui_common.gui_elements import FixedWidthLabel
from gui_common import map_viz

from arm_constants.stations import *
from arm_utils.display_utils import *
from arm_utils.draw_utils import draw_tree_schedule
from arm_utils.job_utils import *
from arm_utils.sched_utils import *


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
        rospy.on_shutdown(self.shutdownGUI)
        rospy.loginfo(f"{log_tag}: Node started.")

        # RViz config location.
        rviz_folder = os.path.join(
            rospkg.RosPack().get_path("arm_gui"), 'rviz'
        )
        # self.rviz_path = rospy.get_param(
        #     "gui_rviz_path", os.path.join(rviz_folder, "config.rviz")
        # )
        self.rviz_path = os.path.join(rviz_folder, "world_one_robot.rviz")
        # self.rviz_path = os.path.join(rviz_folder, "world_multi_robot.rviz")

        # Publishers for ticket management. Add/edit/remove ticket/job.
        self.add_ticket_pub = rospy.Publisher(
            "add_ticket", Tickets, queue_size=10
        )
        self.edit_ticket_pub = rospy.Publisher(
            "edit_ticket", Ticket, queue_size=10
        )
        self.end_ticket_pub = rospy.Publisher(
            "end_ticket", Ticket, queue_size=10
        )

        # Ticket list and subsets - waiting, ready, ongoing, done.
        self.all_tickets = {}
        self.waiting = {}
        self.ready = {}
        self.ongoing = {}
        self.done = {}
        self.job_info = {}
        self.job_list = []
        self.minTicketNumber = 0

        # Set the central widget and window layout.
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.overallLayout = QHBoxLayout(self.centralWidget)
        self.centralWidget.setLayout(self.overallLayout)

        # Resize, center, and show the window.
        self.windowWidth = 1500
        self.windowLength = 1200
        self.resize(self.windowWidth, self.windowLength)
        self.center_window()

        # Create the display and status bar.
        self.create_ui()
        self.create_status_bar()
        self.update_gui()

        # Set the update interval for the GUI in milliseconds.
        self.updateInterval = 5000
        self.updateTimer = QTimer()
        self.updateTimer.timeout.connect(self.update_gui)
        self.updateTimer.setInterval(self.updateInterval)
        self.updateTimer.start()

        self.show()

    def center_window(self):
        # Get the screen geometry.
        screenGeometry = QDesktopWidget().screenGeometry()

        # Calculate the center position for the window.
        x = (screenGeometry.width() - self.width()) // 2
        y = (screenGeometry.height() - self.height()) // 2

        # Move the window to the center position.
        self.move(x, y)

    def shutdownGUI(self):
        '''Gracefully shutdown the GUI elements. Particularly RViz.'''
        # self.manager

    def create_ui(self):
        '''Create the basic display.'''

        # Create the matplotlib canvas for the schedule and RViz widget.
        self.create_schedule_canvas()
        self.create_map_widget()

        # Tabs for the left side of the screen. Buttons remain visible always.
        tabs = QTabWidget()
        tab1 = QWidget()
        tab2 = QWidget()
        tab3 = QWidget()

        tabs.addTab(tab1, "Map & Schedule")
        tabs.addTab(tab2, "Job List")
        tabs.addTab(tab3, "Full Schedule")

        # Vertical layout for the map and schedule.
        mapSchedLayout = QVBoxLayout(tabs)
        mapSchedLayout.addWidget(self.mapWidget)
        mapSchedLayout.addWidget(self.scheduleCanvas)
        tab1.setLayout(mapSchedLayout)

        # Create the job list display.
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

        # # Update the job list and add the elements to the layout.
        # self.update_job_list_layout()

        # Set the scroll layout to tab2.
        tab2.setLayout(self.jobScrollLayout)

        # Add the tabs and ticket management layout.
        self.overallLayout.addWidget(tabs)
        self.overallLayout.addLayout(self._createTicketManagementLayout())

    def create_status_bar(self):
        '''Create a simple status bar.'''
        status = QStatusBar(self.centralWidget)
        status.showMessage("Supervisor GUI")
        self.setStatusBar(status)

    def _createTicketManagementLayout(self):
        '''Create the ticket management layout.'''
        buttonLayout = QVBoxLayout()

        # Ticket Management label.
        buttonLayout.addWidget(QLabel("Ticket Management"))
        
        # Buttons for importing, adding, editing, and removing tickets.
        importTixButton = QPushButton("Bulk Add Tickets")
        addTicketButton = QPushButton("Add Ticket")
        editTicketButton = QPushButton("Edit Ticket")
        editJobButton = QPushButton("Edit Job")
        removeTicketButton = QPushButton("Remove Ticket")

        # Methods when each button is clicked.
        importTixButton.clicked.connect(self._createImportTixDialog)
        addTicketButton.clicked.connect(self._createNewTicketDialog)
        editTicketButton.clicked.connect(self._createEditTicketDialog)
        editJobButton.clicked.connect(self._createEditJobDialog)
        removeTicketButton.clicked.connect(self._createRemoveTicketDialog)

        buttonLayout.addWidget(importTixButton)
        buttonLayout.addWidget(addTicketButton)
        buttonLayout.addWidget(editTicketButton)
        buttonLayout.addWidget(editJobButton)
        buttonLayout.addWidget(removeTicketButton)

        # Add stretch, so buttons stay at the top when resizing.
        buttonLayout.addStretch()

        return buttonLayout

    def _createImportTixDialog(self):
        '''Create a dialog for importing a set of new tickets.'''
        importTicketsDialog = ImportTicketsDialog(self, self.minTicketNumber)
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

            # print(ticket)
            ticket_list.append(ticket)

        # Create the Tickets msg and publish it.
        msg = Tickets()
        msg.tickets = ticket_list
        self.add_ticket_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Published imported tickets.")

    def _createNewTicketDialog(self):
        '''Create a dialog for creating a new ticket.'''
        newTicketDialog = NewTicketDialog(self, self.minTicketNumber, self.job_info)
        newTicketDialog.dataEntered.connect(self._processAddedTicket)

    def _processAddedTicket(self, data):
        '''.'''
        ticket = Ticket()
        ticket.ticket_id = data[0]
        ticket.parents = data[1]
        ticket.duration = data[2]
        ticket.machine_type = data[3]

        # print(ticket)
        ticket_list = [ticket]
        # ticket_list.append(ticket)

        # Create the Tickets msg and publish it.
        msg = Tickets()
        msg.tickets = ticket_list
        self.add_ticket_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Published added ticket.")

    def _createEditTicketDialog(self):
        '''Create a dialog for editing a target ticket.'''
        editTicketDialog = EditTicketDialog(self, self.job_info, self.all_tickets, self.ongoing)
        editTicketDialog.dataEntered.connect(self._processEditedTicket)

    def _processEditedTicket(self, data):
        '''Processes the edits made to the ticket.'''
        ticket = Ticket()
        ticket.ticket_id = data[0]
        ticket.duration = data[1]
        ticket.machine_type = data[2]

        self.edit_ticket_pub.publish(ticket)
        rospy.loginfo(f"{log_tag}: Published edited ticket.")


    def _createEditJobDialog(self):
        '''Create a dialog for editing a target job.'''
        pass

    def _createRemoveTicketDialog(self):
        '''Create a dialog for removing a target ticket.'''
        removeTicketDialog = RemoveTicketDialog(self)
        removeTicketDialog.setModal(True)
        removeTicketDialog.show()

    def _drawSchedule(self):
        '''.'''
        # scheduleDialog = QDialog()
        # toolbar = NavigationToolbar(canvas, scheduleDialog)

        # clearing old figure
        self.ax.clear()

        # create an axis

        # # random data
        # # plot data
        # data = [random.random() for i in range(10)]
        # ax.plot(data, '*-')

        draw_tree_schedule(schedule, self.ax)
        # refresh canvas
        self.scheduleCanvas.draw()

    def create_schedule_canvas(self):
        '''.'''
        figure = plt.figure()
        self.scheduleCanvas = FigureCanvas(figure)
        self.ax = figure.add_subplot(111)
        self._drawSchedule()

    def create_map_widget(self):
        '''.'''
        self.mapWidget, top_button, side_button = map_viz.create_map_widget(self.rviz_path)
        top_button.clicked.connect(self._onTopButtonClick)
        side_button.clicked.connect(self._onSideButtonClick)
        self.manager = self.mapWidget.frame.getManager()

    def _onTopButtonClick(self):
        '''.'''
        self._switchToView("Top View")

    def _onSideButtonClick(self):
        '''.'''
        self._switchToView("Side View")

    def _switchToView(self, view_name):
        '''Looks for view_name in the saved views in the config.'''
        view_man = self.manager.getViewManager()
        for i in range(view_man.getNumViews()):
            if view_man.getViewAt(i).getName() == view_name:
                view_man.setCurrentFrom(view_man.getViewAt(i))
                return
        print(f"Could not find view named {view_name}")

    def request_ticket_list(self):
        '''Requests the current ticket list.'''
        # rospy.wait_for_service('ticket_service')
        try:
            request = TicketListRequest()
            ticket_list = rospy.ServiceProxy('ticket_service', TicketList)
            response = ticket_list(request)

            self.all_tickets = convert_ticket_list_to_task_dict(response.all_tickets)
            self.waiting = convert_ticket_list_to_task_dict(response.waiting)
            self.ready = convert_ticket_list_to_task_dict(response.ready)
            self.ongoing = convert_ticket_list_to_task_dict(response.ongoing)
            self.done = convert_ticket_list_to_task_dict(response.done)
            # print(self.all_tickets)

            if len(self.all_tickets) != 0 :
                self.minTicketNumber = max(self.all_tickets.keys())+1

            self.add_ticket_status()
            # Convert all_tickets to a list of lists, where each sub-list is a job.
            # Good for iterating.
            self.job_list = convert_task_list_to_job_list(self.all_tickets)
            self.job_info = get_job_id_ticket_ids(self.job_list)
        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Ticket list request failed: {e}.')

    def add_ticket_status(self):
        '''Adds a status key to all_tickets based on which subset they are in.'''
        for ticket_id, ticket in self.all_tickets.items():
            if ticket_id in self.waiting:
                ticket["status"] = "Waiting"
            elif ticket_id in self.ready:
                ticket["status"] = "Ready"
            elif ticket_id in self.ongoing:
                ticket["status"] = "Ongoing"
            elif ticket_id in self.done:
                ticket["status"] = "Done"

    def update_job_list_layout(self):
        '''Display the list of jobs with their tasks.'''
        
        # Clear the layout first.
        self.clear_layout(self.jobListLayout)
        labelWidth = self.width()//8

        titleLayout = QHBoxLayout()
        titleLayout.addWidget(FixedWidthLabel("Job"), labelWidth)
        titleLayout.addWidget(FixedWidthLabel("Ticket"), labelWidth)
        titleLayout.addWidget(FixedWidthLabel("Parents"), labelWidth)
        titleLayout.addWidget(FixedWidthLabel("Duration"), labelWidth)
        titleLayout.addWidget(FixedWidthLabel("Machine Type"), labelWidth)
        titleLayout.addWidget(FixedWidthLabel("Status"), labelWidth)
        titleLayout.addWidget(FixedWidthLabel("Time Left"), labelWidth)
        self.jobListLayout.addLayout(titleLayout)

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
                ticketWidget = QWidget()
                ticketLayout = QHBoxLayout()
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['ticket_id']}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['parents']}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['duration']: .2f}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{station_type_names[ticket['station_type']]}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['status']}", labelWidth))
                ticketLayout.addWidget(FixedWidthLabel(f"{ticket['time_left']: .2f}", labelWidth))

                ticketWidget.setLayout(ticketLayout)
                if ticket['status'] == "Ongoing":
                    ticketWidget.setStyleSheet("background-color: lightgreen;")
                elif ticket['status'] == "Done":
                    ticketWidget.setStyleSheet("background-color: lightblue;")

                ticketListLayout.addWidget(ticketWidget)
            ticketListLayout.addStretch()
            jobLayout.addLayout(ticketListLayout)
            self.jobListLayout.addLayout(jobLayout)
        self.jobListLayout.addStretch()

    def clear_layout(self, layout):
        '''Clears the specified layout.

        Need to do so recursively to clean everything properly.
        '''
        for i in reversed(range(layout.count())): 
            item = layout.itemAt(i)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
            elif isinstance(item, QVBoxLayout) or isinstance(item, QHBoxLayout):
                self.clear_layout(item)
            else:
                layout.removeItem(item)

    def update_gui(self):
        '''Operations to keep the GUI updated. Called from a ROS timer.'''
        self.request_ticket_list()
        self.update_job_list_layout()