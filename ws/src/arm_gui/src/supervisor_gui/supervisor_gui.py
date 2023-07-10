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
from PyQt5.QtWidgets import *
from rviz import bindings as rviz

from arm_msgs.msg import Ticket, Tickets
from gui_common.dialogs import ImportTicketsDialog, NewTicketDialog, EditTicketDialog, RemoveTicketDialog
from gui_common.map_viz import create_map_widget

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
        self.rviz_path = rospy.get_param(
            "gui_rviz_path", os.path.join(rviz_folder, "config.rviz")
        )

        # Set the central widget and window layout.
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.overallLayout = QHBoxLayout(self.centralWidget)
        self.centralWidget.setLayout(self.overallLayout)

        # Create the display and status bar.
        self.create_display()
        self.create_status_bar()

        self.minTicketNumber = 5

        # Publishers for ticket management. Add/edit/remove ticket/job.
        self.add_ticket_pub = rospy.Publisher(
            "add_ticket", Tickets, queue_size=10
        )
        self.end_ticket_pub = rospy.Publisher(
            "end_ticket", Ticket, queue_size=10
        )

        self.resize(1500, 1200)
        # 
        self.show()

    def shutdownGUI(self):
        '''Gracefully shutdown the GUI elements. Particularly RViz.'''
        # self.manager

    def create_display(self):
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
        mapSchedLayout.addWidget(self.canvas)
        tab1.setLayout(mapSchedLayout)

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
        importTixButton = QPushButton("Import Tickets")
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
            ticket.ticket_id = int(ticket_row[0])
            ticket.machine_type = int(ticket_row[1])
            ticket.parents = self.get_parents_list(ticket_row[2])
            ticket.duration = float(ticket_row[3])

            print(ticket)
            ticket_list.append(ticket)

        # Create the Tickets msg and publish it.
        msg = Tickets()
        msg.tickets = ticket_list
        self.add_ticket_pub.publish(msg)
        rospy.loginfo(f"{log_tag}: Published imported tickets.")
    
    def get_parents_list(self, parents_string: str):
        '''Gets a list of parents from a string.

        Returns an empty list if the string is empty.
        '''
        parents_string_list = parents_string.split(',')
        parents_list = []
        for a in parents_string_list:
            if a != "":
                parents_list.append(int(a))
        return parents_list

    def _createNewTicketDialog(self):
        '''Create a dialog for creating a new ticket.'''
        newTicketDialog = NewTicketDialog(self)
        newTicketDialog.setModal(True)
        newTicketDialog.show()

    def _createEditTicketDialog(self):
        '''Create a dialog for editing a target ticket.'''
        editTicketDialog = EditTicketDialog(self)
        editTicketDialog.setModal(True)
        editTicketDialog.show()

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
        self.canvas.draw()

    def create_schedule_canvas(self):
        '''.'''
        figure = plt.figure()
        self.canvas = FigureCanvas(figure)
        self.ax = figure.add_subplot(111)
        self._drawSchedule()

    def create_map_widget(self):
        '''.'''
        self.mapWidget, top_button, side_button = create_map_widget(self.rviz_path)
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
        

