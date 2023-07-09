#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Constructs and shows the Supervisor GUI using PyQt5.
'''
import json
import pandas as pd
import os
import sys

from PyQt5.QtWidgets import *

import rospy
import rospkg
from rviz import bindings as rviz

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

# from dialogs import NewTicketDialog, EditTicketDialog, RemoveTicketDialog

from arm_gui.scripts.dialogs import NewTicketDialog, EditTicketDialog, RemoveTicketDialog
from arm_gui.scripts.utils.display_utils import *
from arm_gui.scripts.utils.draw_utils import draw_tree_schedule
from arm_gui.scripts.utils.job_utils import *
from arm_gui.scripts.utils.sched_utils import *

# from utils.display_utils import *
# from utils.draw_utils import draw_tree_schedule
# from utils.job_utils import *
# from utils.sched_utils import *


# *****************************************************************************
schedule = pd.read_csv(f"CPSATScheduleD.csv", index_col=0)
# schedule.insert(0, 't', schedule.index)
# schedule.reset_index(drop=True, inplace=True)
# schedule.index.name = 't'
# print(schedule.head())

# Lists get converted to strings when saved. This converts the string back.
schedule["parents"] = schedule["parents"].apply(lambda x:json.loads(x))
schedule["start"] = schedule["start"].apply(lambda x:int(x))
schedule["end"] = schedule["end"].apply(lambda x:int(x))



class SupervisorGUI(QMainWindow):
    '''Supervisor GUI class.'''
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Supervisor GUI")

        
        rospy.init_node("supervisor_gui")
        # rate = rospy.Rate(1)
        rospy.loginfo("Initializing Supervisor GUI")
        rp = rospkg.RosPack()
        self.gui_path = rp.get_path("arm_gui")
        rospy.on_shutdown(self.shutdownGUI)

        # Set the central widget and window layout.
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.overallLayout = QHBoxLayout(self.centralWidget)
        self.centralWidget.setLayout(self.overallLayout)

        # Create the display and status bar.
        self._createDisplay()
        self._createStatusBar()

    def shutdownGUI(self):
        '''Gracefully shutdown the GUI elements. Particularly RViz.'''
        # self.manager

    def _createDisplay(self):
        '''Create the basic display.'''

        # Create the matplotlib canvas for the schedule and RViz widget.
        self._createScheduleCanvas()
        self._createMapWidget()

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

    def _createStatusBar(self):
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
        removeTicketButton = QPushButton("Remove Ticket")

        # Methods when each button is clicked.
        importTixButton.clicked.connect(self._createImportTixDialog)
        addTicketButton.clicked.connect(self._createNewTicketDialog)
        editTicketButton.clicked.connect(self._createEditTicketDialog)
        removeTicketButton.clicked.connect(self._createRemoveTicketDialog)

        buttonLayout.addWidget(importTixButton)
        buttonLayout.addWidget(addTicketButton)
        buttonLayout.addWidget(editTicketButton)
        buttonLayout.addWidget(removeTicketButton)

        # Add stretch, so buttons' positions don't change when resizing.
        buttonLayout.addStretch()

        return buttonLayout

    def _createImportTixDialog(self):
        '''Create a dialog for importing a set of new tickets.'''
        pass

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

    def _createScheduleCanvas(self):
        '''.'''
        figure = plt.figure()
        self.canvas = FigureCanvas(figure)
        self.ax = figure.add_subplot(111)
        self._drawSchedule()

    def _createMapWidget(self):
        '''.'''
        self.mapWidget = QWidget()

        # Create the RViz visualization frame.
        self.mapWidget.frame = rviz.VisualizationFrame()
        self.mapWidget.frame.setSplashPath("")
        self.mapWidget.frame.initialize()

        rviz_folder = os.path.join(self.gui_path, 'rviz')

        reader = rviz.YamlConfigReader()
        config = rviz.Config()

        reader.readFile(config, os.path.join(rviz_folder, "multi_dingo.rviz"))
        # reader.readFile(config, os.path.join(rviz_folder, "config.rviz"))
        # reader.readFile(config, os.path.join(rviz_folder, "tutorial.rviz"))
        self.mapWidget.frame.load(config)

        self.mapWidget.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        self.mapWidget.frame.setMenuBar( None )
        self.mapWidget.frame.setStatusBar( None )
        self.mapWidget.frame.setHideButtonVisibility( False )

        self.manager = self.mapWidget.frame.getManager()

        # self.mapWidget.setGeometry()
        # self.mapWidget.setFixedSize(500, 400)

        mapLayout = QVBoxLayout()
        mapLayout.addWidget(self.mapWidget.frame)

        h_layout = QHBoxLayout()

        top_button = QPushButton( "Top View" )
        top_button.clicked.connect( self._onTopButtonClick )
        h_layout.addWidget( top_button )

        side_button = QPushButton( "Side View" )
        side_button.clicked.connect( self._onSideButtonClick )
        h_layout.addWidget( side_button )
        mapLayout.addLayout( h_layout )

        self.mapWidget.setLayout(mapLayout)

    def _onTopButtonClick(self):
        '''.'''
        self.switchToView( "Top View" )

    def _onSideButtonClick(self):
        '''.'''
        self.switchToView( "Side View" )
        
    ## switchToView() works by looping over the views saved in the
    ## ViewManager and looking for one with a matching name.
    ##
    ## view_man.setCurrentFrom() takes the saved view
    ## instance and copies it to set the current view
    ## controller.
    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )
        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ww = SupervisorGUI()
    ww.show()
    sys.exit(app.exec())
