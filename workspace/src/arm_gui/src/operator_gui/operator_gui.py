#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Operator GUI class designed using PyQt5.
'''
import os
import rospy
import rospkg
import threading
from typing import Tuple

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import UInt32
from tf import TransformListener

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import *

from arm_msgs.msg import Ticket, TicketMotionParams
from arm_msgs.srv import FleetInformation, FleetInformationRequest,\
        MachineStatus, MachineStatusRequest,\
        RobotAssignments, RobotAssignmentsRequest,\
        RobotReplacement, RobotReplacementRequest,\
        TicketList, TicketListRequest,\
        UnboundMachines, UnboundMachinesRequest
from arm_utils.conversion_utils import convert_ticket_list_to_task_dict

from gui_common.dialogs import BasicConfirmDialog, TicketDetailsDialog
from gui_common.gui_elements import ToggleButton, FixedWidthLabel,\
        LEDIndicator, LEDManager, MapWidget, RobotButton, ServiceButton,\
        ToggleServiceButton
from gui_common.gui_utils import clear_layout, disable_layout, enable_layout


log_tag = "Operator GUI"

callback_lock = threading.Lock()
structureButtonFontSize = 20


class OperatorGUI(QMainWindow):
    '''Operator GUI class.'''

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Operator GUI")

        rospy.init_node("operator_gui")
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

        # Publishers for binding/releasing a machine to/from the GUI instance.
        self.bind_machine_pub = rospy.Publisher(
            "bind_machine", UInt32, queue_size=10
        )
        self.release_machine_pub = rospy.Publisher(
            "release_machine", UInt32, queue_size=10
        )

        # Publishers for starting and ending a ticket.
        self.start_ticket_pub = rospy.Publisher(
            "start_ticket", Ticket, queue_size=10
        )# Publisher for ending a ticket when done.
        self.end_ticket_pub = rospy.Publisher(
            "end_ticket", Ticket, queue_size=10
        )

        # Publisher for calling robots to the station.
        self.call_robots_pub = rospy.Publisher(
            "move_base_simple/goal", PoseStamped, queue_size=10
        )

        # Publisher for starting ticket-specific motion.
        self.start_task_motion_pub = rospy.Publisher(
            "path_csv_filename", TicketMotionParams, queue_size=10
        )

        # *********************************************************************
        # Swarm Version 1-inspired Code.
        # Control setup.
        self.tf = TransformListener()
        # self.tf_changer = None
        self.guiStartedUp = False

        self.get_control_params()

        self.team_buttons = []
        self.buttons = []
        self.labels = []
        self.leds = []

        self.synced_control_enabled = False
        self.rotation_disabled = False
        self.translation_disabled = False

        # Subscribe to the input command topic.
        rospy.Subscriber(self.input_command_topic, Twist, self.offset_callback)
        # *********************************************************************

        self.unbound_machines = []
        self.unbound_machine_names = []

        self.assigned_tickets = []
        self.ready_assigned_tickets = []
        self.machine_location = []

        # Current ticket ID.
        self.ticket_id = None
        self.is_ongoing = False
        self.machine_id = None
        self.machine_name = ""
        self.gui_is_bound = False

        # Set the central widget and window layout.
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.overallLayout = QVBoxLayout(self.centralWidget)
        self.centralWidget.setLayout(self.overallLayout)

        # Resize, and center the window.
        self.resize(600, 900)
        self.center_window()

        # Create the ui and status bar.
        self.create_ui()
        self.create_status_bar()

        # Manually making the fonts larger for the tablets' resolutions.
        self.setStyleSheet("""
                           QLabel{font-size: 15pt;}
                           QPushButton{font-size: 15pt;}
                           QComboBox{font-size: 15pt;}
                           QGroupBox{font-size: 15pt;}
                           QTextEdit{font-size: 15pt;}
                           """)

        # Request info once to populate the necessary variables.
        self.update_gui()

        # Show the window.
        # self.show()
        self.showMaximized()
        self.guiStartedUp = True

        # Set the update interval for the GUI in milliseconds.
        self.update_interval = 1000
        self.updateTimer = QTimer()
        self.updateTimer.timeout.connect(self.update_gui)
        self.updateTimer.setInterval(self.update_interval)
        self.updateTimer.start()

    def center_window(self):
        '''Center the window on the screen.'''
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
        # TODO.
        self.request_ticket_list()
        self.status_manager.poll_led_indicators()
        self.update_machine_dropdown()
        self.update_ticket_dropdown()

    def shutdown_gui(self):
        '''Gracefully shutdown the GUI elements.
        
        Particularly, we need to make sure the task is ended and the machine
        is released.
        '''
        # self.manager
        rospy.loginfo(f"{log_tag}: Node shutdown.")

        # If end button is enabled, task is still ongoing. End it and release
        # the machine.
        if self.guiStartedUp and self.endButton.isEnabled():
            self.release_machine_id()
            self.end_ticket()
        # If release button is enabled, a machine is still bound. Release it.
        elif self.guiStartedUp and self.machineIDReleaseButton.isEnabled():
            self.release_machine_id()

    def closeEvent(self, event):
        '''.'''
        if self.endButton.isEnabled():
            message = "A task is still ongoing. Please finish it before" +\
                " closing the GUI."
            QMessageBox.information(self, "Task in Progress", message)
            event.ignore()
        elif self.machineIDReleaseButton.isEnabled():
            message = "A machine is still bound to this operator GUI." +\
                "\nClosing will release the machine.\n\nClose the GUI?"
            # QMessageBox.information(self, "Task in Progress", message)
            popup_dialog = BasicConfirmDialog(
                self,
                "Release Bound Machine?",
                message
            )
            result = popup_dialog.exec_()
            if result == QDialog.Accepted:
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()

    def get_control_params(self):
        '''Get the ROS parameters for controlling robots when on a task.'''
        # Parameters saved in gui_params.yaml.
        self.resize_scaling_factor = float(rospy.get_param("resize_scaling_factor"))
        self.input_command_topic = rospy.get_param("input_command_topic")
        self.team_command_topic = rospy.get_param("team_command_topic")
        self.team_frame_command_topic = rospy.get_param("team_frame_command_topic")
        self.team_tf_frame_name = rospy.get_param("team_tf_frame_name")

        self.in_dev_mode = rospy.get_param("in_dev_mode", False)

        # These parameters are task-specific, so they're empty on startup.
        # They get updated whenever a task is started.
        self.num_assigned_robots = 0
        self.assigned_robot_ids = []
        self.assigned_robot_indices = []

        # Information about the robot fleet that does not change. Robot names,
        # command topics, tf frame names, enable status topic, and tf changer
        # topic. Will use ticket assigned robot IDs to index the lists.
        self.fleet_size = 0
        self.robot_names = []
        self.robot_command_topics = []
        self.robot_frame_command_topics = []
        self.real_robot_frame_names = []
        self.virtual_robot_frame_names = []
        self.tf_changer_topic = ""
        self.robot_enable_status_topic = ""

        # Populate the parameters.
        self.request_fleet_information()

    def create_ui(self):
        '''Create the basic UI.'''

        # Create the machine, ticket, and control layouts and RViz widget.
        self.create_machine_layout()
        self.create_ticket_layout()
        self.create_control_layout()
        self.mapWidget = MapWidget(self.rviz_path)
        self.mapLayout = QHBoxLayout()
        self.mapLayout.addWidget(self.mapWidget)

        # Frames are for placing borders around the layouts.
        mapFrame = QFrame()
        mapFrame.setLayout(self.mapLayout)
        mapFrame.setFrameStyle(QFrame.Box | QFrame.Plain)

        controlFrame = QFrame()
        controlFrame.setLayout(self.controlLayout)
        controlFrame.setFrameStyle(QFrame.Box | QFrame.Plain)

        # Splitter to allow the user resize the boundary between rviz
        # view and control layout.
        splitter = QSplitter()
        splitter.setOrientation(Qt.Vertical)
        splitter.addWidget(controlFrame)
        splitter.addWidget(mapFrame)

        self.overallLayout.addLayout(self.machineLayout)
        self.overallLayout.addLayout(self.ticketLayout)
        self.overallLayout.addWidget(splitter)

        # Disable the ticket and control layouts.
        disable_layout(self.ticketLayout)
        disable_layout(self.controlLayout)

    def create_status_bar(self):
        '''Create a simple status bar.'''
        status = QStatusBar(self.centralWidget)
        status.showMessage("Operator GUI")
        self.setStatusBar(status)

    def create_machine_layout(self):
        '''Machine layout allows the operator set which machine they're on.'''
        self.request_unbound_machines()

        self.machineLayout = QHBoxLayout()
        machineIDLabel = QLabel("Machine ID")
        machineIDLabel.setAlignment(Qt.AlignCenter)

        self.machineIDComboBox = QComboBox()
        self.update_machine_dropdown()
        self.machineIDComboBox.currentIndexChanged.connect(
            self.on_machine_dropdown_changed
        )

        self.machineIDSelectButton = QPushButton("Set Machine")
        self.machineIDSelectButton.setEnabled(False)
        self.machineIDSelectButton.clicked.connect(self.bind_machine_id)

        self.machineIDReleaseButton = QPushButton("Release Machine")
        self.machineIDReleaseButton.setStyleSheet("background-color : red")
        self.machineIDReleaseButton.setEnabled(False)
        self.machineIDReleaseButton.clicked.connect(self.release_machine_id)

        self.machineLayout.addWidget(machineIDLabel)
        self.machineLayout.addWidget(self.machineIDComboBox)
        self.machineLayout.addWidget(self.machineIDSelectButton)
        self.machineLayout.addWidget(self.machineIDReleaseButton)

    def create_ticket_layout(self):
        '''Layout for selecting, starting, and ending a ticket.'''
        self.ticketLayout = QHBoxLayout()
        self.ticketIDLabel = QLabel("Current Ticket")
        self.ticketIDLabel.setAlignment(Qt.AlignCenter)

        self.ticketIDComboBox = QComboBox()
        self.update_ticket_dropdown()
        self.ticketIDComboBox.currentIndexChanged.connect(
            self.on_ticket_dropdown_changed
        )

        self.detailsButton = QPushButton("Details")
        self.detailsButton.clicked.connect(self.display_ticket_details)

        self.startButton = QPushButton("Start")
        self.startButton.clicked.connect(self.start_ticket)

        self.endButton = QPushButton("End")
        self.endButton.setStyleSheet("background-color : green")
        self.endButton.clicked.connect(self.end_ticket)

        self.ticketLayout.addWidget(self.ticketIDLabel)
        self.ticketLayout.addWidget(self.ticketIDComboBox)
        self.ticketLayout.addWidget(self.detailsButton)
        self.ticketLayout.addWidget(self.startButton)
        self.ticketLayout.addWidget(self.endButton)

    def create_control_layout(self):
        '''Layout for working on a task with robots.'''
        self.controlLayout = QGridLayout()
        self.create_task_layout()
        self.create_robot_control_layout()

        controlLabel = QLabel("<h3>Robot Controls</h3>")
        controlLabel.setAlignment(Qt.AlignCenter)
        controlLabel.setFrameStyle(QFrame.Box | QFrame.Plain)
        controlLabel.setStyleSheet("background-color: darkgray;")

        self.controlLayout.addWidget(controlLabel, 0, 0, 1, 2)
        self.controlLayout.addLayout(self.taskLayout, 1, 0)
        self.controlLayout.addLayout(self.robotControlLayout, 1, 1)

    def create_task_layout(self):
        '''Layout for task motion controls and calling the robots.'''
        self.taskLayout = QVBoxLayout()

        # Task motion controls. Start, pause, adjust, and cancel.
        self.automatedMotionGroupBox = QGroupBox("Automated Motion")
        self.automatedMotionGroupBox.setCheckable(False)
        self.automatedMotionLayout = QGridLayout()

        # Button for calling robots to the workstation.
        self.callRobotsButton = QPushButton("Call Robots to Workstation")
        self.callRobotsButton.clicked.connect(self.call_robots_to_station)

        # Button for starting task-specific motion.
        self.startTaskMotion = QPushButton("Start Task Motion")
        self.startTaskMotion.clicked.connect(self.start_task_motion)

        # Buttons for pausing, continuing, cancelling, and adjusting
        # automated robot motions.
        self.enableMotionButton = ServiceButton(
            "Enable Execution", "enable_path_execution", log_tag
        )
        self.disableMotionButton = ServiceButton(
            "Disable Execution", "disable_path_execution", log_tag
        )
        self.cancelMotionButton = ServiceButton(
            "Cancel Execution", "cancel_path_execution", log_tag
        )
        self.adjustMotionButton = ToggleServiceButton(
            "Adjust Path", "toggle_adjust_path", log_tag
        )

        self.automatedMotionLayout.addWidget(self.callRobotsButton, 0, 0)
        self.automatedMotionLayout.addWidget(self.startTaskMotion, 0, 1)
        self.automatedMotionLayout.addWidget(self.enableMotionButton, 1, 0)
        self.automatedMotionLayout.addWidget(self.disableMotionButton, 1, 1)
        self.automatedMotionLayout.addWidget(self.cancelMotionButton, 2, 0)
        self.automatedMotionLayout.addWidget(self.adjustMotionButton, 2, 1)
        self.automatedMotionGroupBox.setLayout(self.automatedMotionLayout)

        # Robot replacement.
        self.replaceRobotGroupBox = QGroupBox("Robot Replacement")
        self.replaceRobotGroupBox.setCheckable(False)
        self.replaceRobotLayout = QHBoxLayout()

        self.replaceRobotButton = QPushButton("Request a Replacement")
        self.replaceRobotButton.clicked.connect(self.replace_robot)
        self.replaceRobotDropdown = QComboBox()
        self.replaceRobotDropdown.addItem("Select Robot ID")

        self.replaceRobotLayout.addWidget(self.replaceRobotDropdown)
        self.replaceRobotLayout.addWidget(self.replaceRobotButton)

        self.replaceRobotGroupBox.setLayout(self.replaceRobotLayout)

        self.taskLayout.addWidget(self.automatedMotionGroupBox)
        self.taskLayout.addWidget(self.replaceRobotGroupBox)
        self.taskLayout.addStretch()

    def create_robot_control_layout(self):
        '''Layout for manually controlling the robots.

        Contains buttons for enabling individual robots, frames, and their
        team as applicable. Also contains structure controls - expand, shrink,
        save, load. Sync frames.
        '''
        self.robotControlLayout = QVBoxLayout()

        structureSizeLayout = QHBoxLayout()
        self.shrinkButton = QPushButton("-")
        self.shrinkButton.setFont(QFont('Times', structureButtonFontSize))
        self.shrinkButton.clicked.connect(self.shrink_structure)

        self.expandButton = QPushButton("+")
        self.expandButton.setFont(QFont('Times', structureButtonFontSize))
        self.expandButton.clicked.connect(self.expand_structure)

        adjustStructure = QLabel("<h3>Adjust Structure Size</h3>")
        adjustStructure.setAlignment(Qt.AlignCenter)

        structureSizeLayout.addWidget(self.shrinkButton)
        structureSizeLayout.addWidget(adjustStructure)
        structureSizeLayout.addWidget(self.expandButton)

        # Version 2 no longer had a need for saving and loading team
        # structures. Keeping these commented out for the next person.
        # saveLoadLayout = QHBoxLayout()
        # self.saveStructureButton = QPushButton("Save Structure")
        # self.saveStructureButton.clicked.connect(self.save_structure)
        # self.loadStructureButton = QPushButton("Load Structure")
        # self.loadStructureButton.clicked.connect(self.load_structure)
        # saveLoadLayout.addWidget(self.saveStructureButton)
        # saveLoadLayout.addWidget(self.loadStructureButton)

        # Translation and rotation disable buttons.
        rotationTranslationLayout = QHBoxLayout()
        self.disableRotationButton = ToggleButton("Disable Rotation")
        self.disableRotationButton.clicked.connect(self.toggle_rotation)
        self.disableTranslationButton = ToggleButton("Disable Translation")
        self.disableTranslationButton.clicked.connect(self.toggle_translation)
        rotationTranslationLayout.addWidget(self.disableRotationButton)
        rotationTranslationLayout.addWidget(self.disableTranslationButton)

        # Center Team Frame and Sync Robot Frames buttons.
        syncCenterLayout = QHBoxLayout()
        self.centerTeamFrameButton = ServiceButton(
            "Center Team Frame", "send_swarm_frame_to_centroid", log_tag
        )
        self.syncFramesButton = QPushButton("Sync Robot Frames")
        self.syncFramesButton.clicked.connect(self.sync_frames)
        syncCenterLayout.addWidget(self.centerTeamFrameButton)
        syncCenterLayout.addWidget(self.syncFramesButton)

        # Team control buttons.
        self.teamLayout = QHBoxLayout()

        # Individual robot control buttons and labels.
        self.robotLabelLayout = QHBoxLayout()
        self.robotButtonLayout = QHBoxLayout()
        self.robotFrameButtonLayout = QHBoxLayout()
        self.robotLEDLayout = QHBoxLayout()

        self.update_robot_control_layout()

        # Add all the layouts to the control layout.
        self.robotControlLayout.addLayout(structureSizeLayout)
        # self.robotControlLayout.addLayout(saveLoadLayout)
        self.robotControlLayout.addLayout(rotationTranslationLayout)
        self.robotControlLayout.addLayout(syncCenterLayout)
        self.robotControlLayout.addLayout(self.teamLayout)
        self.robotControlLayout.addLayout(self.robotLabelLayout)
        self.robotControlLayout.addLayout(self.robotButtonLayout)
        self.robotControlLayout.addLayout(self.robotFrameButtonLayout)
        self.robotControlLayout.addLayout(self.robotLEDLayout)
        self.robotControlLayout.addStretch()

    def on_machine_dropdown_changed(self):
        '''Updates machine ID, name, and button when dropdown is updated.'''
        # Check the current dropdown text. If it's empty or no machine is
        # selected, disable the machine select button. If something is
        # selected, enable them.
        if self.machineIDComboBox.currentText() == "Select Machine ID" or\
            self.machineIDComboBox.currentText() == "":
            self.machineIDSelectButton.setEnabled(False)
        elif self.gui_is_bound == False:
            self.machineIDSelectButton.setEnabled(True)

        # If the current selection is in unbound machines, set the
        # machine id and name to that. Otherwise, if nothing is selected,
        # set both to None.
        if self.machineIDComboBox.currentText() in self.unbound_machine_names:
            self.machine_id = self.unbound_machines[
                    self.unbound_machine_names.index(
                        self.machineIDComboBox.currentText()
            )]
            self.machine_name = self.machineIDComboBox.currentText()
        # User selected the default choice and the dropdown has more
        # than 1 entry. Without this, the dropdown doesn't let the user select
        # default once they've chosen any other option.
        elif self.machineIDComboBox.currentText() == "Select Machine ID" and\
                self.machineIDComboBox.count() > 1:
            self.machine_id = None
            self.machine_name = None

    def update_machine_dropdown(self):
        '''Updates the machine ID dropdown.'''
        self.request_unbound_machines()

        self.machineIDComboBox.clear()
        self.machineIDComboBox.addItem("Select Machine ID")
        self.machineIDComboBox.addItems(self.unbound_machine_names)

        # Set the chosen machine ID. 
        # If our previous selection is in unbound, set it to that.
        # If the GUI is bound, the machine ID is not in
        # unbound, so add that name and set it manually.
        if self.machine_id in self.unbound_machines:
            self.machineIDComboBox.setCurrentIndex(
                self.unbound_machines.index(self.machine_id)+1
            )
        elif self.gui_is_bound == True:
            self.machineIDComboBox.addItem(self.machine_name)
            self.machineIDComboBox.setCurrentIndex(
                self.machineIDComboBox.count()-1
            )
            # self.machineIDSelectButton.setEnabled(False)

    def bind_machine_id(self):
        '''Binds the Operator GUI to the selected machine ID.

        Sends the machine ID message over the topic, which Machine Manager
        subscribes to.
        '''
        # Publish the message.
        msg = UInt32()
        msg.data = self.machine_id
        self.bind_machine_pub.publish(msg)

        # Set the GUI as bound.
        self.gui_is_bound = True
        # print(f"Binding {self.machine_name} with id {self.machine_id}.")

        # Disable select button and combo box, and enable release button.
        self.machineIDComboBox.setEnabled(False)
        self.machineIDSelectButton.setEnabled(False)
        self.machineIDReleaseButton.setEnabled(True)

        self.update_ticket_dropdown()

        # Enable the ticket label and dropdown.
        self.ticketIDLabel.setEnabled(True)
        self.ticketIDComboBox.setEnabled(True)

    def release_machine_id(self):
        '''Releases the machine ID from the Operator GUI.

        Sends the machine ID message over the topic, which Machine Manager
        subscribes to.
        '''
        # Publish the message.
        msg = UInt32()
        msg.data = self.machine_id
        self.release_machine_pub.publish(msg)

        # Set the .
        self.gui_is_bound = False
        # print(f"Releasing {self.machine_name} with id {self.machine_id}.")

        # Enable select button and combo box, and disable release button.
        self.update_machine_dropdown()
        self.machineIDComboBox.setEnabled(True)
        self.machineIDSelectButton.setEnabled(False)
        self.machineIDReleaseButton.setEnabled(False)

        # Empty the ticket combo box and the assigned ticket list.
        self.update_ticket_dropdown()

        # Disable the ticket and buttons layouts.
        disable_layout(self.ticketLayout)
        disable_layout(self.controlLayout)

    def request_unbound_machines(self):
        '''Requests the machines that are not already bound to a GUI.'''
        rospy.wait_for_service('unbound_machine_service', timeout=10)
        try:
            # UnboundMachinesRequest() is empty.
            request = UnboundMachinesRequest()
            unbound_machines = rospy.ServiceProxy('unbound_machine_service', UnboundMachines)
            response = unbound_machines(request)

            self.unbound_machines = response.machine_ids
            self.unbound_machine_names = response.machine_names
        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Unbound machines request failed: {e}.')

    def request_machine_assigned_tickets(self):
        '''Requests the tickets assigned to the selected machine ID.'''
        rospy.wait_for_service('machine_status_service', timeout=10)
        try:
            request = MachineStatusRequest()
            request.machine_id = self.machine_id
            machine_status = rospy.ServiceProxy(
                'machine_status_service',
                MachineStatus
            )
            response = machine_status(request)

            self.assigned_tickets = response.assigned_ids
            self.ready_assigned_tickets = response.ready_assigned_ids
            self.machine_status = response.status
            self.machine_location = response.machine_location
            self.needle_location = response.needle_location
        except rospy.ServiceException as e:
            rospy.logerr(f"{log_tag}: Machine assigned tickets "
                         f"request failed: {e}."
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
            # self.waiting = response.waiting
            # self.ready = response.ready
            # self.ongoing = response.ongoing
            # self.done = response.done
        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Ticket list request failed: {e}.')

    def request_fleet_information(self):
        '''Requests the information about all robots in the fleet.

        The information includes the total number of robots in the fleet,
        robot names, command topics, and frame names.
        '''
        rospy.wait_for_service('fleet_information_service', timeout=10)
        try:
            request = FleetInformationRequest()
            fleet_information = rospy.ServiceProxy(
                "fleet_information_service",
                FleetInformation
            )
            response = fleet_information(request)

            self.fleet_size = response.fleet_size
            self.robot_names = response.robot_names
            self.robot_command_topics = response.robot_command_topics
            self.robot_frame_command_topics = response.robot_frame_command_topics
            self.real_robot_frame_names = response.real_robot_frame_names
            self.virtual_robot_frame_names = response.virtual_robot_frame_names
            # self.state_publish_topics = response.robot_desired_state_topics

            self.tf_changer_topic = response.tf_changer_topic
            self.tf_changer = rospy.Publisher(
                self.tf_changer_topic, PoseStamped, queue_size=10
            )
            self.robot_enable_status_topic = response.robot_enable_status_topic
        except rospy.ServiceException as e:
            rospy.logerr(f"{log_tag}: Fleet information request failed: {e}.")

    def request_assigned_robot_information(self):
        '''Requests info about the robots assigned to the selected ticket.'''
        rospy.wait_for_service('robot_assignments_service', timeout=10)
        try:
            request = RobotAssignmentsRequest()
            request.ticket_id = self.ticket_id
            robot_assignments = rospy.ServiceProxy(
                'robot_assignments_service',
                RobotAssignments
            )
            response = robot_assignments(request)

            self.num_assigned_robots = response.num_assigned_robots
            self.assigned_robot_ids = response.assigned_robot_ids

            # Robot IDs start at 1, so subtract 1 to get the indices for
            # accessing their info from the robot info lists.
            self.assigned_robot_indices = [id-1 for id in self.assigned_robot_ids]
        except rospy.ServiceException as e:
            rospy.logerr(f"{log_tag}: Robot assignment request failed: {e}.")

    def request_robot_replacement(self, robot_id: int) -> Tuple[int, bool]:
        '''Requests a replacement for a specific robot.

        Args:
            robot_id: ID of the robot to replace.
        Returns:
            replacement_id: ID of the replacement robot. -1 if no
                    replacement was given.
            replacement_successful: whether or not the replacement
                    was successful.
        '''
        rospy.wait_for_service('robot_replacement_service', timeout=10)
        try:
            request = RobotReplacementRequest()
            request.robot_id = robot_id
            robot_replacement = rospy.ServiceProxy(
                'robot_replacement_service',
                RobotReplacement
            )
            response = robot_replacement(request)
            return response.replacement_id, response.replacement_successful
        except rospy.ServiceException as e:
            rospy.logerr(f"{log_tag}: Robot replacement request failed: {e}.")
            return -1, False

    def replace_robot(self) -> None:
        '''Replaces the selected robot when the button is pressed.'''
        # If no options or no robot chosen, return.
        if self.replaceRobotDropdown.currentText() == "Select Robot ID" or\
            self.replaceRobotDropdown.currentText() == "":
            return

        # Get the remove ID.
        remove_id = int(self.replaceRobotDropdown.currentText())

        # Check if they are sure they want to replace the robot.
        message = f"Attempting to replace robot {remove_id}. " +\
                "Continuing will remove the robot from the team and the\n" +\
                "system will attempt to assign a replacement." +\
                "\n\nWould you like to continue with replacement?"
        popup_dialog = BasicConfirmDialog(
            self,
            "Replace robot?",
            message
        )
        result = popup_dialog.exec_()
        if result == QDialog.Rejected:
            return

        # Disable all robots in case they're executing some motion.
        for led in self.leds:
            led.active = False
            led.setChecked(led.active)
        self.status_manager.publish_enable_status()

        # Request a replacement from the Robot Assigner.
        replacement_id, replacement_successful = \
            self.request_robot_replacement(
                remove_id
            )

        # Update the robot control buttons. Right now, just clearing
        # and repopulating. TODO: Think about doing this better.
        self.clear_robot_control_layout()
        self.request_assigned_robot_information()
        self.update_robot_control_layout()

        if replacement_successful == True:
            # Show a message that we got a replacement.
            message = f"Successfully replaced robot {remove_id} " +\
                f"with robot {replacement_id}. " +\
                f"Enable only the replacement and click 'Call for Robots' " +\
                "to call it to your workstation."
            QMessageBox.information(self, "Replacement Successful", message)
        else:
            # Show a message that we got no replacement.
            message = f"Could not replace robot {remove_id}, but it has " +\
                "been removed from the team and can be moved out of the way."
            QMessageBox.information(self, "No Replacement Available", message)
            return

    def on_ticket_dropdown_changed(self):
        '''Updates the ticket buttons when the dropdown is updated.'''
        # Check the current dropdown text. If empty or no ticket is selected,
        # disable the ticket start detail and start buttons. If something is
        # selected, enable them.
        if self.ticketIDComboBox.currentText() == "Select Ticket ID" or\
                self.ticketIDComboBox.currentText() == "":
            self.startButton.setEnabled(False)
            self.detailsButton.setEnabled(False)
            selected_ticket = False
        else:
            self.startButton.setEnabled(True)
            self.detailsButton.setEnabled(True)
            selected_ticket = True

        if self.ticketIDComboBox.currentText() == "Select Ticket ID" and\
                self.ticketIDComboBox.count() > 1:
            self.ticket_id = None
        elif selected_ticket == True and\
                int(self.ticketIDComboBox.currentText()) in\
                    self.ready_assigned_tickets:
            self.ticket_id = int(
                self.ticketIDComboBox.currentText()
            )

    def update_ticket_dropdown(self):
        '''Updates the ticket ID dropdown.'''
        # Empty the ticket combo box, then add the IDs after requesting
        # a new ticket list and machine assignments.
        self.ticketIDComboBox.clear()
        self.ticketIDComboBox.addItem("Select Ticket ID")

        # If no machine is selected, exit after adding the top text.
        if self.gui_is_bound == False:
            return

        self.request_machine_assigned_tickets()

        # Add the ready assigned tickets.
        self.ticketIDComboBox.addItems([
            str(id) for id in self.ready_assigned_tickets
        ])

        # Set the chosen ticket ID.
        # If previous selection is in ready, set it to that.
        # If the ticket is ongoing, the ID is not in ready, so
        # add it and set it manually.
        if self.ticket_id in self.ready_assigned_tickets:
            self.ticketIDComboBox.setCurrentIndex(
                self.ready_assigned_tickets.index(self.ticket_id)+1
            )
        elif self.is_ongoing == True:
            self.ticketIDComboBox.addItem(str(self.ticket_id))
            self.ticketIDComboBox.setCurrentIndex(
                self.ticketIDComboBox.count()-1
            )
            self.startButton.setEnabled(False)

    def display_ticket_details(self):
        '''Displays the ticket details.'''
        TicketDetailsDialog(
            self, self.all_tickets[self.ticket_id], self.machine_name
        )

    def start_ticket(self):
        '''Starts the selected ticket and publishes its ID.'''
        # Publish the message.
        msg = Ticket()
        msg.ticket_id = self.ticket_id

        # Request the assigned robot information.
        self.request_assigned_robot_information()

        # If there are no assigned robots and it's a top-level ticket, ask the
        # operator if they want to work on the task manually.
        # If they choose no, exit the function.
        if self.num_assigned_robots == 0 and \
            len(self.all_tickets[self.ticket_id]["parents"]) == 0:
            message = "There are no robots assigned to this ticket." +\
                "\nWould you like to continue manually?"
            popup_dialog = BasicConfirmDialog(
                self,
                "Conduct task manually?",
                message
            )
            result = popup_dialog.exec_()
            if result == QDialog.Rejected:
                return

        self.start_ticket_pub.publish(msg)
        self.is_ongoing = True

        # Disable the release machine button and the ID dropdown.
        self.ticketIDComboBox.setEnabled(False)
        self.machineIDReleaseButton.setEnabled(False)
        self.startButton.setEnabled(False)
        self.endButton.setEnabled(True)

        if self.num_assigned_robots != 0:
            enable_layout(self.controlLayout)
            self.update_robot_control_layout()

    def end_ticket(self):
        '''Ends the selected ticket and publishes its ID.'''
        # Publish the message.
        msg = Ticket()
        msg.ticket_id = self.ticket_id
        self.end_ticket_pub.publish(msg)

        self.is_ongoing = False

        # Disable all robots before clearing the robot controls.
        for led in self.leds:
            led.active = False
            led.setChecked(led.active)
        self.status_manager.publish_enable_status()

        # Clear the robot control layout.
        self.clear_robot_control_layout()

        # Enable the release machine button and the ID dropdown.
        self.ticketIDComboBox.setEnabled(True)
        self.machineIDReleaseButton.setEnabled(True)
        self.startButton.setEnabled(True)
        self.endButton.setEnabled(False)
        disable_layout(self.controlLayout)

        self.update_ticket_dropdown()

    def call_robots_to_station(self):
        '''Calls the robots to the station when clicked.'''
        msg = PoseStamped()

        # Header and map frame.
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        # Get the machine's location.
        msg.pose.position.x = self.machine_location[0]
        msg.pose.position.y = self.machine_location[1]
        # msg.pose.orientation. = self.machine_location[1]

        print(msg)

        self.call_robots_pub.publish(msg)

    def start_task_motion(self) -> None:
        '''Sends a TicketMotionParams message.'''
        msg = TicketMotionParams()
        msg.path_csv_filename = f"ticket_{self.ticket_id}.csv"
        msg.machine_location = self.machine_location
        msg.needle_location = self.needle_location
        self.start_task_motion_pub.publish(msg)

    def clear_robot_control_layout(self):
        '''Clears the robot-specific control layout and parameters.'''
        # Empty the parameters to ensure the GUI doesn't maintain control
        # of robots it shouldn't.
        self.num_assigned_robots = 0
        self.assigned_robot_ids = []
        self.assigned_robot_indices = []

        # Unregister the publishers for the buttons.
        for button in self.team_buttons:
            button.publisher.unregister()
        for button in self.buttons:
            button.publisher.unregister()
        
        # Unregister the status manager's publisher.
        self.status_manager.publisher.unregister()

        # Clear the robot replacement dropdown and add the placeholder.
        self.replaceRobotDropdown.clear()
        self.replaceRobotDropdown.addItem("Select Robot ID")

        # Clear the 5 layouts.
        clear_layout(self.teamLayout)
        clear_layout(self.robotLabelLayout)
        clear_layout(self.robotButtonLayout)
        clear_layout(self.robotFrameButtonLayout)
        clear_layout(self.robotLEDLayout)

        # Update the layouts. With empty parameters, this empties the button,
        # led, and label lists.
        self.update_robot_control_layout()

    def update_robot_control_layout(self):
        '''Updates the robot control layout.

        Called whenever a task with robots is started to set the correct
        number of robot buttons with their topics.
        '''
        self.team_buttons = []
        self.buttons = []
        self.labels = []
        self.leds = []

        if self.num_assigned_robots != 0:
            self.moveTeamButton = RobotButton("Team", self.team_command_topic)
            self.moveTeamFrameButton = RobotButton(
                "Team Frame",
                self.team_frame_command_topic
            )
            self.team_buttons.append(self.moveTeamButton)
            self.team_buttons.append(self.moveTeamFrameButton)
            self.teamLayout.addWidget(self.moveTeamButton)
            self.teamLayout.addWidget(self.moveTeamFrameButton)

        for idx in range(self.num_assigned_robots):
            # Get the robot's index, so we select the correct topics and names.
            robot_idx = self.assigned_robot_indices[idx]

            # Create the LED Indicator with the robot's ID.
            led = LEDIndicator(self.assigned_robot_ids[idx])
            self.robotLEDLayout.addWidget(led)
            self.leds.append(led)

            label = QLabel(self.robot_names[robot_idx])
            label.setAlignment(Qt.AlignCenter)
            self.robotLabelLayout.addWidget(label)
            self.labels.append(label)

            button = RobotButton(
                self.robot_names[robot_idx], self.robot_command_topics[robot_idx]
            )
            self.robotButtonLayout.addWidget(button)
            self.buttons.append(button)
            )

            # Individual robot frame control only when in dev mode.
            # It's just more to confuse the users.
            if self.in_dev_mode:
                button = RobotButton(
                    self.robot_names[robot_idx] + " Frame",
                    self.robot_frame_command_topics[robot_idx]
                )
                self.robotFrameButtonLayout.addWidget(button)
                self.buttons.append(button)

            # Add the robot ID to the replace dropdown.
            self.replaceRobotDropdown.addItem(str(self.assigned_robot_ids[idx]))

        self.status_manager = LEDManager(
            self.leds, self.robot_enable_status_topic
        )

    def toggle_rotation(self):
        '''.'''
        self.rotation_disabled = not(self.rotation_disabled)

    def toggle_translation(self):
        '''.'''
        self.translation_disabled = not(self.translation_disabled)

    def offset_callback(self, msg):
        '''Alters the received input command, then publishes to enabled bots.'''
        with callback_lock:
            if(self.rotation_disabled):
                msg.angular.x = msg.angular.x*0.
                msg.angular.y = msg.angular.y*0.
                msg.angular.z = msg.angular.z*0.
            if(self.translation_disabled):
                msg.linear.x = msg.linear.x*0.
                msg.linear.y = msg.linear.y*0.
                msg.linear.z = msg.linear.z*0.

            for i in range(len(self.team_buttons)):
                if(self.team_buttons[i].enabled):
                    self.team_buttons[i].publisher.publish(msg)

            for i in range(len(self.buttons)):
                if(self.buttons[i].enabled):
                    self.buttons[i].publisher.publish(msg)

    def sync_frames(self):
        '''Copies real robot frame to virtual robot frame.

        Finds the transform from the team frame to the real robot frame, then
        copies that to the virtual robot frame.
        '''
        for idx in range(self.num_assigned_robots):
            # Get the robot's index, so we select the correct topics and names.
            robot_idx = self.assigned_robot_indices[idx]

            if self.tf.frameExists(self.team_tf_frame_name) and\
                self.tf.frameExists(self.real_robot_frame_names[robot_idx]):

                t = self.tf.getLatestCommonTime(
                    self.real_robot_frame_names[robot_idx],
                    self.team_tf_frame_name
                )
                trans, quaternions = self.tf.lookupTransform(
                    self.team_tf_frame_name,
                    self.real_robot_frame_names[robot_idx],
                    t
                )

                poseMsg = PoseStamped()
                poseMsg.header.frame_id = self.virtual_robot_frame_names[robot_idx]

                poseMsg.pose.position.x = float(trans[0])
                poseMsg.pose.position.y = float(trans[1])
                poseMsg.pose.position.z = float(trans[2])
                poseMsg.pose.orientation.w = float(quaternions[3])
                poseMsg.pose.orientation.x = float(quaternions[0])
                poseMsg.pose.orientation.y = float(quaternions[1])
                poseMsg.pose.orientation.z = float(quaternions[2])

                self.tf_changer.publish(poseMsg)

    def expand_structure(self):
        '''Expand the team's structure by a scaling factor.'''
        for idx in range(self.num_assigned_robots):
            # Get the robot's index, so we select the correct topics and names.
            robot_idx = self.assigned_robot_indices[idx]

            if self.tf.frameExists(self.team_tf_frame_name) and\
                self.tf.frameExists(self.virtual_robot_frame_names[robot_idx]):
                t = self.tf.getLatestCommonTime(
                    self.virtual_robot_frame_names[robot_idx],
                    self.team_tf_frame_name
                )
                trans, quaternions = self.tf.lookupTransform(
                    self.team_tf_frame_name,
                    self.virtual_robot_frame_names[robot_idx],
                    t
                )

                poseMsg = PoseStamped()
                poseMsg.header.frame_id = self.virtual_robot_frame_names[robot_idx]

                trans[0] += trans[0]*self.resize_scaling_factor
                trans[1] += trans[1]*self.resize_scaling_factor

                poseMsg.pose.position.x = float(trans[0])
                poseMsg.pose.position.y = float(trans[1])
                poseMsg.pose.position.z = float(trans[2])
                poseMsg.pose.orientation.w = float(quaternions[3])
                poseMsg.pose.orientation.x = float(quaternions[0])
                poseMsg.pose.orientation.y = float(quaternions[1])
                poseMsg.pose.orientation.z = float(quaternions[2])

                self.tf_changer.publish(poseMsg)

    def shrink_structure(self):
        '''Shrink the team's structure by a scaling factor.'''
        for idx in range(self.num_assigned_robots):
            # Get the robot's index, so we select the correct topics and names.
            robot_idx = self.assigned_robot_indices[idx]

            if self.tf.frameExists(self.team_tf_frame_name) and\
                self.tf.frameExists(self.virtual_robot_frame_names[robot_idx]):
                t = self.tf.getLatestCommonTime(
                    self.virtual_robot_frame_names[robot_idx],
                    self.team_tf_frame_name
                )
                trans, quaternions = self.tf.lookupTransform(
                    self.team_tf_frame_name,
                    self.virtual_robot_frame_names[robot_idx],
                    t
                )

                poseMsg = PoseStamped()
                poseMsg.header.frame_id = self.virtual_robot_frame_names[robot_idx]

                trans[0] -= trans[0]*self.resize_scaling_factor
                trans[1] -= trans[1]*self.resize_scaling_factor

                poseMsg.pose.position.x = float(trans[0])
                poseMsg.pose.position.y = float(trans[1])
                poseMsg.pose.position.z = float(trans[2])
                poseMsg.pose.orientation.w = float(quaternions[3])
                poseMsg.pose.orientation.x = float(quaternions[0])
                poseMsg.pose.orientation.y = float(quaternions[1])
                poseMsg.pose.orientation.z = float(quaternions[2])

                self.tf_changer.publish(poseMsg)

    def save_structure(self):
        '''Save the current structure to a file.'''
        name, done1 = QInputDialog.getText(
             self, 'Save Structure', 'Enter desired save name:')
        if(done1):
            name = self.package_path + '/resource/' + name + '.txt'
            f = open(name, "w")

            for idx in range(self.num_assigned_robots):
                # Get the robot's index, so we select the correct topics and names.
                robot_idx = self.assigned_robot_indices[idx]

                if self.tf.frameExists(self.virtual_robot_frame_names[robot_idx]):
                    t = self.tf.getLatestCommonTime(
                        self.virtual_robot_frame_names[robot_idx],
                        self.team_tf_frame_name
                    )
                    trans, quaternions = self.tf.lookupTransform(
                        self.team_tf_frame_name,
                        self.virtual_robot_frame_names[robot_idx],
                        t
                    )

                    print(trans, quaternions)
                    f.write(f"robot_name: {self.virtual_robot_frame_names[robot_idx]}\n")
                    f.write(str(trans)+"\n")
                    f.write(str(quaternions)+"\n")
            f.close()

    def load_structure(self):
        '''Load a structure from a file.'''
        name, done1 = QInputDialog.getText(
            self, 'Load Structure', 'Enter desired file name:'
        )

        if done1:
            name = self.package_path+'/resource/'+ name + '.txt'
            f = open(name, "r+")
            lines = f.readlines()
            length = len(lines)//3

            for i in range(length):
                index=3*i
                frame_name=lines[index][12:].strip()
                position_line=lines[index+1][1:-2]
                quaternion_line=lines[index+2][1:-2]

                if self.tf.frameExists(self.team_tf_frame_name) and\
                    self.tf.frameExists(frame_name):
                    rospy.loginfo(str(i))

                    poseMsg = PoseStamped()
                    poseMsg.header.frame_id = frame_name
                    positions=position_line.split(', ')
                    quaternions=quaternion_line.split(', ')

                    poseMsg.pose.position.x = float(positions[0])
                    poseMsg.pose.position.y = float(positions[1])
                    poseMsg.pose.position.z = float(positions[2])
                    poseMsg.pose.orientation.w = float(quaternions[3])
                    poseMsg.pose.orientation.x = float(quaternions[0])
                    poseMsg.pose.orientation.y = float(quaternions[1])
                    poseMsg.pose.orientation.z = float(quaternions[2])

                    self.tf_changer.publish(poseMsg)
