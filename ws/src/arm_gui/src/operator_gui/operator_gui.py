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

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import UInt32
from tf import TransformListener

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import *

from arm_msgs.msg import Ticket
from arm_msgs.srv import MachineStatus, MachineStatusRequest,\
        RobotAssignments, RobotAssignmentsRequest,\
        TicketList, TicketListRequest,\
        UnboundMachines, UnboundMachinesRequest
from arm_utils.conversion_utils import convert_ticket_list_to_task_dict

from gui_common.dialogs import BasicConfirmDialog, TicketDetailsDialog
from gui_common.gui_elements import ToggleButton, FixedWidthLabel,\
        LEDIndicator, LEDManager, MapWidget, RobotButton
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

        # *********************************************************************
        # Swarm Version 1 Code.
        # Control setup.
        self.get_control_params()

        self.team_buttons = []
        self.buttons = []
        self.labels = []
        self.leds = []

        self.synced_control_enabled = False
        self.rotation_disabled = False
        self.translation_disabled = False

        self.tf = TransformListener()
        self.tf_changer = None

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
        self.status_manager.poll_node_names()
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
        if self.endButton.isEnabled():
            self.release_machine_id()
            self.end_ticket()
        # If release button is enabled, a machine is still bound. Release it.
        elif self.machineIDReleaseButton.isEnabled():
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
        # These parameters are empty on startup and get updated whenever a task
        # is started 
        self.num_robots = 0
        self.node_names = []
        self.robot_command_topics = []
        self.robot_frame_command_topics = []
        self.robot_names = []
        self.real_robot_frame_names = []
        self.virtual_robot_frame_names = []

        self.resize_swarm_scaling_factor = float(rospy.get_param('resize_scaling_factor'))
        self.input_command_topic = rospy.get_param('input_command_topic')

        self.team_command_topic = ""
        self.team_frame_command_topic = ""
        self.team_footprint_topic = ""
        self.team_tf_frame = ""
        self.tf_changer_topic = ""

    def create_ui(self):
        '''Create the basic UI.'''

        # Create the machine, ticket, and control layouts and RViz widget.
        self.create_machine_layout()
        self.create_ticket_layout()
        self.create_control_layout()
        self.mapWidget = MapWidget(self.rviz_path)

        self.overallLayout.addLayout(self.machineLayout)
        self.overallLayout.addLayout(self.ticketLayout)
        self.overallLayout.addLayout(self.controlLayout)
        self.overallLayout.addWidget(self.mapWidget)

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

        # Set the .
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
        rospy.wait_for_service('unbound_machine_service')
        try:
            # TicketListRequest() is empty.
            request = UnboundMachinesRequest()
            unbound_machines = rospy.ServiceProxy('unbound_machine_service', UnboundMachines)
            response = unbound_machines(request)

            self.unbound_machines = response.machine_ids
            self.unbound_machine_names = response.machine_names

        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Unbound machines request failed: {e}.')

    def request_machine_assigned_tickets(self):
        '''Requests the tickets assigned to the selected machine ID.'''
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
            # print(f"Ready assigned tickets: {self.ready_assigned_tickets}")
        except rospy.ServiceException as e:
            rospy.logerr(f"{log_tag}: Machine assigned tickets "
                         f"request failed: {e}."
            )

    def request_ticket_list(self):
        '''Request the current ticket list from the ticket_service.'''
        rospy.wait_for_service('ticket_service')
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
        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Ticket list request failed: {e}.')

    def request_assigned_robot_information(self, ticket_id: int):
        '''.'''
        rospy.wait_for_service('robot_assignments_service')
        try:
            request = RobotAssignmentsRequest()
            request.ticket_id = ticket_id
            robot_assignments = rospy.ServiceProxy(
                'robot_assignments_service',
                RobotAssignments
            )
            response = robot_assignments(request)

            self.num_robots = response.num_assigned_robots
            self.robot_ids = response.robot_ids
            self.robot_names = response.robot_names
            self.robot_frame_command_topics = response.robot_frame_command_topics
            self.robot_command_topics = response.robot_command_topics
            self.virtual_robot_frame_names = response.virtual_robot_frame_names
            self.real_robot_frame_names = response.real_robot_frame_names

            self.team_id = response.team_id

            self.team_command_topic = response.team_command_topic
            self.team_frame_command_topic = response.team_frame_command_topic
            self.team_footprint_topic = response.team_footprint_topic
            self.team_tf_frame = response.team_tf_frame
            self.tf_changer_topic = response.tf_changer_topic

            if self.tf_changer is not None:
                self.tf_changer.unregister()

            if len(self.tf_changer_topic) > 0:
                self.tf_changer = rospy.Publisher(
                    self.tf_changer_topic, PoseStamped, queue_size=10
                )

            self.node_names = []
            for robot in range(self.num_robots):
                self.node_names.append(response.robot_node_names[robot].string_list)
        except rospy.ServiceException as e:
            rospy.logerr(f"{log_tag}: Robot assignment request failed: {e}.")

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
        self.request_assigned_robot_information(self.ticket_id)

        # If there are no assigned robots, ask the operator if they want to
        # work on the task manually. If they choose no, exit the function.
        if self.num_robots == 0:
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

        if self.num_robots != 0:
            enable_layout(self.controlLayout)
            self.update_robot_control_layout()
        
    def end_ticket(self):
        '''Ends the selected ticket and publishes its ID.'''
        # Publish the message.
        msg = Ticket()
        msg.ticket_id = self.ticket_id
        self.end_ticket_pub.publish(msg)

        self.is_ongoing = False

        # Clear the robot control layout.
        self.clear_robot_control_layout()

        # Enable the release machine button and the ID dropdown.
        self.ticketIDComboBox.setEnabled(True)
        self.machineIDReleaseButton.setEnabled(True)
        self.startButton.setEnabled(True)
        self.endButton.setEnabled(False)
        disable_layout(self.controlLayout)
        
        self.update_ticket_dropdown()

    def create_control_layout(self):
        '''.'''
        self.controlLayout = QHBoxLayout()
        self.create_task_layout()
        self.create_robot_control_layout()

        self.controlLayout.addLayout(self.taskLayout)
        self.controlLayout.addLayout(self.robotControlLayout)

    def create_task_layout(self):
        '''.'''
        self.taskLayout = QVBoxLayout()

        self.callRobotsButton = QPushButton("Call for Robots")
        self.callRobotsButton.clicked.connect(self.call_robots)

        self.taskMotionGroupBox = QGroupBox("Task Motion")
        self.taskMotionGroupBox.setCheckable(False)
        self.taskMotionLayout = QHBoxLayout()

        self.startMotionButton = QPushButton("Start Motion")
        self.startMotionButton.clicked.connect(self.start_task_motion)
        self.pauseMotionButton = QPushButton("Pause Motion")
        self.pauseMotionButton.clicked.connect(self.pause_task_motion)

        self.taskMotionLayout.addWidget(self.startMotionButton)
        self.taskMotionLayout.addWidget(self.pauseMotionButton)

        self.taskMotionGroupBox.setLayout(self.taskMotionLayout)

        self.taskLayout.addWidget(self.callRobotsButton)
        self.taskLayout.addWidget(self.taskMotionGroupBox)
        self.taskLayout.addWidget(FixedWidthLabel("Placeholder for Task Specific Controls", 300))
        self.taskLayout.addStretch()

    def call_robots(self):
        '''Calls the robots to the station when pressed.'''
        msg = PoseStamped()

        # Get the machine's location.
        msg.pose.position.x = self.machine_location[0]
        msg.pose.position.y = self.machine_location[1]
        # msg.pose.orientation. = self.machine_location[1]

        print(msg)

        # self.call_robots_pub.publish(msg)

    def start_task_motion(self):
        '''.'''

    def pause_task_motion(self):
        '''.'''

    def create_robot_control_layout(self):
        '''.'''
        self.robotControlLayout = QVBoxLayout()

        structureSizeLayout = QHBoxLayout()
        self.shrinkButton = QPushButton("-")
        self.shrinkButton.setFont(QFont('Times', structureButtonFontSize))
        self.shrinkButton.pressed.connect(self.shrink_structure)

        self.expandButton = QPushButton("+")
        self.expandButton.setFont(QFont('Times', structureButtonFontSize))
        self.expandButton.pressed.connect(self.expand_structure)

        adjustStructure = QLabel("<h2>Adjust Structure Size</h2>")
        adjustStructure.setAlignment(Qt.AlignCenter)

        structureSizeLayout.addWidget(self.shrinkButton)
        structureSizeLayout.addWidget(adjustStructure)
        structureSizeLayout.addWidget(self.expandButton)

        saveLoadLayout = QHBoxLayout()
        self.saveStructureButton = QPushButton("Save Structure")
        self.saveStructureButton.pressed.connect(self.save_structure)
        self.loadStructureButton = QPushButton("Load Structure")
        self.loadStructureButton.pressed.connect(self.load_structure)
        saveLoadLayout.addWidget(self.saveStructureButton)
        saveLoadLayout.addWidget(self.loadStructureButton)

        # Sync frame and rotation disable buttons.
        syncRotateLayout = QHBoxLayout()
        self.syncFramesButton = ToggleButton("Sync Frames")
        self.syncFramesButton.pressed.connect(self.sync_frames)
        syncRotateLayout.addWidget(self.syncFramesButton)
        self.disableRotationButton = ToggleButton("Disable Rotation")
        self.disableRotationButton.pressed.connect(self.toggle_rotation)
        syncRotateLayout.addWidget(self.disableRotationButton)

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
        self.robotControlLayout.addLayout(saveLoadLayout)
        self.robotControlLayout.addLayout(syncRotateLayout)
        self.robotControlLayout.addLayout(self.teamLayout)
        self.robotControlLayout.addLayout(self.robotLabelLayout)
        self.robotControlLayout.addLayout(self.robotButtonLayout)
        self.robotControlLayout.addLayout(self.robotFrameButtonLayout)
        self.robotControlLayout.addLayout(self.robotLEDLayout)
        self.robotControlLayout.addStretch()

    def clear_robot_control_layout(self):
        '''Clears the robot-specific control layout and parameters.'''
        # Empty the parameters to ensure the GUI doesn't maintain control
        # of robots it shouldn't.
        self.num_robots = 0
        self.node_names = []
        self.robot_command_topics = []
        self.robot_frame_command_topics = []
        self.robot_names = []
        self.real_robot_frame_names = []
        self.virtual_robot_frame_names = []

        self.team_command_topic = ""
        self.team_frame_command_topic = ""
        self.team_footprint_topic = ""
        self.team_tf_frame = ""
        self.tf_changer_topic = ""

        # Clear the 5 layouts.
        clear_layout(self.teamLayout)
        clear_layout(self.robotLabelLayout)
        clear_layout(self.robotButtonLayout)
        clear_layout(self.robotFrameButtonLayout)
        clear_layout(self.robotLEDLayout)

        # Update the layouts. With empty parameters, this empties buttons.
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

        if len(self.team_command_topic) and len(self.team_frame_command_topic):
            self.moveTeamButton = RobotButton("Team", self.team_command_topic)
            self.moveTeamFrameButton = RobotButton(
                "Team Frame",
                self.team_frame_command_topic
            )
            self.team_buttons.append(self.moveTeamButton)
            self.team_buttons.append(self.moveTeamFrameButton)
            self.teamLayout.addWidget(self.moveTeamButton)
            self.teamLayout.addWidget(self.moveTeamFrameButton)

        for robot in range(self.num_robots):
            led = LEDIndicator(robot)
            led.led_change(False)
            self.robotLEDLayout.addWidget(led)
            self.leds.append(led)

            label = QLabel(self.robot_names[robot])
            label.setAlignment(Qt.AlignCenter)
            self.robotLabelLayout.addWidget(label)
            self.labels.append(label)

            button = RobotButton(
                self.robot_names[robot], self.robot_command_topics[robot]
            )
            self.robotButtonLayout.addWidget(button)
            self.buttons.append(button)

            button = RobotButton(
                self.robot_names[robot] + " Frame",
                self.robot_frame_command_topics[robot]
            )
            self.robotFrameButtonLayout.addWidget(button)
            self.buttons.append(button)

        self.status_manager = LEDManager(self.node_names, self.leds)

    def toggle_rotation(self):
        '''.'''
        self.rotation_disabled = not(self.rotation_disabled)

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
        '''.'''
        for i in range(self.num_robots):
            if self.tf.frameExists(self.team_tf_frame) and self.tf.frameExists(self.real_robot_frame_names[i]):
                t = self.tf.getLatestCommonTime(self.real_robot_frame_names[i], self.team_tf_frame)
                (trans,quaternions) = self.tf.lookupTransform(self.team_tf_frame,self.real_robot_frame_names[i],t)
                poseMsg = PoseStamped()
                poseMsg.header.frame_id = self.virtual_robot_frame_names[i]
                
                poseMsg.pose.position.x = float(trans[0])
                poseMsg.pose.position.y = float(trans[1])
                poseMsg.pose.position.z = float(trans[2])
                poseMsg.pose.orientation.w = float(quaternions[3])
                poseMsg.pose.orientation.x = float(quaternions[0])
                poseMsg.pose.orientation.y = float(quaternions[1])
                poseMsg.pose.orientation.z = float(quaternions[2])
                #p_in_base = self.tf.transformPose("/base_link", poseMsg)
                self.tf_changer.publish(poseMsg)

    def expand_structure(self):
        '''Expand the swarm's structure by a scaling factor.'''
        for i in range(self.num_robots):
            if self.tf.frameExists(self.team_tf_frame) and self.tf.frameExists(self.virtual_robot_frame_names[i]):
                t = self.tf.getLatestCommonTime(self.virtual_robot_frame_names[i], self.team_tf_frame)
                trans, quaternions = self.tf.lookupTransform(self.team_tf_frame,self.virtual_robot_frame_names[i], t)
                poseMsg = PoseStamped()
                poseMsg.header.frame_id = self.virtual_robot_frame_names[i]
                #rospy.logwarn(str(trans[0]))
                trans[0] += trans[0]*self.resize_swarm_scaling_factor
                trans[1] += trans[1]*self.resize_swarm_scaling_factor
                #rospy.logwarn(str(trans[0]))
                poseMsg.pose.position.x = float(trans[0])
                poseMsg.pose.position.y = float(trans[1])
                poseMsg.pose.position.z = float(trans[2])
                poseMsg.pose.orientation.w = float(quaternions[3])
                poseMsg.pose.orientation.x = float(quaternions[0])
                poseMsg.pose.orientation.y = float(quaternions[1])
                poseMsg.pose.orientation.z = float(quaternions[2])
                #p_in_base = self.tf.transformPose("/base_link", poseMsg)
                self.tf_changer.publish(poseMsg)

    def shrink_structure(self):
        '''Shrink the swarm's structure by a scaling factor.'''
        for i in range(self.num_robots):
            if self.tf.frameExists(self.team_tf_frame) and self.tf.frameExists(self.virtual_robot_frame_names[i]):
                t = self.tf.getLatestCommonTime(self.virtual_robot_frame_names[i], self.team_tf_frame)
                trans, quaternions = self.tf.lookupTransform(self.team_tf_frame,self.virtual_robot_frame_names[i], t)
                poseMsg = PoseStamped()
                poseMsg.header.frame_id = self.virtual_robot_frame_names[i]
                trans[0] -= trans[0]*self.resize_swarm_scaling_factor
                trans[1] -= trans[1]*self.resize_swarm_scaling_factor
                poseMsg.pose.position.x = float(trans[0])
                poseMsg.pose.position.y = float(trans[1])
                poseMsg.pose.position.z = float(trans[2])
                poseMsg.pose.orientation.w = float(quaternions[3])
                poseMsg.pose.orientation.x = float(quaternions[0])
                poseMsg.pose.orientation.y = float(quaternions[1])
                poseMsg.pose.orientation.z = float(quaternions[2])

                self.tf_changer.publish(poseMsg)

# TODO: Cross-check structure mechanisms. Do they make sense?
    def save_structure(self):
        '''Save the current structure to a file.'''
        name, done1 = QInputDialog.getText(
             self, 'Save Structure', 'Enter desired save name:')
        if(done1):
            name = self.package_path + '/resource/' + name + '.txt'
            f = open(name, "w")
            
            for i in range(len(self.virtual_robot_frame_names)):
                if(self.tf.frameExists(self.virtual_robot_frame_names[i])):
                    t = self.tf.getLatestCommonTime(self.virtual_robot_frame_names[i], self.team_tf_frame)
                    position, quaternion = self.tf.lookupTransform(self.virtual_robot_frame_names[i], self.team_tf_frame, t)
                    
                    print(position, quaternion)
                    f.write("robot_name: %s\n"%self.virtual_robot_frame_names[i])
                    f.write(str(position)+"\n")
                    f.write(str(quaternion)+"\n")
            f.close()

    def load_structure(self):
        '''Load a structure from a file.'''
        name, done1 = QInputDialog.getText(
             self, 'Load Structure', 'Enter desired file name:')
        if(done1):
            name = self.package_path+'/resource/'+ name + '.txt'
            # rospy.logwarn("operator_gui.py: line 368: "+ name)
            f = open(name, "r+")
            lines=f.readlines()
            length=len(lines)//3
            for i in range(length):
                index=3*i
                frame_name=lines[index][12:].strip()
                position_line=lines[index+1][1:-2]
                quaternion_line=lines[index+2][1:-2]
                rospy.loginfo(frame_name)
                if self.tf.frameExists(self.team_tf_frame) and self.tf.frameExists(frame_name):
                    rospy.loginfo(str(i))
                    #t = self.tf.getLatestCommonTime(self.team_tf_frame, frame_name)
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
                    #p_in_base = self.tf.transformPose("/base_link", poseMsg)
                    self.tf_changer.publish(poseMsg)