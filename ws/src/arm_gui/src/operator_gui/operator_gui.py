#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Constructs and shows the Supervisor GUI using PyQt5.
'''
import os
import rospy
import rospkg
import sys
import threading

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import UInt32

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import *

from tf import TransformListener

from arm_msgs.msg import Ticket, Tickets
from arm_msgs.srv import MachineStatus, MachineStatusRequest,\
    TicketList, TicketListRequest,\
    UnboundMachines, UnboundMachinesRequest

from arm_utils.conversion_utils import convert_ticket_list_to_task_dict

from gui_common.gui_elements import ControlToggleButton, RobotButton, FixedWidthLabel
from gui_common.dialogs import TicketInfoDialog
from gui_common.map_viz import create_map_widget


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

        self.ticketID = 4

        # Set the central widget and window layout.
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)

        self.overallLayout = QVBoxLayout(self.centralWidget)
        self.centralWidget.setLayout(self.overallLayout)

        # Publishers for binding/releasing a machine to/from the GUI instance.
        self.bind_machine_pub = rospy.Publisher(
            "bind_machine", UInt32, queue_size=10
        )
        self.release_machine_pub = rospy.Publisher(
            "release_machine", UInt32, queue_size=10
        )
        # Publisher for starting a ticket.
        self.start_ticket_pub = rospy.Publisher(
            "start_ticket", Ticket, queue_size=10
        )# Publisher for ending a ticket when done.
        self.end_ticket_pub = rospy.Publisher(
            "end_ticket", Ticket, queue_size=10
        )

        # Swarm Version 1 Code.
        # Control setup.
        self.get_control_params()

        self.buttons = []
        self.labels = []

        self.synced_control_enabled = False
        self.rotation_disabled = False
        self.translation_disabled = False
        self.tf = TransformListener()
        self.tf_changer = rospy.Publisher(self.tf_changer_topic, PoseStamped, queue_size=10)

        # Subscribe to the input command topic.
        rospy.Subscriber(self.input_command_topic, Twist, self._offsetCallback)

        self.unbound_machines = []
        self.unbound_machine_names = []

        # Create the display and status bar.
        self._createDisplay()
        self._createStatusBar()

        # Show the window.
        self.show()

    def shutdown_gui(self):
        '''Gracefully shutdown the GUI elements. Particularly RViz.'''
        # self.manager

    def get_control_params(self):
        '''.'''
        self.number_of_bots = rospy.get_param('number_of_robots')
        self.nodenames = rospy.get_param('robot_node_names')
        self.open_loop_command_topics = rospy.get_param('open_loop_command_topics')
        self.close_loop_command_topics = rospy.get_param('closed_loop_command_topics')
        self.input_command_topic = rospy.get_param('input_command_topic')
        self.robot_types = rospy.get_param('robot_type_information')
        self.closed_loop_swarm_command_topic = rospy.get_param('closed_loop_swarm_command_topic')
        self.open_loop_swarm_command_topic = rospy.get_param('open_loop_swarm_command_topic')
        #self.sync_topic = rospy.get_param('sync_frames_topic')
        self.swarm_tf = rospy.get_param('swarm_tf_frame')
        self.robot_tfs = rospy.get_param('robot_tf_frames')
        self.real_robot_tfs = rospy.get_param('real_robot_tf_frames')
        self.resize_swarm_scaling_factor = float(rospy.get_param('resize_scaling_factor'))
        self.tf_changer_topic = rospy.get_param('tf_changer_topic')

    def _createDisplay(self):
        '''Create the basic display.'''
        
        self.create_machine_layout()
        self._createTicketLayout()
        self._createButtonsLayout()

        # Create the RViz widget.
        self.create_map_widget()

        self.overallLayout.addLayout(self.machineIDLayout)
        self.overallLayout.addLayout(self.currentTicketLayout)
        self.overallLayout.addLayout(self.buttonsLayout)
        self.overallLayout.addWidget(self.mapWidget)

        # Disable the ticket and buttons layouts.
        self.disable_layout(self.currentTicketLayout)
        self.disable_layout(self.buttonsLayout)

    def _createStatusBar(self):
        '''Create a simple status bar.'''
        status = QStatusBar(self.centralWidget)
        status.showMessage("Operator GUI")
        self.setStatusBar(status)

    def create_machine_layout(self):
        '''.'''
        self.request_unbound_machines()

        componentSizes = self.width()/5
        self.machineIDLayout = QHBoxLayout()
        machineIDLabel = QLabel("Machine ID")
        machineIDLabel.setAlignment(Qt.AlignCenter)

        self.machineIDComboBox = QComboBox()
        self.machineIDComboBox.addItem("Select Machine ID")
        self.machineIDComboBox.addItems(self.unbound_machine_names)
        self.machineIDComboBox.currentIndexChanged.connect(self.select_machine_id)

        self.machineIDSelectButton = QPushButton("Set Machine")
        self.machineIDSelectButton.setEnabled(False)
        self.machineIDSelectButton.clicked.connect(self.bind_machine_id)
        
        self.machineIDReleaseButton = QPushButton("Release Machine")
        self.machineIDReleaseButton.setStyleSheet("background-color : red")
        self.machineIDReleaseButton.setEnabled(False)
        self.machineIDReleaseButton.clicked.connect(self.release_machine_id)

        self.machineIDLayout.addWidget(machineIDLabel)
        self.machineIDLayout.addWidget(self.machineIDComboBox)
        self.machineIDLayout.addWidget(self.machineIDSelectButton)
        self.machineIDLayout.addWidget(self.machineIDReleaseButton)

    def select_machine_id(self):
        '''.'''
        if self.machineIDComboBox.currentText() == "Select Machine ID":
            self.machineIDSelectButton.setEnabled(False)
        else:
            self.machineIDSelectButton.setEnabled(True)

    def bind_machine_id(self):
        '''Binds the Operator GUI to the selected machine ID.

        Sends the machine ID message over the topic, which Machine Manager
        subscribes to.
        '''
        # Publish the message.
        msg = UInt32()
        machine_id = self.unbound_machines[
            self.unbound_machine_names.index(
                self.machineIDComboBox.currentText()
            )
        ]
        msg.data = machine_id
        self.bind_machine_pub.publish(msg)

        # Disable the select button and combo box, and enable the release button.
        self.machineIDComboBox.setEnabled(False)
        self.machineIDSelectButton.setEnabled(False)
        self.machineIDReleaseButton.setEnabled(True)

        # Populate the ticket combo box with the IDs that are assigned to the
        # machine and ready.
        self.request_machine_assigned_tickets()
        self.request_ticket_list()
        self.ready_assigned_tickets = [
            id for id in self.assigned_tickets if id in self.ready
        ]
        self.ticketIDComboBox.addItems([
            str(id) for id in self.ready_assigned_tickets
        ])

        # Enable the ticket and buttons layouts.
        self.enable_layout(self.currentTicketLayout)
        self.enable_layout(self.buttonsLayout)

    def release_machine_id(self):
        '''Releases the machine ID from the Operator GUI.

        Sends the machine ID message over the topic, which Machine Manager
        subscribes to.
        '''
        # Publish the message.
        msg = UInt32()
        machine_id = self.unbound_machines[
            self.unbound_machine_names.index(
                self.machineIDComboBox.currentText()
            )
        ]
        msg.data = machine_id
        self.release_machine_pub.publish(msg)

        # Enable the select button and combo box, and disable the release button.
        self.machineIDComboBox.setEnabled(True)
        self.machineIDSelectButton.setEnabled(True)
        self.machineIDReleaseButton.setEnabled(False)

        # Empty the ticket combo box and the assigned ticket list.
        self.ready_assigned_tickets = []
        self.ticketIDComboBox.clear()

        # Disable the ticket and buttons layouts.
        self.disable_layout(self.currentTicketLayout)
        self.disable_layout(self.buttonsLayout)

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
            request.machine_id = self.unbound_machines[
                self.unbound_machine_names.index(
                    self.machineIDComboBox.currentText()
                )
            ]
            machine_status = rospy.ServiceProxy('machine_status_service', MachineStatus)
            response = machine_status(request)

            self.assigned_tickets = response.assigned_ids
            self.machine_status = response.status
        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Machine assigned tickets request failed: {e}.')

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

    def _createTicketLayout(self):
        '''.'''
        self.currentTicketLayout = QHBoxLayout()
        ticketLabel = QLabel("Current Ticket")
        ticketLabel.setAlignment(Qt.AlignCenter)

        self.ticketIDComboBox = QComboBox()
        self.ticketIDComboBox.currentIndexChanged.connect(self.on_ticket_id_changed)

        self.idButton = QPushButton("Details")
        self.idButton.clicked.connect(self._displayJobCharacteristics)

        self.startButton = QPushButton("Start")
        self.startButton.clicked.connect(self.start_ticket)

        self.endButton = QPushButton("End")
        self.endButton.setStyleSheet("background-color : green")
        self.endButton.clicked.connect(self.end_ticket)

        self.currentTicketLayout.addWidget(ticketLabel)
        self.currentTicketLayout.addWidget(self.ticketIDComboBox)
        self.currentTicketLayout.addWidget(self.idButton)
        self.currentTicketLayout.addWidget(self.startButton)
        self.currentTicketLayout.addWidget(self.endButton)

    def on_ticket_id_changed(self):
        '''.'''

    def _displayJobCharacteristics(self):
        '''.'''
        ticketInfoDialog = TicketInfoDialog(self)
        ticketInfoDialog.setModal(True)
        ticketInfoDialog.show()

    def start_ticket(self):
        '''Starts the selected ticket and publishes its ID.'''
        # Publish the message.
        msg = Ticket()
        msg.ticket_id = int(self.ticketIDComboBox.currentText())
        self.start_ticket_pub.publish(msg)

        # Disable the release machine button and the ID dropdown.
        self.ticketIDComboBox.setEnabled(False)
        self.machineIDReleaseButton.setEnabled(False)
        
    def end_ticket(self):
        '''Ends the selected ticket and publishes its ID.'''
        # Publish the message.
        msg = Ticket()
        msg.ticket_id = int(self.ticketIDComboBox.currentText())
        self.end_ticket_pub.publish(msg)

        # Enable the release machine button and the ID dropdown.
        self.ticketIDComboBox.setEnabled(True)
        self.machineIDReleaseButton.setEnabled(True)

        # Empty the ticket combo box, then add the IDs after requesting
        # a new ticket list and machine assignments.
        self.request_machine_assigned_tickets()
        self.request_ticket_list()

        self.ready_assigned_tickets = [
            id for id in self.assigned_tickets if id in self.ready
        ]
        self.ticketIDComboBox.clear()
        self.ticketIDComboBox.addItems([
            str(id) for id in self.ready_assigned_tickets
        ])

    def _createButtonsLayout(self):
        '''.'''
        self.buttonsLayout = QHBoxLayout()
        self._createPerimeterLayout()
        self._createControlLayout()

        self.buttonsLayout.addLayout(self.perimeterLayout)
        self.buttonsLayout.addLayout(self.controlLayout)

    def _createPerimeterLayout(self):
        '''.'''
        self.perimeterLayout = QVBoxLayout()
        self.perimeterLayout.addWidget(FixedWidthLabel("Placeholder for Task Specific Controls", 300))

    def _createControlLayout(self):
        '''.'''
        self.controlLayout = QVBoxLayout()

        structureSizeLayout = QHBoxLayout()
        self.shrinkButton = QPushButton("-")
        self.shrinkButton.setFont(QFont('Times', structureButtonFontSize))
        self.shrinkButton.pressed.connect(self._shrinkStructure)

        self.expandButton = QPushButton("+")
        self.expandButton.setFont(QFont('Times', structureButtonFontSize))
        self.expandButton.pressed.connect(self._expandStructure)

        adjustStructure = QLabel("<h2>Adjust Structure Size</h2>")
        adjustStructure.setAlignment(Qt.AlignCenter)

        structureSizeLayout.addWidget(self.shrinkButton)
        structureSizeLayout.addWidget(adjustStructure)
        structureSizeLayout.addWidget(self.expandButton)

        saveLoadLayout = QHBoxLayout()
        self.saveStructureButton = QPushButton("Save Structure")
        self.saveStructureButton.pressed.connect(self._saveStructure)
        self.loadStructureButton = QPushButton("Load Structure")
        self.loadStructureButton.pressed.connect(self._loadStructure)
        saveLoadLayout.addWidget(self.saveStructureButton)
        saveLoadLayout.addWidget(self.loadStructureButton)

        # Sync frame and rotation disable buttons.
        syncRotateLayout = QHBoxLayout()
        self.syncFramesButton = ControlToggleButton("Sync Frames")
        self.syncFramesButton.pressed.connect(self._syncFrames)
        syncRotateLayout.addWidget(self.syncFramesButton)
        self.disableRotationButton = ControlToggleButton("Disable Rotation")
        self.disableRotationButton.pressed.connect(self._toggleRotation)
        syncRotateLayout.addWidget(self.disableRotationButton)
        
        # Swarm control buttons.
        swarmLayout = QHBoxLayout()
        
        self.moveSwarmButton = RobotButton("Swarm", True, self.closed_loop_swarm_command_topic)
        self.moveSwarmFrameButton = RobotButton("Swarm", False, self.open_loop_swarm_command_topic)
        self.buttons.append(self.moveSwarmButton)
        self.buttons.append(self.moveSwarmFrameButton)
        swarmLayout.addWidget(self.moveSwarmButton.button)
        swarmLayout.addWidget(self.moveSwarmFrameButton.button)


        # Individual robot control buttons and labels.
        robotLabelLayout = QHBoxLayout()
        robotButtonLayout = QHBoxLayout()
        robotFrameButtonLayout = QHBoxLayout()

        for robot in range(self.number_of_bots):
            label = QLabel(self.robot_types[robot])
            label.setAlignment(Qt.AlignCenter)
            robotLabelLayout.addWidget(label)
            self.labels.append(label)
            
            button = RobotButton(self.robot_types[robot], True, self.open_loop_command_topics[robot])
            robotButtonLayout.addWidget(button.button)
            self.buttons.append(button)

            button = RobotButton(self.robot_types[robot], False, self.close_loop_command_topics[robot])
            robotFrameButtonLayout.addWidget(button.button)
            self.buttons.append(button)

        # Add all the layouts to the control layout.
        self.controlLayout.addLayout(structureSizeLayout)
        self.controlLayout.addLayout(saveLoadLayout)
        self.controlLayout.addLayout(syncRotateLayout)
        self.controlLayout.addLayout(swarmLayout)
        self.controlLayout.addLayout(robotLabelLayout)
        self.controlLayout.addLayout(robotButtonLayout)
        self.controlLayout.addLayout(robotFrameButtonLayout)

    def _toggleRotation(self):
        '''.'''
        self.rotation_disabled = not(self.rotation_disabled)

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

    def _switchToView(self, view_name: str):
        '''Looks for view_name in the saved views in the config.'''
        view_man = self.manager.getViewManager()
        for i in range(view_man.getNumViews()):
            if view_man.getViewAt(i).getName() == view_name:
                view_man.setCurrentFrom(view_man.getViewAt(i))
                return
        print("Could not find view named {view_name}")

    def enable_layout(self, layout):
        '''Enables all the widgets in the given layout.'''
        for i in range(layout.count()):
            item = layout.itemAt(i)
            if isinstance(item, QWidgetItem):
                widget = item.widget()
                if widget:
                    widget.setEnabled(True)
            elif isinstance(item, QLayoutItem):
                sub_layout = item.layout()
                if sub_layout:
                    self.enable_layout(sub_layout)

    def disable_layout(self, layout):
        '''Disables all the widgets in the given layout.'''
        for i in range(layout.count()):
            item = layout.itemAt(i)
            if isinstance(item, QWidgetItem):
                widget = item.widget()
                if widget:
                    widget.setEnabled(False)
            elif isinstance(item, QLayoutItem):
                sub_layout = item.layout()
                if sub_layout:
                    self.disable_layout(sub_layout)

    def _offsetCallback(self, msg):
        '''Alters the received input command, then publishes to enabled bots.'''
        with callback_lock:
            if(self.rotation_disabled):
                msg.angular.x = msg.angular.x*0.
                msg.angular.y = msg.angular.y*0.
                msg.angular.z = msg.angular.z*0.
            else:
                msg.angular.x = msg.angular.x*1.0
                msg.angular.y = msg.angular.y*1.0
                msg.angular.z = msg.angular.z*1.0

            if(self.translation_disabled):
                msg.linear.x = msg.linear.x*0.
                msg.linear.y = msg.linear.y*0.
                msg.linear.z = msg.linear.z*0.
            else:
                msg.linear.x = msg.linear.x*1.0
                msg.linear.y = msg.linear.y*1.0
                msg.linear.z = msg.linear.z*1.0

            for i in range(len(self.buttons)):
                if(self.buttons[i].enabled):
                    self.buttons[i].publisher.publish(msg)
      
    def _syncFrames(self):
        '''.'''
        for i in range(self.number_of_bots):
            if self.tf.frameExists(self.swarm_tf) and self.tf.frameExists(self.real_robot_tfs[i]):
                t = self.tf.getLatestCommonTime(self.real_robot_tfs[i], self.swarm_tf)
                (trans,quaternions) = self.tf.lookupTransform(self.swarm_tf,self.real_robot_tfs[i],t)
                poseMsg = PoseStamped()
                poseMsg.header.frame_id = self.robot_tfs[i]
                
                poseMsg.pose.position.x = float(trans[0])
                poseMsg.pose.position.y = float(trans[1])
                poseMsg.pose.position.z = float(trans[2])
                poseMsg.pose.orientation.w = float(quaternions[3])
                poseMsg.pose.orientation.x = float(quaternions[0])
                poseMsg.pose.orientation.y = float(quaternions[1])
                poseMsg.pose.orientation.z = float(quaternions[2])
                #p_in_base = self.tf.transformPose("/base_link", poseMsg)
                self.tf_changer.publish(poseMsg)

    # def sync_robot_motion_pressed(self):
    #     '''.'''
    #     self.synced_control_enabled = True
    #     for i in range(len(self.buttons)):
    #         if(not self.buttons[i].enabled):
    #             self.synced_control_enabled=False
    #             break
                
    #     #if(self.rob1en and self.rob2en and self.rob3en): self.synced_control_enabled=True
    #     self.synced_control_enabled = not(self.synced_control_enabled)
        
    #     if(self.synced_control_enabled):
    #         for i in range(len(self.buttons)):
    #             self.buttons[i].enabled=True
    #             self.buttons[i].button.setStyleSheet('QPushButton {background-color: orange; color: white;}')
    #     else:
    #         for i in range(len(self.buttons)):
    #             self.buttons[i].enabled=False
    #             self.buttons[i].button.setStyleSheet('QPushButton {background-color: white; color: black;}')

    def _expandStructure(self):
        '''Expand the swarm's structure by a scaling factor.'''
        for i in range(self.number_of_bots):
            if self.tf.frameExists(self.swarm_tf) and self.tf.frameExists(self.robot_tfs[i]):
                t = self.tf.getLatestCommonTime(self.robot_tfs[i], self.swarm_tf)
                trans, quaternions = self.tf.lookupTransform(self.swarm_tf,self.robot_tfs[i], t)
                poseMsg = PoseStamped()
                poseMsg.header.frame_id = self.robot_tfs[i]
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


    def _shrinkStructure(self):
        '''Shrink the swarm's structure by a scaling factor.'''
        for i in range(self.number_of_bots):
            if self.tf.frameExists(self.swarm_tf) and self.tf.frameExists(self.robot_tfs[i]):
                t = self.tf.getLatestCommonTime(self.robot_tfs[i], self.swarm_tf)
                trans, quaternions = self.tf.lookupTransform(self.swarm_tf,self.robot_tfs[i], t)
                poseMsg = PoseStamped()
                poseMsg.header.frame_id = self.robot_tfs[i]
                trans[0] -= trans[0]*self.resize_swarm_scaling_factor
                trans[1] -= trans[1]*self.resize_swarm_scaling_factor
                poseMsg.pose.position.x = float(trans[0])
                poseMsg.pose.position.y = float(trans[1])
                poseMsg.pose.position.z = float(trans[2])
                poseMsg.pose.orientation.w = float(quaternions[3])
                poseMsg.pose.orientation.x = float(quaternions[0])
                poseMsg.pose.orientation.y = float(quaternions[1])
                poseMsg.pose.orientation.z = float(quaternions[2])
                #p_in_base = self.tf.transformPose("/base_link", poseMsg)
                self.tf_changer.publish(poseMsg)

# TODO: Cross-check structure mechanisms. Do they make sense?
    def _saveStructure(self):
        '''Save the current structure to a file.'''
        name, done1 = QInputDialog.getText(
             self, 'Save Structure', 'Enter desired save name:')
        if(done1):
            name = self.package_path + '/resource/' + name + '.txt'
            f = open(name, "w")
            
            for i in range(len(self.robot_tfs)):
                if(self.tf.frameExists(self.robot_tfs[i])):
                    t = self.tf.getLatestCommonTime(self.robot_tfs[i], self.swarm_tf)
                    position, quaternion = self.tf.lookupTransform(self.robot_tfs[i], self.swarm_tf, t)
                    
                    print(position, quaternion)
                    f.write("robot_name: %s\n"%self.robot_tfs[i])
                    f.write(str(position)+"\n")
                    f.write(str(quaternion)+"\n")
            f.close()

    def _loadStructure(self):
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
                if self.tf.frameExists(self.swarm_tf) and self.tf.frameExists(frame_name):
                    rospy.loginfo(str(i))
                    #t = self.tf.getLatestCommonTime(self.swarm_tf, frame_name)
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

    

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ww = OperatorGUI()
    sys.exit(app.exec())
