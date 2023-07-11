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

from geometry_msgs.msg import Pose2D, Twist, PoseStamped
from rviz import bindings as rviz
from tf import TransformListener

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import *

from ws.src.arm_gui.src.gui_common.gui_elements import ControlToggleButton, RobotButton
from gui_common.dialogs import TicketInfoDialog
from gui_common.map_viz import create_map_widget


callback_lock = threading.Lock()
structureButtonFontSize = 20


class OperatorGUI(QMainWindow):
    '''Operator GUI class.'''
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Operator GUI")

        rospy.init_node("operator_gui")
        rospy.loginfo("Initializing Operator GUI")
        rp = rospkg.RosPack()
        self.gui_path = rp.get_path("arm_gui")
        rospy.on_shutdown(self.shutdownGUI)

        self.ticketID = 4

        # Set the central widget and window layout.
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)

        self.overallLayout = QVBoxLayout(self.centralWidget)
        self.centralWidget.setLayout(self.overallLayout)

        # Swarm Version 1 Code.
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

        self.buttons = []
        self.labels = []
        

        # Subscribe to the input command topic.
        rospy.Subscriber(self.input_command_topic, Twist, self._offsetCallback)

        # self.end_ticket_pub = rospy.Publisher("end_ticket",)

        self.synced_control_enabled = False
        self.rotation_disabled = False
        self.translation_disabled = False
        self.tf = TransformListener()
        self.tf_changer = rospy.Publisher(self.tf_changer_topic, PoseStamped, queue_size=10)

        # Create the display and status bar.
        self._createDisplay()
        self._createStatusBar()

        # Show the window.
        self.show()

    def shutdownGUI(self):
        '''Gracefully shutdown the GUI elements. Particularly RViz.'''
        # self.manager

    def _createDisplay(self):
        '''Create the basic display.'''
        
        self._createTicketLayout()
        self._createButtonsLayout()

        # Create the RViz widget.
        self.create_map_widget()

        self.overallLayout.addLayout(self.currentTicketLayout)
        self.overallLayout.addLayout(self.buttonsLayout)
        self.overallLayout.addWidget(self.mapWidget)

    def _createStatusBar(self):
        '''Create a simple status bar.'''
        status = QStatusBar(self.centralWidget)
        status.showMessage("Operator GUI")
        self.setStatusBar(status)

    def _createTicketLayout(self):
        '''.'''
        self.currentTicketLayout = QHBoxLayout()
        ticketLabel = QLabel("Current Ticket: ")
        ticketLabel.setAlignment(Qt.AlignCenter)
        self.idButton = QPushButton(str(self.ticketID))
        self.idButton.clicked.connect(self._displayJobCharacteristics)
        self.endButton = QPushButton("End")
        self.endButton.setStyleSheet("background-color : green")
        
        self.currentTicketLayout.addWidget(ticketLabel)
        self.currentTicketLayout.addWidget(self.idButton)
        self.currentTicketLayout.addWidget(self.endButton)

    def _displayJobCharacteristics(self):
        '''.'''
        ticketInfoDialog = TicketInfoDialog(self)
        ticketInfoDialog.setModal(True)
        ticketInfoDialog.show()

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
