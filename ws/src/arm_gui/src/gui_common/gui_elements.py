#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Common GUI elements used across the Supervisor and Operator GUIs.
'''
import rospy
import time

from PyQt5.QtCore import QPointF, Qt, pyqtProperty
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen, QRadialGradient
from PyQt5.QtWidgets import QAbstractButton, QLabel, QLineEdit, QPushButton
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton, QVBoxLayout
from rviz import bindings as rviz

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class ToggleButton(QPushButton):
    '''A push button that changes color when pressed.'''
    def __init__(self, label):
        '''.'''
        super().__init__(label)
        self.pressed.connect(self._buttonPressed)
        self.enabled = False

    def _buttonPressed(self):
        '''Toggles the button color when pressed.'''
        self.enabled = not(self.enabled)
        if(self.enabled):
            self.setStyleSheet("QPushButton {background-color: blue; color: white;}")
        else:
            self.setStyleSheet("QPushButton {background-color: light gray; color: black;}")

class RobotButton(ToggleButton):
    '''.'''
    def __init__(self, button_name, button_topic) -> None:
        self.buttonName = button_name + "\n Motion Enable"
        super().__init__(self.buttonName)
        self.publisher = rospy.Publisher(button_topic, Twist, queue_size=0)

class FixedWidthLabel(QLabel):
    '''Creates a fixed width label. Allows us specify the size in the constructor.'''
    def __init__(self, label, width=None):
        super().__init__(label)
        if width is not None:
            self.setFixedWidth(width)
        self.setAlignment(Qt.AlignCenter)


class FixedWidthLineEdit(QLineEdit):
    '''Creates a fixed width line edit.'''
    def __init__(self, width=None):
        super().__init__()
        if width is not None:
            self.setFixedWidth(width)


class FixedWidthPushButton(QPushButton):
    '''Creates a fixed width push button.'''
    def __init__(self, width=None):
        super().__init__()
        if width is not None:
            self.setFixedWidth(width)


class MapWidget(QWidget):
    '''Widget containing an RViz view for map visualization.'''
    def __init__(self, rviz_path: str) -> None:
        super().__init__()
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, rviz_path)
        self.frame.load(config)

        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )

        self.manager = self.frame.getManager()

        mapLayout = QVBoxLayout()
        mapLayout.addWidget(self.frame)

        h_layout = QHBoxLayout()

        self.top_button = QPushButton("Top View")
        self.top_button.clicked.connect(self._onTopButtonClick)
        h_layout.addWidget(self.top_button)

        self.side_button = QPushButton("Side View")
        self.side_button.clicked.connect(self._onSideButtonClick)
        h_layout.addWidget(self.side_button)

        mapLayout.addLayout(h_layout)

        self.setLayout(mapLayout)

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


class LEDManager:
    def __init__(self,node_names,led_objects):
        self.node_names = node_names
        self.led_objects = led_objects
        self.active_bots = []
        self.robot_enable_status_topic = rospy.get_param('robot_enable_status_topic') 
        self.publisher = rospy.Publisher(self.robot_enable_status_topic, Int32, queue_size=10)
        self.send_value = 0
        for i in range(len(led_objects)):
            self.active_bots.append(False)
            
    def poll_node_names(self):
        #node_names is loaded from yaml file and should be a list of lists for each robot of desired nodes
        #print(self.node_names)
        for i in range(len(self.led_objects)):
            if(self.active_bots[i]!=self.led_objects[i].active):
                if(self.led_objects[i].active==True):
                    out=pow(2,i)
                    self.send_value+=out
                    self.active_bots[i]=True
                else:
                    out=pow(2,i)
                    self.send_value-=out
                    self.active_bots[i]=False

                output = Int32()
                output.data=int(self.send_value)
                self.publisher.publish(output)
        time.sleep(0.01)


class LEDIndicator(QAbstractButton):
    scaledSize=1000.0
    def __init__(self,index):
        QAbstractButton.__init__(self)
        self.setMinimumSize(24, 24)
        self.setCheckable(True)
        self.index=index
        self.active=False
        self.pressed.connect(self.led_pressed)
        # Green
        self.on_color_1 = QColor(0, 255, 0)
        self.on_color_2 = QColor(0, 192, 0)
        self.off_color_1 = QColor(255, 0, 0)
        self.off_color_2 = QColor(128, 0, 0)

    def led_pressed(self):
        self.active=not(self.active)
        self.led_change(self.active)
        rospy.logwarn("hello")

    def resizeEvent(self, QResizeEvent):
        self.update()

    def paintEvent(self, QPaintEvent):
        realSize = min(self.width(), self.height())

        painter = QPainter(self)
        pen = QPen(Qt.black)
        pen.setWidth(1)

        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.width() / 2, self.height() / 2)
        painter.scale(realSize / self.scaledSize, realSize / self.scaledSize)

        gradient = QRadialGradient(QPointF(-500, -500), 1500, QPointF(-500, -500))
        gradient.setColorAt(0, QColor(224, 224, 224))
        gradient.setColorAt(1, QColor(28, 28, 28))
        painter.setPen(pen)
        painter.setBrush(QBrush(gradient))
        painter.drawEllipse(QPointF(0, 0), 500, 500)

        gradient = QRadialGradient(QPointF(500, 500), 1500, QPointF(500, 500))
        gradient.setColorAt(0, QColor(224, 224, 224))
        gradient.setColorAt(1, QColor(28, 28, 28))
        painter.setPen(pen)
        painter.setBrush(QBrush(gradient))
        painter.drawEllipse(QPointF(0, 0), 450, 450)

        painter.setPen(pen)
        if self.isChecked():
            gradient = QRadialGradient(QPointF(-500, -500), 1500, QPointF(-500, -500))
            gradient.setColorAt(0, self.on_color_1)
            gradient.setColorAt(1, self.on_color_2)
        else:
            gradient = QRadialGradient(QPointF(500, 500), 1500, QPointF(500, 500))
            gradient.setColorAt(0, self.off_color_1)
            gradient.setColorAt(1, self.off_color_2)

        painter.setBrush(gradient)
        painter.drawEllipse(QPointF(0, 0), 400, 400)

    def led_change(self, state):
        self.setChecked(state)

    @pyqtProperty(QColor)
    def onColor1(self):
        return self.on_color_1

    @onColor1.setter
    def onColor1(self, color):
        self.on_color_1 = color

    @pyqtProperty(QColor)
    def onColor2(self):
        return self.on_color_2

    @onColor2.setter
    def onColor2(self, color):
        self.on_ciagnosticscreen.backToRun.pressed.connect(self._to_run_screen)
        #self._runscolor_2 = color

    @pyqtProperty(QColor)
    def offColor1(self):
        return self.off_color_1

    @offColor1.setter
    def offColor1(self, color):
        self.off_color_1 = color

    @pyqtProperty(QColor)
    def offColor2(self):
        return self.off_color_2

    @offColor2.setter
    def offColor2(self, color):
        self.off_color_2 = color