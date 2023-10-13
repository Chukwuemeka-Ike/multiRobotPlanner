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

from matplotlib.backends.backend_qtagg import \
    NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtCore import QPointF, Qt, pyqtProperty
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen, QRadialGradient
from PyQt5.QtWidgets import QAbstractButton, QLabel, QLineEdit, QPushButton
from PyQt5.QtWidgets import QFrame, QWidget, QHBoxLayout, QPushButton, QVBoxLayout
from rviz import bindings as rviz

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest

from arm_msgs.msg import RobotEnableStatus


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

    def setEnabled(self, enabled):
        '''Ensures the toggle button is toggled off first and looks disabled when disabled.'''
        if enabled:
            self.setStyleSheet("QPushButton {background-color: light gray; color: black;}")
        else:
            self.enabled = False
            self.setStyleSheet("QPushButton {background-color: light gray; color: light gray;}")
        super().setEnabled(enabled)


class RobotButton(ToggleButton):
    '''.'''
    def __init__(self, button_name, button_topic) -> None:
        self.buttonName = f"Enable\n{button_name} Motion"
        super().__init__(self.buttonName)
        self.publisher = rospy.Publisher(button_topic, Twist, queue_size=0)


class ServiceButton(QPushButton):
    '''A push button that calls a service when clicked.'''
    def __init__(self, label: str, button_service: str, log_tag: str) -> None:
        '''.'''
        super().__init__(label)
        self.buttonService = button_service
        self.logTag = log_tag # Tag to use when sending logs.
        self.pressed.connect(self._buttonPressed)

    def _buttonPressed(self):
        '''Calls the service .'''
        rospy.wait_for_service(self.buttonService, timeout=1)
        try:
            request = TriggerRequest()
            service = rospy.ServiceProxy(self.buttonService, Trigger)
            service(request)
        except rospy.ServiceException as e:
            rospy.logerr(f'{self.logTag}: Service call failed: {e}.')


class ToggleServiceButton(ToggleButton):
    '''Push button that calls a service and changes color based on response.'''
    def __init__(self, label: str, button_service: str, log_tag: str) -> None:
        '''.'''
        super().__init__(label)
        self.buttonService = button_service
        self.logTag = log_tag

    def _buttonPressed(self):
        '''Toggles the button color when pressed.'''
        # Desired state is opposite of button's enable when it was pressed.
        desired_state = not(self.enabled)
        rospy.wait_for_service(self.buttonService, timeout=1)
        try:
            request = SetBoolRequest()
            request.data = desired_state
            service = rospy.ServiceProxy(self.buttonService, SetBool)
            result = service(request)

            # Only toggle the enable state if call succeeds.
            if result:
                self.enabled = not(self.enabled)
        except rospy.ServiceException as e:
            rospy.logerr(f'{self.logTag}: Service call failed: {e}.')

        if(self.enabled):
            self.setStyleSheet("QPushButton {background-color: blue; color: white;}")
        else:
            self.setStyleSheet("QPushButton {background-color: light gray; color: black;}")


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


class SaveOnlyNavigationToolbar(NavigationToolbar):
    '''NavigationToolbar with only the 'Save' button.'''
    toolitems = [t for t in NavigationToolbar.toolitems if t[0] == 'Save']


class MapWidget(QFrame):
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

        # Removed top and side view buttons to save real estate.
        # Keeping because they were cool.
        # h_layout = QHBoxLayout()

        # self.top_button = QPushButton("Top View")
        # self.top_button.clicked.connect(self._onTopButtonClick)
        # h_layout.addWidget(self.top_button)

        # self.side_button = QPushButton("Side View")
        # self.side_button.clicked.connect(self._onSideButtonClick)
        # h_layout.addWidget(self.side_button)

        # mapLayout.addLayout(h_layout)

        self.setFrameStyle(QFrame.Box | QFrame.Plain)
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
    def __init__(
            self, led_indicators, robot_enable_status_topic
        ) -> None:
        '''.'''

        self.led_indicators = led_indicators
        self.robot_enable_status = [
            led_indicators[i].active for i in range(len(led_indicators))
        ]

        # Register the publisher.
        self.publisher = rospy.Publisher(
            robot_enable_status_topic, RobotEnableStatus, queue_size=10
        )
        # Publish the enable status once to ensure other nodes have the
        # right values on creation.
        self.publish_enable_status()
   
    def poll_led_indicators(self):
        '''Polls the LED indicators.

        If any indicator has changed since last poll, it publishes all
        robots' enable statuses.
        '''
        # Count the number of LED indicators that have changed.
        num_changed_status = 0
        for i in range(len(self.led_indicators)):
            if(self.robot_enable_status[i] != self.led_indicators[i].active):
                num_changed_status += 1
                self.robot_enable_status[i] = self.led_indicators[i].active

        # Send the enabled and disabled IDs if any indicator has changed.
        if num_changed_status > 0:
            self.publish_enable_status()
        time.sleep(0.01)

    def publish_enable_status(self):
        '''Publishes the enable status for all robots.'''
        enabled_ids = []
        disabled_ids = []
        for i in range(len(self.led_indicators)):
            if self.led_indicators[i].active == True:
                enabled_ids.append(self.led_indicators[i].robot_id)
            elif self.led_indicators[i].active == False:
                disabled_ids.append(self.led_indicators[i].robot_id)

        output = RobotEnableStatus()
        output.enabled_ids = enabled_ids
        output.disabled_ids = disabled_ids
        self.publisher.publish(output)


class LEDIndicator(QAbstractButton):
    scaledSize = 1000.0
    def __init__(self, robot_id: int):
        '''.

        Args:
            robot_id: ID of the robot the indicator is tied to.
        '''
        super().__init__()

        self.setMinimumSize(24, 24)
        self.setCheckable(True)

        self.robot_id = robot_id
        self.active = False
        self.setChecked(self.active)
        self.setFixedHeight(50)

        self.clicked.connect(self.led_pressed)

        # Green when on.
        self.on_color_1 = QColor(0, 255, 0)
        self.on_color_2 = QColor(0, 192, 0)

        # Red when off.
        self.off_color_1 = QColor(255, 0, 0)
        self.off_color_2 = QColor(128, 0, 0)

    def led_pressed(self):
        '''Toggles the button's active state.'''
        self.active = not(self.active)
        self.setChecked(self.active)

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
