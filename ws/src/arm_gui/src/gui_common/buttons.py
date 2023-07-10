import rospy

from PyQt5.QtWidgets import QPushButton
from geometry_msgs.msg import Pose2D, Twist, PoseStamped


class ControlToggleButton(QPushButton):
    '''Changes push button color when pressed.'''
    def __init__(self, label):
        '''.'''
        super().__init__(label)
        self.pressed.connect(self._buttonPressed)
        self.toggled = False

    def _buttonPressed(self):
        '''Toggles the button color when pressed.'''
        self.toggled = not(self.toggled)
        if(self.toggled):
            self.setStyleSheet("QPushButton {background-color: blue; color: white;}")
        else:
            self.setStyleSheet("QPushButton {background-color: light gray; color: black;}")


class RobotButton():
    '''.'''
    def __init__(self, button_name, command_mode, button_topic) -> None:
        if(command_mode):
            self.buttonName = button_name + "\n Motion Enable"
        else:
            self.buttonName = button_name + " Frame\n Motion Enable"
        # self.motion_frame = "world"
        self.button = ControlToggleButton(self.buttonName)
        self.publisher = rospy.Publisher(button_topic, Twist, queue_size=0)
       