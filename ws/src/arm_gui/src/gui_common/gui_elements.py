import rospy

from PyQt5.QtCore import QPointF, Qt, pyqtProperty
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen, QRadialGradient
from PyQt5.QtWidgets import QAbstractButton, QLabel, QLineEdit, QPushButton

from geometry_msgs.msg import Twist


class ControlToggleButton(QPushButton):
    '''Changes push button color when pressed.'''
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