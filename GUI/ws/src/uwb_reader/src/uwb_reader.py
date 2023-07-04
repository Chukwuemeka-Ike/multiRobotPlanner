#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

import serial
import time

'''
uwb_reader.py
Alex Elias

Sends UWB LEC readings to ROS ropic as a string

Parameters:
    serial_port: e.g. '/dev/ttyS0'
    topic_name:  e.g. 'uwb_serial_front'
'''


class Uwb_reader:
    def __init__(self):
        rospy.init_node('uwb_reader', anonymous=True, disable_signals=True)
        self.serial_port = rospy.get_param('~serial_port')
        topic_name = rospy.get_param('~topic_name')

        self.ser = None

        self.pub = rospy.Publisher(topic_name, String, queue_size=1)
        
        rospy.on_shutdown(self.close_serial_if_active)

    def close_serial_if_active(self):
        if(not(self.ser == None)):
            self.ser.close()

    def start_lec_mode(self):
       # Reset UWB tag so that we're in a known state
        self.ser.write('reset\r'.encode())
        self.ser.write('reset\r'.encode())

        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
        ser_bytes = self.ser.readline().decode().strip()
        while not 'dwm>' in ser_bytes:
            rospy.loginfo('waiting for dwm>')
            rospy.loginfo(ser_bytes)
            self.ser.write('\r\r'.encode())
            ser_bytes = self.ser.readline().decode().strip()
            time.sleep(0.1)
            while(self.ser.in_waiting):
                rospy.loginfo('waiting for dwm> (ser.in_waiting)')
                ser_bytes = self.ser.readline().decode().strip()
                time.sleep(0.1)

        # Tell UWB tag to give us distance readings
        self.ser.write("lec\r".encode())

        ser_bytes = self.ser.readline().decode().strip()

        # Throw out first reading (has extra "dwm> ")
        ser_bytes = self.ser.readline().decode().strip() 

    def start_reading(self):
        while not rospy.is_shutdown():
            try:
                if(self.ser == None):
                    rospy.loginfo("Trying to reconnect to serial")
                    self.ser = serial.Serial(self.serial_port, 115200, timeout=1, xonxoff=True)
                    rospy.loginfo("Connected to serial")
                    # self.ser.reset_input_buffer()
                    # self.ser.reset_output_buffer()
                    time.sleep(1)
                    self.start_lec_mode()

                ser_bytes = self.ser.readline().decode().strip()
                if(ser_bytes):
                    self.pub.publish(ser_bytes)
                else:
                    rospy.logwarn("Serial timeout occured")

            except serial.serialutil.SerialException:
                if(not(self.ser == None)):
                    self.ser.close()
                    self.ser = None
                    rospy.logwarn("Disconnecting from serial")
                rospy.logwarn("Serial disconnected")
                time.sleep(0.25)


if __name__ == '__main__':
    uwb_reader = Uwb_reader()
    uwb_reader.start_reading()
