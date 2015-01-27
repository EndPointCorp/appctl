#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String


class PortalLEDController():
    def __init__(self, path, baud):
        self.path = path
        self.baud = baud
        self.connect_serial()

    def connect_serial(self):
        self.port = serial.Serial(self.path, self.baud)

    def handle_command(self, command):
        rospy.loginfo('sending {}'.format(command))
        try:
            self.port.write(command)
        except serial.SerialTimeoutException as e:
            rospy.logerror('Timed out writing to port: {}'.format(e))


def listener():
    rospy.init_node('portal_leds_listener')

    device_path = rospy.get_param('~device_path')
    baud_rate = int(rospy.get_param('~baud_rate', 9600))

    controller = PortalLEDController(device_path, baud_rate)

    def handle_msg(msg):
        command = msg.data
        controller.handle_command(command)

    rospy.Subscriber('/portal_leds/command', String, handle_msg)

    rospy.spin()

if __name__ == '__main__':
    listener()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
