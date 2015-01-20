#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Byte


def main():
    buttondown_pub = rospy.Publisher(
        '/rfreceiver/buttondown',
        Byte,
        queue_size=1
    )
    rospy.init_node('rfreceiver')

    device_path = rospy.get_param('~device_path')
    baud_rate = rospy.get_param('~baud_rate', 9600)
    receiver = serial.Serial(device_path, baud_rate)

    buf = ''

    while not rospy.is_shutdown():
        try:
            button = int(receiver.readline(10).strip())
        except serial.SerialException as e:
            print e
            break

        buttondown_pub.publish(Byte(button))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
