#!/usr/bin/env python

import os
import rospy
import subprocess
from std_msgs.msg import Byte

DEVNULL = open(os.devnull, 'w')
CLEAR_BUTTON = 2


def handle_button_msg(msg):
    if msg.data == CLEAR_BUTTON:
        subprocess.call(
            ['/home/lg/bin/lg-run-bg', 'pkill chrome'],
            stdout=DEVNULL,
            stderr=DEVNULL
        )


def main():
    buttondown_pub = rospy.Subscriber(
        '/rfreceiver/buttondown',
        Byte,
        handle_button_msg
    )
    rospy.init_node('rfreceiver_kill_browser')

    rospy.spin()

if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
