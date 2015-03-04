#!/usr/bin/env python

import os
import rospy
import subprocess
from std_msgs.msg import Byte
from appctl.msg import Mode

DEVNULL = open(os.devnull, 'w')
CLEAR_BUTTON = 2


class Blah:
    def __init__(self):
        rospy.init_node('rfreceiver_kill_browser')
        self.appctl_pub = rospy.Publisher(
            '/appctl/mode',
            Mode,
            queue_size=1
        )

    def handle_button_msg(self, msg):
        if msg.data == CLEAR_BUTTON:
            subprocess.call(
                ['/home/lg/bin/lg-run-bg', 'pkill chrome'],
                stdout=DEVNULL,
                stderr=DEVNULL
            )
            self.appctl_pub.publish(Mode(mode='tactile'))


    def main(self):
        buttondown_pub = rospy.Subscriber(
            '/rfreceiver/buttondown',
            Byte,
            self.handle_button_msg
        )

        rospy.spin()

if __name__ == '__main__':
    Blah().main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
