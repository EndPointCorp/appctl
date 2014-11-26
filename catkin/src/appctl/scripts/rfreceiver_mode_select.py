#!/usr/bin/env python

"""
This node listens for keyfob button presses and changes the mode accordingly.
"""

import rospy
from appctl.msg import Mode
from std_msgs.msg import Byte


class ButtonHandler:
    def __init__(self, modes, mode_pub):
        self.modes = modes
        self.mode_pub = mode_pub

    def handle_msg(self, msg):
        if msg.data in self.modes:
            self.mode_pub.publish(mode=self.modes[msg.data])


def main():
    rospy.init_node('rfreceiver_mode_select')

    modes = {
        1: 'tactile',
        2: 'attended'
    }

    mode_pub = rospy.Publisher(
        '/appctl/mode',
        Mode,
        queue_size = 1
    )

    button_handler = ButtonHandler(modes, mode_pub)

    mode_sub = rospy.Subscriber(
        '/rfreceiver/buttondown',
        Byte,
        button_handler.handle_msg
    )

    rospy.spin()

if __name__=='__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
