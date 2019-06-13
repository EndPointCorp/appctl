#!/usr/bin/env python

"""
This node listens for appctl/Mode changes and toggles an evdev_teleport
receiver accordingly.
"""

import rospy
from appctl.msg import Mode
from std_msgs.msg import Bool


class ModeHandler:
    def __init__(self, modes, activation_pub):
        self.modes = modes
        self.activation_pub = activation_pub

    def handle_msg(self, msg):
        if msg.mode in self.modes:
            self.activation_pub.publish(data=True)
        else:
            self.activation_pub.publish(data=False)


def main():
    rospy.init_node('evdev_teleport_switcher', anonymous=True)

    modes = rospy.get_param('~modes').split(',')
    activation_node = '/evdev_teleport/activation/{}'.format(
        rospy.get_param('~activation_node')
    )

    activation_pub = rospy.Publisher(
        activation_node,
        Bool,
        queue_size=1,
        latch=True
    )

    mode_handler = ModeHandler(modes, activation_pub)

    mode_sub = rospy.Subscriber('/appctl/mode', Mode, mode_handler.handle_msg)

    rospy.spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
