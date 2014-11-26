#!/usr/bin/env python
# This app sends messages to dbus signalling visibility of the onboard on-screen keyboard.

import os
import rospy
from std_msgs.msg import Bool


def callback(data):
    # TODO: use python dbus bindings instead of dbus cli
    if data.data:
        os.system('dconf load /org/onboard/ < ' + os.getenv('HOME') + '/etc/onboard.dconf')
        os.system('dbus-send --type=method_call --dest=org.onboard.Onboard /org/onboard/Onboard/Keyboard org.onboard.Onboard.Keyboard.Show')
    else:
        os.system('dbus-send --type=method_call --dest=org.onboard.Onboard /org/onboard/Onboard/Keyboard org.onboard.Onboard.Keyboard.Hide')


def listener():
    rospy.init_node('onboard_listener')
    rospy.Subscriber('/onboard/visibility', Bool, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
