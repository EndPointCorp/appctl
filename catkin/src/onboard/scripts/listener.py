#!/usr/bin/env python
# This app sends messages to dbus signalling visibility of the onboard on-screen keyboard.

import os
import rospy
from std_msgs.msg import Bool


NODE_NAME = "onboard_listener"
CONFIG_FILE = "~config"  # taken from the portal.launch parameter value
DEFAULT_CONFIG_FILE = os.getenv("HOME") + "/etc/onboard.dconf"


def callback(data):
    config_file = rospy.get_param(CONFIG_FILE, DEFAULT_CONFIG_FILE)
    rospy.loginfo("%s using '%s' conf file." % (NODE_NAME, config_file))
    # TODO:
    #   use python dbus bindings instead of dbus cli
    if data.data:
        os.system('dconf load /org/onboard/ < %s' % config_file)
        os.system('dbus-send --type=method_call --dest=org.onboard.Onboard /org/onboard/Onboard/Keyboard org.onboard.Onboard.Keyboard.Show')
    else:
        os.system('dbus-send --type=method_call --dest=org.onboard.Onboard /org/onboard/Onboard/Keyboard org.onboard.Onboard.Keyboard.Hide')


def listener():
    rospy.init_node(NODE_NAME)
    rospy.Subscriber('/onboard/visibility', Bool, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4