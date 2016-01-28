#!/usr/bin/env python

import rospy
from appctl_support import ProcController
from appctl_support import ModeHandler


if __name__ == '__main__':
    rospy.init_node('spin_test_node', anonymous=True)

    modes = ['test_appctl']
    cmd = ['/bin/sh', '-c', 'sleep 23023023']

    proc_controller = ProcController(cmd)
    mode_handler = ModeHandler(modes, proc_controller)
    mode_handler.begin_handling_modes()

    rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
