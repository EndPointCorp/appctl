#!/usr/bin/env python

import rospy
import shlex
import appctl_support

def main():
    rospy.init_node('onboard_manager', anonymous=True)

    modes = rospy.get_param('~modes').split(',')
    display = rospy.get_param('~display', ':0')
    rospy.loginfo('running on modes {}'.format(modes))
    rospy.loginfo('running with display number {}'.format(display))

    cmd = ['/home/lg/bin/onboard_wrapper.sh'] #TODO (WZ) pass display parameter here
    rospy.loginfo("cmd = %s" % cmd)

    proc_controller = appctl_support.ProcController(cmd)
    mode_handler = appctl_support.ModeHandler(modes, proc_controller)

    mode_handler.begin_handling_modes()

    rospy.spin()

    mode_handler.shutdown()

if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
