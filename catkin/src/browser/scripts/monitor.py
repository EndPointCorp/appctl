#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import rospy

import appctl_support

BROWSER_PAGE_MONITOR = '/home/lg/bin/chromium-page-monitor'


def main():
    rospy.init_node('browser_page_monitor', anonymous=True)

    modes = rospy.get_param('~modes').split(',')
    app = rospy.get_param('~app', 'appctl')
    debug_port = rospy.get_param('~debug_port', None)
    names = rospy.get_param('~names', None)
    rospy.loginfo('running on modes {}'.format(modes))

    cmd = [BROWSER_PAGE_MONITOR]
    cmd.extend(('-a', app))
    if debug_port is not None:
        cmd.extend(('-p', debug_port))
    if names is not None:
        for name in names.split(','):
            cmd.extend(('-n', name))

    sanitized_cmd = map(lambda arg: str(arg), cmd)

    proc_controller = appctl_support.ProcController(sanitized_cmd)
    mode_handler = appctl_support.ModeHandler(modes, proc_controller)
    mode_handler.begin_handling_modes()
    rospy.spin()
    mode_handler.shutdown()

if __name__ == '__main__':
    main()
