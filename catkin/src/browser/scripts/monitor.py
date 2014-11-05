#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import rospy

import appctl_support
from appctl.msg import Mode

BROWSER_PAGE_MONITOR = '/home/lg/bin/chromium-page-monitor'


def main():
    rospy.init_node('browser_page_monitor', anonymous=True)

    modes = rospy.get_param('~modes').split(',')
    window = rospy.get_param('~window', 'lgS0')
    app = rospy.get_param('~app', 'appctl')
    debug_port = rospy.get_param('~debug_port', None)
    names = rospy.get_param('~names', None).split(',')
    rospy.loginfo('running on modes {}'.format(modes))

    cmd = [BROWSER_PAGE_MONITOR]
    cmd.extend(('-a', app))
    cmd.extend(('-w', window))
    if debug_port is not None:
        cmd.extend(('-p', debug_port))
    if names is not None:
        for name in names:
            cmd.extend(('-n', name))

    sanitized_cmd = map(lambda arg: str(arg), cmd)

    mode_handler = appctl_support.ModeHandler(modes, sanitized_cmd)
    rospy.Subscriber('/appctl/mode', Mode, mode_handler.handle_mode_msg)
    rospy.spin()
    mode_handler.shutdown()

if __name__ == '__main__':
    main()
