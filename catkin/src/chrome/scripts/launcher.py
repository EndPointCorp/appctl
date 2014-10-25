#!/usr/bin/env python

import rospy

import appctl_support
from appctl.msg import Mode
from appctl.srv import Query

BROWSER_LAUNCHER = '/home/lg/bin/lg-browser'

def main():
  rospy.init_node('chrome', anonymous=True)

  modes = rospy.get_param('~modes').split(',')
  window = rospy.get_param('~window', 'lgS0')
  app = rospy.get_param('~app', 'appctl')
  url = rospy.get_param('~url')
  disable_extensions = rospy.get_param('~disable_extensions', False)

  if rospy.has_param('~debug_port'):
    debug_port = str(rospy.get_param('~debug_port'))
  else:
    debug_port = None

  if rospy.has_param('~browser_bin'):
    browser_bin = rospy.get_param('~browser_bin')
  else:
    browser_bin = None

  rospy.loginfo('running on modes {}'.format(modes))

  cmd = [BROWSER_LAUNCHER]
  cmd.extend(('-w', window))
  cmd.extend(('-a', app))
  cmd.extend(('-u', url))
  if disable_extensions:
    cmd.append('-D')
  if debug_port is not None:
    cmd.extend(('-p', debug_port))
  if browser_bin is not None:
    cmd.extend(('-b', browser_bin))

  mode_handler = appctl_support.ModeHandler(modes, cmd)

  rospy.Subscriber('/appctl/mode', Mode, mode_handler.handle_mode_msg)

  # TODO(mv): check /appctl/query

  rospy.spin()

  mode_handler.shutdown()

if __name__=='__main__':
  main()
