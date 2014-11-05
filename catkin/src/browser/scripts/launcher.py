#!/usr/bin/env python

import rospy

import appctl_support
from appctl.msg import Mode
from appctl.srv import Query

BROWSER_LAUNCHER = '/home/lg/bin/lg-browser'

def main():
  rospy.init_node('browser', anonymous=True)

  modes = rospy.get_param('~modes').split(',')
  window = rospy.get_param('~window', 'lgS0')
  app = rospy.get_param('~app', 'appctl')
  url = rospy.get_param('~url')

  disable_extensions = rospy.get_param('~disable_extensions', False)
  disable_kiosk = rospy.get_param('~disable_kiosk', False)

  debug_port = rospy.get_param('~debug_port', None)
  browser_bin = rospy.get_param('~browser_bin', None)
  scale_factor = rospy.get_param('~scale_factor', None)
  user_agent = rospy.get_param('~user_agent', None)
  extensions = rospy.get_param('~extensions', None)

  rospy.loginfo('running on modes {}'.format(modes))

  cmd = [BROWSER_LAUNCHER]
  cmd.extend(('-w', window))
  cmd.extend(('-a', app))
  cmd.extend(('-u', url))
  if disable_extensions:
    cmd.append('-D')
  if disable_kiosk:
    cmd.append('-K')
  if debug_port is not None:
    cmd.extend(('-p', debug_port))
  if browser_bin is not None:
    cmd.extend(('-b', browser_bin))
  if scale_factor is not None:
    cmd.extend(('-s', scale_factor))
  if user_agent is not None:
    cmd.extend(('-U', user_agent))
  if extensions is not None:
    for extension in extensions.split(','):
      cmd.extend(('-E', extension))

  sanitized_cmd = map(lambda arg: str(arg), cmd)

  mode_handler = appctl_support.ModeHandler(modes, sanitized_cmd)

  rospy.Subscriber('/appctl/mode', Mode, mode_handler.handle_mode_msg)

  # TODO(mv): check /appctl/query

  rospy.spin()

  mode_handler.shutdown()

if __name__=='__main__':
  main()
