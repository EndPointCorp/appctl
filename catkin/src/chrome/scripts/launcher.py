#!/usr/bin/env python

import rospy

import appctl_support
from appctl.msg import Mode
from appctl.srv import Query

B = [
  '/bin/nc', '-l', '9999'
]

def main():
  rospy.init_node('chrome', anonymous=True)

  modes = rospy.get_param('~modes').split(',')

  rospy.loginfo('running on modes {}'.format(modes))

  mode_handler = appctl_support.ModeHandler(modes, B)

  rospy.Subscriber('/appctl/mode', Mode, mode_handler.handle_mode_msg)

  # TODO(mv): check /appctl/query

  rospy.spin()

  mode_handler.shutdown()

if __name__=='__main__':
  main()
