#!/usr/bin/env python

import os
import rospy
import serial
import subprocess
from std_msgs.msg import Byte

DEVNULL = open(os.devnull, 'w')
CLEAR_BUTTON = 2

def main():
  buttondown_pub = rospy.Publisher(
    '/rfreceiver/buttondown',
    Byte,
    queue_size = 1
  )
  rospy.init_node('rfreceiver')

  receiver = serial.Serial('/dev/promicro16.0', 9600)

  buf = ''

  while not rospy.is_shutdown():
    try:
      button = int(receiver.readline(10).strip())
    except serial.SerialException as e:
      print e
      break

    # TODO(mv): move this to a more general portal system ros interface
    if button == CLEAR_BUTTON:
      subprocess.call(
        ['/home/lg/bin/lg-run-bg', 'pkill chrome'],
        stdout=DEVNULL,
        stderr=DEVNULL
      )

    buttondown_pub.publish(Byte(button))

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
