#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Byte

def main():
  buttondown_pub = rospy.Publisher('/rfreceiver/buttondown', Byte, queue_size = 1)
  rospy.init_node('rfreceiver')

  receiver = serial.Serial('/dev/ttyACM0', 9600)

  buf = ''

  while not rospy.is_shutdown():
    try:
      msg = int(receiver.readline(10).strip())
    except serial.SerialException as e:
      print e
      break

    buttondown_pub.publish(Byte(msg))

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
