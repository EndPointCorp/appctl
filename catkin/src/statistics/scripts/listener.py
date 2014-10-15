#!/usr/bin/env python

import rospy
from statistics.msg import StatsD

def handle(msg):
  print msg

def listen():
  rospy.init_node('statistics')
  rospy.Subscriber('statistics/render', StatsD, handle)

  rospy.spin()

if __name__=='__main__':
  listen()

