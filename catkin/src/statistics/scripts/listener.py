#!/usr/bin/env python

import rospy
import socket
from statistics.msg import StatsD

class StatsDHandler():
  def __init__(self, statsd_host, statsd_port):
    self.statsd_target = (statsd_host, statsd_port)
    self.sock = socket.socket(
      socket.AF_INET,
      socket.SOCK_DGRAM
    )

  def _send_msg(self, statsd_msg):
    self.sock.sendto(statsd_msg, self.statsd_target)

  def handle(self, msg):
    statsd_msg = '{name}:{value}|{type}|@{rate}'.format(
      name=msg.name,
      value=msg.value,
      type=msg.type,
      rate=msg.rate
    )
    print statsd_msg
    self._send_msg(statsd_msg)

def listen():
  rospy.init_node('statistics')

  statsd_host = rospy.get_param(
    '~statsd_host',
    'lg-head'
  )
  statsd_port = rospy.get_param(
    '~statsd_port',
    8125
  )

  handler = StatsDHandler(statsd_host, statsd_port)

  rospy.Subscriber('statistics/render', StatsD, handler.handle)

  rospy.spin()

if __name__=='__main__':
  listen()

