#!/usr/bin/env python

import rospy
import threading
import os
import signal
import subprocess
import time

from appctl.msg import Mode
from appctl.srv import Query

DEVNULL = open(os.devnull, 'rw')

POLL_DELAY = 1

"""
B = [
  '/home/lg/bin/lg-browser',
  '-a', 'roschrome',
  '-w', 'lgS0',
  '-u', 'http://youtube.com'
]
"""

B = [
  'nc', '-l', '9999'
]

class WatcherThread(threading.Thread):
  def __init__(self, cmd):
    super(self.__class__, self).__init__()
    self.cmd = cmd
    self.done = False
    self.proc = None

  def run(self):
    if self.done:
      rospy.warn('tried to run a finished {}'.format(self.__name__))
      return

    def _run():
      self.proc = subprocess.Popen(self.cmd, stdin=DEVNULL, stdout=DEVNULL, stderr=DEVNULL)

    _run()
    while not self.done:
      if self.proc.returncode is not None:
        rospy.logwarn('respawning process')
        _run()

      time.sleep(POLL_DELAY)
      self.proc.poll()

  def shutdown(self, *args, **kwargs):
    self.done = True
    if self.proc is not None and self.proc.returncode is None:
      self.proc.kill()
      self.proc.communicate()

class AppController():
  def __init__(self, cmd):
    self.cmd = cmd
    self.started = False
    self.proc = None

  def start(self, *args, **kwargs):
    if self.started:
      return

    rospy.loginfo('starting WatcherThread')

    self.watcher = WatcherThread(self.cmd)
    self.watcher.start()
    self.started = True

  def stop(self, *args, **kwargs):
    if not self.started:
      return

    rospy.loginfo('stopping WatcherThread')

    self.watcher.shutdown()
    self.started = False

class ModeSelector():
  def __init__(self, modes):
    self.modes = modes
    self.controller = AppController(B)

  def mode_change(self, mode_msg):
    mode = mode_msg.mode
    rospy.loginfo('got mode {}'.format(mode))

    if mode in self.modes:
      self.controller.start()
    else:
      self.controller.stop()

  def shutdown(self):
    self.controller.stop()

def main():
  rospy.init_node('chrome', anonymous=True)

  modes = rospy.get_param('~modes').split(',')

  rospy.loginfo('running on modes {}'.format(modes))

  selector = ModeSelector(modes)

  rospy.Subscriber('/appctl/mode', Mode, selector.mode_change)

  # TODO(mv): check /appctl/query

  rospy.spin()

  selector.shutdown()

if __name__=='__main__':
  main()
