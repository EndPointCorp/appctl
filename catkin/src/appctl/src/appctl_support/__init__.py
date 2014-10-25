#!/usr/bin/env python

import rospy
import threading
import os
import subprocess
import time

DEVNULL = open(os.devnull, 'rw')

RESPAWN_POLL_DELAY = 1

class WatcherThread(threading.Thread):
  """
  A Thread that launches and manages a subprocess.
  """
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

      time.sleep(RESPAWN_POLL_DELAY)
      self.proc.poll()

  def shutdown(self, *args, **kwargs):
    self.done = True
    if self.proc is not None and self.proc.returncode is None:
      self.proc.terminate()
      self.proc.communicate()

class AppController():
  """
  Controls startup and shutdown of a WatcherThread.
  """
  def __init__(self, cmd):
    self.cmd = cmd
    self.started = False
    self.proc = None

  def start(self, *args, **kwargs):
    if self.started:
      return

    rospy.logdebug('starting WatcherThread')

    self.watcher = WatcherThread(self.cmd)
    self.watcher.start()
    self.started = True

  def stop(self, *args, **kwargs):
    if not self.started:
      return

    rospy.logdebug('stopping WatcherThread')

    self.watcher.shutdown()
    self.started = False

class ModeHandler():
  """
  Wraps an AppController and provides a mode message handler.
  """
  def __init__(self, modes, cmd):
    self.modes = modes
    self.controller = AppController(cmd)

  def handle_mode_msg(self, mode_msg):
    mode = mode_msg.mode
    rospy.logdebug('got mode {}'.format(mode))

    if mode in self.modes:
      self.controller.start()
    else:
      self.controller.stop()

  def shutdown(self):
    self.controller.stop()
