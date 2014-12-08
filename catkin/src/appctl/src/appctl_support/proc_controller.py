import rospy
from controller import BaseController
from proc_runner import ProcRunner


class ProcController(BaseController):
    """
    Controls startup and shutdown of a ProcRunner.
    """
    def __init__(self, cmd):
        self.cmd = cmd
        self.started = False
        self.proc = None

    def start(self, *args, **kwargs):
        if self.started:
            return

        rospy.logdebug('starting ProcRunner')

        self.watcher = ProcRunner(self.cmd)
        self.watcher.start()
        self.started = True

    def stop(self, *args, **kwargs):
        if not self.started:
            return

        rospy.logdebug('stopping ProcRunner')

        self.watcher.shutdown()
        self.started = False

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
