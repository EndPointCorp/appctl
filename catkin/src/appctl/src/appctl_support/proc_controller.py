import rospy
import threading
from controller import BaseController
from proc_runner import ProcRunner


class ProcController(BaseController):
    """
    Controls startup and shutdown of a ProcRunner.
    """
    def __init__(self, cmd):
        self.cmd = cmd
        self.started = False
        self.watcher = None
        self.status_lock = threading.Lock()

        # Always stop on rospy shutdown.
        rospy.on_shutdown(self.stop)

    def start(self, *args, **kwargs):
        with self.status_lock:
            if self.started:
                return
            self.started = True

        rospy.logdebug('starting ProcRunner')

        self.watcher = ProcRunner(self.cmd)
        self.watcher.start()

    def stop(self, *args, **kwargs):
        with self.status_lock:
            if not self.started:
                return
            self.started = False

        rospy.logdebug('stopping ProcRunner')

        self.watcher.shutdown()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
