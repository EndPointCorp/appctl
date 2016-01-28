import rospy
import threading
from controller import BaseController
from proc_runner import ProcRunner


class ProcController(BaseController):
    """
    Controls startup and shutdown of a ProcRunner.
    """
    def __init__(self, cmd, shell=False, spawn_hooks=[]):
        self.cmd = cmd
        self.shell = shell
        self.started = False
        self.watcher = None
        self.spawn_hooks = spawn_hooks
        self.status_lock = threading.Lock()

        # Always stop on rospy shutdown.
        rospy.on_shutdown(self.stop)

    def start(self, *args, **kwargs):
        with self.status_lock:
            if self.started:
                return
            self.started = True
            self.watcher = ProcRunner(self.cmd, shell=self.shell,
                                      spawn_hooks=self.spawn_hooks)
            self.watcher.daemon = False
            self.watcher.start()

    def stop(self, *args, **kwargs):
        with self.status_lock:
            if not self.started:
                return
            self.started = False
            self.watcher.shutdown()
            self.watcher = None

    def add_spawn_hook(self, spawn_hook):
        """
        Adds a spawn hook to the current list of spawn hooks

        If there is already a watcher, also calls down to that object
        to add the new spawn hook as well
        """
        self.spawn_hooks.append(spawn_hook)
        if self.watcher:
            self.watcher.add_spawn_hook(spawn_hook)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
