import rospy
import threading
from controller import BaseController
from proc_runner import ProcRunner


class ProcController(BaseController):
    """
    The purpose of ProcController is to start and stop ProcRunners with
    guarantees of thread safety and no waifs.
    """
    def __init__(self, cmd, shell=False, spawn_hooks=[], respawn=True):
        """
        respawn handles whether or not the application shall be automatically
                respawned at all, default is True.

        """
        self.lock = threading.Lock()
        self.cmd = cmd
        self.shell = shell
        self.started = False
        self.watcher = None
        self.spawn_hooks = spawn_hooks
        self.respawn = respawn
        self.start_count = 0
        self.stop_count = 0

    def start(self, *args, **kwargs):
        with self.lock:
            if self.started:
                return
            self.started = True
            self.start_count += 1
            if self.start_count != self.stop_count + 1:
                raise AssertionError('Start/stop count mismatch during start()')
            self.watcher = ProcRunner(self.cmd,
                                      shell=self.shell,
                                      spawn_hooks=self.spawn_hooks,
                                      respawn=self.respawn)
            self.watcher.start()

    def stop(self, *args, **kwargs):
        with self.lock:
            if not self.started:
                return
            self.started = False
            self.stop_count += 1
            if self.stop_count != self.start_count:
                raise AssertionError('Start/stop count mismatch during stop()')
            self.watcher.shutdown()
            self.watcher = None

    def close(self):
        try:
            self.watcher.shutdown()
        except AttributeError:
            pass
        finally:
            self.spawn_hooks = self.watcher = None

    def __del__(self):
        self.close()

    def add_spawn_hook(self, spawn_hook):
        """
        Adds a spawn hook to the current list of spawn hooks

        If there is already a watcher, also calls down to that object
        to add the new spawn hook as well
        """
        self.spawn_hooks.append(spawn_hook)
        if self.watcher:
            self.watcher.add_spawn_hook(spawn_hook)

    def get_pid(self):
        """
        Return process id, or 0 to signify error
        """
        if self.watcher:
            return self.watcher.get_pid()
        return None

    def handle_soft_relaunch(self, *args, **kwargs):
        self.watcher.handle_soft_relaunch()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
