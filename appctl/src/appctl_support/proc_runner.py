import rospy
from functools import partial
import atexit
import threading
import os
import signal
import subprocess
import sys
import weakref
from collections.abc import Callable

DEFAULT_RESPAWN_DELAY = 1.0

_refs = []


def _get_runner_refs():
    global _refs
    return _refs


def _add_cleanup_ref(runner):
    refs = _get_runner_refs()
    ref = weakref.ref(runner)
    refs.append(ref)


def _cleanup_all_runners():
    refs = _get_runner_refs()
    list(map(_cleanup_ref, refs))
    del refs[:]


def _cleanup_ref(ref):
    runner = ref()
    if runner is None:
        return
    runner._shutdown()


atexit.register(_cleanup_all_runners)


class ProcRunner(threading.Thread):
    """
    A Thread that launches and manages a subprocess.

    """
    def __init__(self,
                 cmd,
                 respawn_delay=DEFAULT_RESPAWN_DELAY,
                 shell=False,
                 spawn_hooks=[],
                 respawn_limit=-1,
                 respawn=True,
                 env=None,
                 stdout=None,
                 stderr=None):
        """
        respawn handles whether or not the application shall be automatically
                respawned at all, default is True.

        """
        super(self.__class__, self).__init__()
        self.cmd = cmd
        self.shell = shell
        self.respawn_delay = respawn_delay
        self.respawn = respawn
        self.env = env
        self.stdout = stdout
        self.stderr = stderr
        self.lock = threading.Lock()
        self.spawn_count = 0
        if not shell:
            self.cmd_str = ' '.join(cmd)
        self.done = False
        self.proc = None
        self._spawn_hooks = []
        self.respawn_limit = respawn_limit
        # add all spawn hooks passed
        for spawn_hook in spawn_hooks:
            self.add_spawn_hook(spawn_hook)

        # Must be a daemon thread to ensure clean shutdown.
        self.daemon = True

        # Register for cleanup.
        _add_cleanup_ref(self)

    def __del__(self):
        self._shutdown()

    def _kill_proc(self):
        """
        Attempts to kill the process group with SIGTERM.
        """
        if self.proc is None:
            return

        pid = self.proc.pid

        rospy.loginfo('sending SIGTERM to process group {}'.format(pid))

        try:
            os.killpg(pid, signal.SIGTERM)
        except OSError:
            rospy.logwarn('process group {} did not exist'.format(pid))
        finally:
            self.proc.wait()
            self.proc = None

    def _reached_respawn_limit(self):
        """
        Introduces possibility to give up on respawning of a process
        """
        if (self.respawn_limit == -1):
            rospy.logdebug("Respawn limit set to infinity")
            return False
        elif (self.respawn_limit > 0) and (self.spawn_count >= self.respawn_limit):
            rospy.logwarn("Respawn limit of %s reached - not launching application %s" % (self.respawn_limit, self.cmd_str))
            return True

    def _start_proc(self):
        """
        Starts or restarts the process and checks whether respawn limit was reached
        """
        if self._reached_respawn_limit():
            return False
        if self.spawn_count > 0:
            rospy.logwarn('Respawn #{} for process: {}'.format(
                self.spawn_count, self.cmd_str))

        self.proc = subprocess.Popen(self.cmd,
                                     preexec_fn=os.setsid,
                                     shell=self.shell,
                                     close_fds=True,
                                     stdout=self.stdout,
                                     stderr=self.stderr,
                                     env=self.env)
        self._run_spawn_hooks()
        self.spawn_count += 1
        rospy.loginfo("Launched '{}' with pid {}".format(self.cmd, self.proc.pid))

    def _run_spawn_hooks(self):
        """
        Calls all _spawn_hooks if any exist
        """
        def run_spawn_hook(hook):
            try:
                hook()
            except Exception:
                rospy.logerr("Caught an Exception while running a spawn hook!")
                # Log the traceback.
                rospy.logerr(sys.exc_info()[2])

        list(map(run_spawn_hook, self._spawn_hooks))

    def run(self):
        """
        Begin managing the process.
        """
        while True:
            with self.lock:
                if self.done:
                    return
                self._start_proc()
            try:
                self.proc.wait()
            except AttributeError:
                # in this case, self.proc has been dereferenced by _kill_proc()
                pass
            if not self.respawn:
                return
            if not self.done:
                rospy.sleep(self.respawn_delay)

    def _shutdown(self):
        """
        Internal, unlocked shutdown procedure.

        This should only be used upon destruction.
        """
        self.done = True
        self._kill_proc()
        self._spawn_hooks = []

    def shutdown(self, *args, **kwargs):
        """
        Finish this Thread by killing the process and marking completion.
        """
        with self.lock:
            self._shutdown()

    def add_spawn_hook(self, spawn_hook):
        """
        Add a single spawn hook to any existing spawn hooks

        spawn_hook must be callable
        """
        if not isinstance(spawn_hook, Callable):
            raise TypeError("Passed a non-callable object as a spawn hook")
        self._spawn_hooks.append(spawn_hook)

    def get_pid(self):
        """
        Return process id, or 0 to signify error
        """
        if self.proc:
            return self.proc.pid
        return None

    def handle_soft_relaunch(self, *args, **kwargs):
        self._kill_proc()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
