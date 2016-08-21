import rospy
import threading
import os
import signal
import subprocess
import sys

DEFAULT_RESPAWN_DELAY = 1.0


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
                 respawn=True):
        """
        respawn handles whether or not the application shall be automatically
                respawned at all, default is True.

        """
        super(self.__class__, self).__init__()
        self.cmd = cmd
        self.shell = shell
        self.respawn_delay = respawn_delay
        self.respawn = respawn
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

    def __del__(self):
        self.shutdown()

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
                                     close_fds=True)
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
            except:
                rospy.logerr("Caught an Exception while running a spawn hook!")
                # Log the traceback.
                rospy.logerr(sys.exc_info()[2])

        map(run_spawn_hook, self._spawn_hooks)

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

    def shutdown(self, *args, **kwargs):
        """
        Finish this Thread by killing the process and marking completion.
        """
        with self.lock:
            self.done = True
            self._kill_proc()

    def add_spawn_hook(self, spawn_hook):
        """
        Add a single spawn hook to any existing spawn hooks

        spawn_hook must be callable
        """
        if not callable(spawn_hook):
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
