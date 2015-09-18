import rospy
import threading
import os
import signal
import subprocess
import sys
from psutil import Process, NoSuchProcess

DEVNULL = open(os.devnull, 'rw')

DEFAULT_RESPAWN_DELAY = 1.0


class ProcRunner(threading.Thread):
    """
    A Thread that launches and manages a subprocess.
    """
    def __init__(self, cmd, respawn_delay=DEFAULT_RESPAWN_DELAY, shell=False,
                 spawn_hooks=[]):
        super(self.__class__, self).__init__()
        self.cmd = cmd
        self.shell = shell
        self.respawn_delay = respawn_delay
        self.spawn_count = 0
        if not shell:
            self.cmd_str = ' '.join(cmd)
        self.done = False
        self.proc = None
        self._spawn_hooks = []
        # add all spawn hooks passed
        for spawn_hook in spawn_hooks:
            self.add_spawn_hook(spawn_hook)

    def _proc_is_alive(self):
        """
        Returns True if the process is alive and running.
        """
        if self.proc is not None:
            try:
                os.kill(self.proc.pid, 0)
            except OSError:
                return False
            else:
                return True
        else:
            return False

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

    def _start_proc(self):
        """
        Starts or restarts the process.
        """
        if self.spawn_count > 0:
            rospy.logwarn('respawn #{} for process: {}'.format(
                self.spawn_count, self.cmd_str))

        rospy.loginfo("Launching command '%s' with shell='%s'" % (self.cmd, self.shell))
        self.proc = subprocess.Popen(
            self.cmd,
            stdin=DEVNULL,
            stdout=DEVNULL,
            stderr=DEVNULL,
            preexec_fn=os.setsid,
            shell=self.shell,
            close_fds=True
        )
        self._spawn()
        self.spawn_count += 1
        rospy.loginfo('started process {}'.format(self.proc.pid))

    def _spawn(self):
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

    def _proc_is_zombie(self):
        """
        Use psutil to check status of current process to see if it's a zombie
        """
        try:
            proc = Process(self.proc.pid)
            if proc.status == 'zombie':
                return True
        except NoSuchProcess:
            return True # process is actually dead
        # not a zombie
        return False

    def _proc_has_zombie_children(self):
        """
        Use psutil to get_children() and check to make sure that none
        of their statuses are 'zombie'
        """
        try:
            proc = Process(self.proc.pid)
            for child in proc.get_children():
                if child.status == 'zombie':
                    return True
        except NoSuchProcess:
            return True # parent process actually dead
        # No zombie found
        return False

    def run(self):
        """
        Begin managing the process.
        """
        if self.done:
            rospy.logwarn('tried to run a finished ProcRunner')
            return

        while not self.done:
            if not self._proc_is_alive():
                self._start_proc()
            elif self._proc_is_zombie() or self._proc_has_zombie_children():
                self._kill_proc()
                self._start_proc()

            rospy.sleep(self.respawn_delay)
            self.proc.wait()

    def shutdown(self, *args, **kwargs):
        """
        Finish this Thread by killing the process and marking completion.
        """
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

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
