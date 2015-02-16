import rospy
import threading
import os
import signal
import subprocess
import time

DEVNULL = open(os.devnull, 'rw')

DEFAULT_RESPAWN_DELAY = 1.0


class ProcRunner(threading.Thread):
    """
    A Thread that launches and manages a subprocess.
    """
    def __init__(self, cmd, respawn_delay=DEFAULT_RESPAWN_DELAY):
        super(self.__class__, self).__init__()
        self.cmd = cmd
        self.respawn_delay = respawn_delay
        self.spawn_count = 0
        self.cmd_str = ' '.join(cmd)
        self.done = False
        self.proc = None

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
            rospy.logwarn(
                'respawn #{} for process: {}'.format(
                    self.spawn_count, self.cmd_str))

        self.proc = subprocess.Popen(
            self.cmd,
            stdin=DEVNULL,
            stdout=DEVNULL,
            stderr=DEVNULL,
            preexec_fn=os.setsid
        )
        self.spawn_count += 1
        rospy.loginfo('started process {}'.format(self.proc.pid))

    def run(self):
        """
        Begin managing the process.
        """
        if self.done:
            rospy.logwarn('tried to run a finished {}'.format(self.__name__))
            return

        while not self.done:
            if not self._proc_is_alive():
                self._start_proc()

            time.sleep(self.respawn_delay)
            self.proc.wait()

    def shutdown(self, *args, **kwargs):
        """
        Finish this Thread by killing the process and marking completion.
        """
        self.done = True
        self._kill_proc()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
