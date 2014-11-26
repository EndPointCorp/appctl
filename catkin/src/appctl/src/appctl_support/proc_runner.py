import rospy
import threading
import os
import subprocess
import time

DEVNULL = open(os.devnull, 'rw')

RESPAWN_POLL_DELAY = 1


class ProcRunner(threading.Thread):
    """
    A Thread that launches and manages a subprocess.
    """
    def __init__(self, cmd):
        super(self.__class__, self).__init__()
        self.cmd = cmd
        self.cmd_str = ' '.join(cmd)
        self.done = False
        self.proc = None

    def run(self):
        if self.done:
            rospy.logwarn('tried to run a finished {}'.format(self.__name__))
            return

        def _run():
            self.proc = subprocess.Popen(
                self.cmd,
                stdin=DEVNULL,
                stdout=DEVNULL,
                stderr=DEVNULL
            )

        _run()
        while not self.done:
            if self.proc.returncode is not None:
                rospy.logwarn('respawning process: {}'.format(self.cmd_str))
                _run()

            time.sleep(RESPAWN_POLL_DELAY)
            self.proc.poll()

    def shutdown(self, *args, **kwargs):
        self.done = True
        if self.proc is not None and self.proc.returncode is None:
            self.proc.terminate()
            self.proc.communicate()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
