#!/usr/bin/env python
PKG = 'appctl'
NAME = 'test_proc_runner'

import os
import rospy
import unittest
import time

from appctl_support import ProcRunner

TEST_CMD = ['/usr/bin/python']


# http://stackoverflow.com/questions/568271/
def check_pid(pid):
    """ Check For the existence of a unix pid. """
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    else:
        return True


class TestProcRunner(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NAME)

    def test_run_and_shutdown(self):
        runner = ProcRunner(TEST_CMD)
        self.assertFalse(runner.done,
            'Runner must not finish immediately')

        runner.start()
        self.assertTrue(runner.is_alive(),
            'Runner must be alive after start()')

        # Wait for the subprocess to spin up
        time.sleep(0.1)
        self.assertIsNone(runner.proc.returncode,
            'Popen must not have returned too soon')

        pid = runner.proc.pid
        self.assertTrue(check_pid(pid),
            'Popen pid must respond to signal 0 while running')

        runner.shutdown()
        self.assertTrue(runner.done,
            'Runner not done after shutdown()')
        self.assertFalse(check_pid(pid),
            'Popen pid still alive after shutdown()')

        runner.join()
        self.assertIsNotNone(runner.proc.returncode,
            'Popen must have returned after join()')
        self.assertFalse(runner.is_alive(),
            'Runner must not be alive after join()')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestProcRunner)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
