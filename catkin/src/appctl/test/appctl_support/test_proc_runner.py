#!/usr/bin/env python
PKG = 'appctl'
NAME = 'test_proc_runner'

import os
import rospy
import unittest
import time

from appctl_support import ProcRunner

TEST_CMD = ['/usr/bin/python']
GRACE_DELAY = 0.1 # seconds


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
        self.runner = ProcRunner(TEST_CMD)

    def tearDown(self):
        self.runner.shutdown()

    def test_proc_is_alive(self):
        self.assertFalse(self.runner._proc_is_alive(),
            'Process must not be alive before start()')

        self.runner.start()

        time.sleep(GRACE_DELAY)
        self.assertTrue(self.runner._proc_is_alive(),
            'Process must be alive after start()')

        self.runner.shutdown()
        self.runner.join()
        self.assertFalse(self.runner._proc_is_alive(),
            'Process must not be alive after shutdown()')


    def test_startup(self):
        self.runner.start()
        self.assertTrue(self.runner.is_alive(),
            'Runner must be alive after start()')

    def test_shutdown(self):
        self.runner.start()
        time.sleep(GRACE_DELAY)

        pid = self.runner.proc.pid
        self.assertIsNotNone(pid,
            'Must get a pid after start()')

        self.runner.shutdown()
        self.runner.join()

        self.assertFalse(check_pid(pid),
            'Process must not respond to sig0 after shutdown()')

    def test_kill_proc(self):
        self.runner.start()

        time.sleep(GRACE_DELAY)

        pid = self.runner.proc.pid
        self.assertTrue(check_pid(pid),
            'Must have a pid to start with')

        self.runner._kill_proc()
        self.runner.proc.wait()

        self.assertFalse(check_pid(pid),
            'Process must be dead')

    def test_respawn(self):
        self.runner.start()

        time.sleep(GRACE_DELAY)

        first_pid = self.runner.proc.pid
        self.runner._kill_proc()

        time.sleep(self.runner.respawn_delay + GRACE_DELAY)

        self.assertTrue(self.runner._proc_is_alive(),
            'Process must respawn after death')

        second_pid = self.runner.proc.pid
        self.assertNotEqual(first_pid, second_pid,
            'Must have a different pid after respawn')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestProcRunner)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
