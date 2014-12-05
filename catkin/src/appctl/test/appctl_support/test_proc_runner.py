#!/usr/bin/env python
PKG = 'appctl'
NAME = 'test_proc_runner'

import rospy
import unittest
import time

from appctl_support import ProcRunner

TEST_CMD = ['/usr/bin/python']


class TestProcRunner(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NAME)

    def test_run_and_shutdown(self):
        runner = ProcRunner(TEST_CMD)
        self.assertFalse(runner.done)

        runner.start()
        self.assertTrue(runner.is_alive())

        # Wait for the subprocess to spin up
        time.sleep(0.1)
        self.assertIsNone(runner.proc.returncode)

        runner.shutdown()
        self.assertTrue(runner.done)

        runner.join()
        self.assertIsNotNone(runner.proc.returncode)
        self.assertFalse(runner.is_alive())

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestProcRunner)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
