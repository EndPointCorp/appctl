#!/usr/bin/env python
PKG = 'appctl'
NAME = 'test_proc_controller'

import rospy
import unittest
import time

from appctl_support import ProcController

TEST_CMD = ['/usr/bin/python']


class TestProcController(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NAME)
        self.controller = ProcController(TEST_CMD)

    def test_start_and_stop(self):
        self.assertFalse(self.controller.started,
            'Controller must not start on init')

        self.controller.start()
        self.assertTrue(self.controller.started,
            'Controller must be started on start()')

        # Wait for the process to spin up
        time.sleep(0.1)

        watcher = self.controller.watcher
        self.assertTrue(watcher.is_alive(),
            'Process watcher must be alive while started')

        self.controller.stop()
        self.assertFalse(self.controller.started,
            'Controller must not be started after stop()')

        watcher.join()
        self.assertFalse(watcher.is_alive(),
            'Process watcher must not be alive after joined')

    def test_redundant_start(self):
        self.controller.start()

        watcher = self.controller.watcher

        self.controller.start()

        self.assertIs(watcher, self.controller.watcher,
            'Stray process watcher on redundant start')

    def tearDown(self):
        self.controller.stop()


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestProcController)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
