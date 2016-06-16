#!/usr/bin/env python
import unittest
import time

from appctl_support import ProcController

PKG = 'appctl'
NAME = 'test_proc_controller'

TEST_CMD = ['sleep', '5']


class TestProcController(unittest.TestCase):

    def setUp(self):
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

    def test_respawn_flag_forward_to_runner(self):
        """
        Test forwarding of the respawn flag setting and associated
        behaviour. Both for True (default value) and False from the
        higher-lever proc_controller POV.

        """
        self.assertTrue(self.controller.respawn, "The 'respawn' flag is by default True.")
        self.controller.start()
        self.assertTrue(self.controller.watcher.respawn, "The 'respawn' flag is by default True.")
        self.controller.stop()
        time.sleep(0.1)
        self.controller = ProcController(TEST_CMD, respawn=False)
        self.assertFalse(self.controller.respawn, "The 'respawn' flag is False now.")
        self.controller.start()
        self.assertFalse(self.controller.watcher.respawn, "The 'respawn' flag is False now.")
        self.controller.stop()
        time.sleep(0.1)

    def tearDown(self):
        self.controller.stop()


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestProcController)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
