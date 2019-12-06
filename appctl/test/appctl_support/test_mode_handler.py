#!/usr/bin/env python3
import unittest

from appctl_support import ModeHandler
from appctl_support.controller import BaseController
from appctl_msg_defs.msg import Mode

PKG = 'appctl'
NAME = 'test_mode_handler'

TEST_MODES = ["_test_foo", "_test_bar"]


class MockController(BaseController):
    def __init__(self):
        self.started = False

    def start(self):
        self.started = True

    def stop(self):
        self.started = False


class TestModeHandler(unittest.TestCase):
    def setUp(self):
        self.handler = ModeHandler(TEST_MODES, MockController())

    def test_mode_handling(self):
        # init stopped
        self.assertFalse(self.handler.controller.started)

        # active modes
        for mode in TEST_MODES:
            msg = Mode(mode)
            self.handler._handle_mode_msg(msg)

            self.assertTrue(self.handler.controller.started)

        # inactive mode
        msg = Mode('_not_an_active_mode')
        self.handler._handle_mode_msg(msg)

        self.assertFalse(self.handler.controller.started)

    def test_shutdown(self):
        self.handler.controller.start()
        self.assertTrue(self.handler.controller.started)

        self.handler.shutdown()
        self.assertFalse(self.handler.controller.started)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestModeHandler)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
