#!/usr/bin/env python
PKG='onboard'
NAME='test_onboard.py'

import rospy
import unittest
import time

class TestOnboard(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestOnboard)
