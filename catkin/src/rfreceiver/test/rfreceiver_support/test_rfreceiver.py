#!/usr/bin/env python
PKG='rfreceiver'
NAME='test_rfreceiver.py'

import rospy
import unittest
import time

class TestRfreceiver(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestRfreceiver)
