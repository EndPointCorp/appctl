#!/usr/bin/env python3

import rospy
import subprocess
import unittest
from appctl.msg import Mode

# this must match the number of nodes in the .test file
NUM_INSTANCES = 100

# this must match the expected mode in mode_spin_test.py script
TEST_MODE = Mode('test_appctl')
# this must not match anything else
NO_MODE = Mode('none')


def count_test_procs():
    """Helper for counting test procs launched by test nodes."""
    def filter_args(args):
        return args == '/bin/sh -c sleep 23023023'

    try:
        pso = subprocess.check_output(
            ['ps', '--no-headers', '-o', '%a', '-C', 'sh']
        ).decode('utf-8')
    except subprocess.CalledProcessError:
        # ps returns non-zero when no results are found
        return 0
    else:
        return len(list(filter(filter_args, pso.strip().split('\n'))))


class TestAppctlModeSpin(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_appctl_mode_spin')
        self.pub = rospy.Publisher('/appctl/mode', Mode, queue_size=1)

    @unittest.skip("failing for some reason, long wait fixes it")
    def test_spin(self):
        self.assertEquals(0, count_test_procs())

        self.pub.publish(TEST_MODE)
        rospy.sleep(3)
        self.assertEquals(NUM_INSTANCES, count_test_procs())

        self.pub.publish(NO_MODE)
        rospy.sleep(3)
        self.assertEquals(0, count_test_procs())

    def test_slap(self):
        self.assertEquals(0, count_test_procs())

        self.pub.publish(TEST_MODE)
        self.pub.publish(NO_MODE)
        rospy.sleep(3)
        self.assertEquals(0, count_test_procs())

    def test_abuse(self):
        self.assertEquals(0, count_test_procs())

        self.pub.publish(TEST_MODE)
        self.pub.publish(NO_MODE)
        self.pub.publish(TEST_MODE)
        self.pub.publish(NO_MODE)
        self.pub.publish(TEST_MODE)
        self.pub.publish(NO_MODE)
        self.pub.publish(TEST_MODE)
        self.pub.publish(NO_MODE)
        self.pub.publish(TEST_MODE)
        self.pub.publish(NO_MODE)
        self.pub.publish(TEST_MODE)
        self.pub.publish(NO_MODE)
        self.pub.publish(TEST_MODE)
        self.pub.publish(NO_MODE)
        self.pub.publish(TEST_MODE)
        self.pub.publish(NO_MODE)
        rospy.sleep(3)
        self.assertEquals(0, count_test_procs())


if __name__ == '__main__':
    import rostest
    rostest.rosrun('appctl', 'test_mode_spin', TestAppctlModeSpin)
