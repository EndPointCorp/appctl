#!/usr/bin/env python3
import os
import unittest
import gc
import weakref

import rospy
from appctl_support import ProcRunner
from collections.abc import Callable


PKG = 'appctl'
NAME = 'test_proc_runner'

TEST_CMD = ['sleep', '5']
GRACE_DELAY = 0.5  # seconds


# http://stackoverflow.com/questions/568271/
def check_pid(pid):
    """ Check For the existence of a unix pid. """
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    else:
        return True


class MockSpawnHandler(object):
    def __init__(self):
        self.spawns = 0

    def __call__(self):
        self.spawns += 1


class MockBrokenSpawnHandler(object):
    def __init__(self):
        return

    def __call__(self):
        raise Exception("I was born to fail.")


class MockInvalidSpawnHandler(object):
    def __init__(self):
        assert not isinstance(self, Callable)


class TestProcRunner(unittest.TestCase):

    def setUp(self):
        self.runner = ProcRunner(TEST_CMD)

    def tearDown(self):
        self.runner.shutdown()
        if self.runner.is_alive():
            self.runner.join()

    def test_startup(self):
        self.runner.start()
        self.assertTrue(self.runner.is_alive(), 'Runner must be alive after start()')

    def test_shutdown(self):
        self.runner.start()
        rospy.sleep(GRACE_DELAY)
        pid = self.runner.proc.pid
        self.assertIsNotNone(pid, 'Must get a pid after start()')
        self.runner.shutdown()
        self.runner.join()
        self.assertFalse(check_pid(pid), 'Process must not respond to sig0 after shutdown()')

    def test_kill_proc(self):
        self.runner.start()
        rospy.sleep(GRACE_DELAY)
        pid = self.runner.proc.pid
        self.assertTrue(check_pid(pid), 'Must have a pid to start with')
        self.runner._kill_proc()
        self.assertFalse(check_pid(pid), 'Process must be dead')

    def test_respawn(self):
        self.runner.start()
        rospy.sleep(GRACE_DELAY)
        first_pid = self.runner.proc.pid
        self.runner._kill_proc()
        rospy.sleep(self.runner.respawn_delay + GRACE_DELAY)
        second_pid = self.runner.proc.pid
        self.assertNotEqual(first_pid, second_pid, 'Must have a different pid after respawn')
        self.assertTrue(check_pid(second_pid), 'Must be alive after respawn')

    def test_not_respawn(self):
        """
        Test the respawn flag.
        respawn is True by default, this test cases aims at not
        respawning the managed process.

        """
        self.runner = ProcRunner(TEST_CMD, respawn=False)
        self.runner.start()
        rospy.sleep(GRACE_DELAY)
        pid = self.runner.proc.pid
        self.runner._kill_proc()
        rospy.sleep(self.runner.respawn_delay + GRACE_DELAY)
        # we want, after the kill, there be no process
        self.assertFalse(check_pid(pid), 'Must be dead, was killed.')
        # can't do more tests the subprocess instance, it's been set None
        self.assertEqual(self.runner.proc, None)

    def test_spawn_handler(self):
        """
        Spawn handlers must be run on each spawn.

        """
        mock_handler = MockSpawnHandler()
        self.runner.add_spawn_hook(mock_handler)
        self.runner.start()
        rospy.sleep(GRACE_DELAY)
        self.assertEqual(mock_handler.spawns, 1, 'Invalid number of spawn handler calls so far')
        self.runner._kill_proc()
        rospy.sleep(self.runner.respawn_delay + GRACE_DELAY)
        self.assertEqual(mock_handler.spawns, 2, 'Invalid number of spawn handler calls so far')

    def test_broken_spawn_handler(self):
        """
        Broken spawn handlers must not wreck the thread.

        """
        mock_handler = MockBrokenSpawnHandler()
        self.runner.add_spawn_hook(mock_handler)
        self.test_spawn_handler()

    def test_invalid_spawn_handler(self):
        """
        Invalid spawn handlers must raise a TypeError when added.

        """
        invalid_hook = MockInvalidSpawnHandler()
        with self.assertRaises(TypeError):
            self.runner.add_spawn_hook(invalid_hook)


class TestProcRunnerCleanup(unittest.TestCase):
    def test_cleanup(self):
        runner = ProcRunner(TEST_CMD)
        runner.start()
        rospy.sleep(GRACE_DELAY)

        runner_ref = weakref.ref(runner)
        proc_ref = weakref.ref(runner.proc)

        runner.shutdown()
        rospy.sleep(GRACE_DELAY)
        gc.collect()
        self.assertIsNone(proc_ref(), 'proc must be freed on shutdown')

        runner = None
        gc.collect()
        self.assertIsNone(runner_ref(), 'runner must be freed post-delete')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestProcRunner)
    rostest.rosrun(PKG, NAME, TestProcRunnerCleanup)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
