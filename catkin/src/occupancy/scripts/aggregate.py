#!/usr/bin/env python

"""
This ROS node aggregates sensor values into a boolean occupancy state.  It is
important that the occupancy state be updated quickly and not have any grace
period before the falling edge; this kind of behavior should be defined by
listening nodes.
"""

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from evdev_teleport.msg import EvdevEvents
from leap_motion.msg import Frame

DEFAULT_CHECK_INTERVAL = 0.1  # seconds
DEFAULT_DISTANCE_THRESHOLD = 1.0  # meters


class OccupancyAggregator():
    """
    Keeps track of sensor readings and provides a method for polling occupancy
    state.
    """
    def __init__(self, distance_threshold):
        self.distance_threshold = distance_threshold
        self.occupied = False
        self.last_distance = None
        self.last_spacenav_twist = None
        self.last_leap_frame = None
        self.last_evdev_event_time = None

    def is_occupied(self, timer_event):
        if timer_event.last_real is not None:
            timer_delta_ns = timer_event.current_real - timer_event.last_real
            timer_delta = timer_delta_ns.to_sec()
        else:
            timer_delta = rospy.Duration(0)

        # TODO(mv): ignore last distance if no messages received recently
        o_proximity = OccupancyAggregator.check_proximity_distance(
            self.last_distance,
            self.distance_threshold
        )

        # TODO(mv): ignore last twist if no messages received recently
        o_spacenav = OccupancyAggregator.check_spacenav_twist(
            self.last_spacenav_twist
        )

        # TODO(mv): ignore last frame if no messages received recently
        o_leap = OccupancyAggregator.check_leap_frame(self.last_leap_frame)

        o_evdev = OccupancyAggregator.check_last_event_time(
            self.last_evdev_event_time,
            timer_delta
        )

        self.occupied = (
            o_proximity or
            o_spacenav or
            (self.occupied and o_leap) or
            o_evdev
        )
        return self.occupied

    def handle_proximity_distance_msg(self, msg):
        self.last_distance = msg.range

    def handle_spacenav_twist_msg(self, msg):
        self.last_spacenav_twist = msg

    def handle_leap_frame_msg(self, msg):
        self.last_leap_frame = msg

    def handle_evdev_event_msg(self, msg):
        self.last_evdev_event_time = OccupancyAggregator.get_current_time()

    @staticmethod
    def get_current_time():
        """Return absolute time expressed in seconds."""
        return rospy.get_time()

    @staticmethod
    def check_proximity_distance(distance, distance_threshold):
        """True if distance measurement indicates occupancy."""
        return distance is not None and distance < distance_threshold

    @staticmethod
    def check_spacenav_twist(twist):
        """True if the SpaceNav frame indicates occupancy."""
        return (
            twist is not None and (
                twist.linear.x != 0.0 or
                twist.linear.y != 0.0 or
                twist.linear.z != 0.0 or
                twist.angular.x != 0.0 or
                twist.angular.y != 0.0 or
                twist.angular.z != 0.0
            )
        )

    @staticmethod
    def check_leap_frame(frame):
        """True if the LEAP frame indicates occupancy."""
        return frame is not None and len(frame.hands) > 0

    @staticmethod
    def check_last_event_time(last_event_time, timer_delta):
        """True if the given time happened in the previous check interval."""
        now = OccupancyAggregator.get_current_time()
        return (
            last_event_time is not None and
            now - last_event_time < timer_delta
        )


def main():
    rospy.init_node('occupancy_aggregator')

    check_interval = rospy.get_param(
        '~check_interval',
        DEFAULT_CHECK_INTERVAL
    )
    distance_threshold = rospy.get_param(
        '~distance_threshold',
        DEFAULT_DISTANCE_THRESHOLD
    )

    aggregator = OccupancyAggregator(distance_threshold)

    occupancy_pub = rospy.Publisher(
        '/occupancy/state',
        Bool,
        queue_size=5
    )

    rospy.Subscriber(
        '/proximity/distance',
        Range,
        aggregator.handle_proximity_distance_msg
    )
    rospy.Subscriber(
        '/spacenav/twist',
        Twist,
        aggregator.handle_spacenav_twist_msg
    )
    rospy.Subscriber(
        '/leap_motion/frame',
        Frame,
        aggregator.handle_leap_frame_msg
    )
    rospy.Subscriber(
        '/evdev_teleport/event',
        EvdevEvents,
        aggregator.handle_evdev_event_msg
    )

    def publish_occupancy(timer_event):
        occupied = aggregator.is_occupied(timer_event)
        occupancy_msg = Bool(occupied)
        occupancy_pub.publish(occupancy_msg)

    check_timer = rospy.Timer(
        rospy.Duration(check_interval),
        publish_occupancy,
        oneshot=False
    )

    rospy.spin()

    check_timer.shutdown()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
