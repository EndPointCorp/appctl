#!/usr/bin/env python

"""
This ROS node aggregates sensor values into a boolean occupancy state.  It is
important that the occupancy state be updated quickly and not have any grace
period before the falling edge; this kind of behavior should be defined by
listening nodes.

There are two types of tracking we are interested in.

Occupancy tracking is whether or not somebody is standing in the space or
interacting with peripherals.  This is used for traffic counting, LED
activation, etc.

Interaction tracking is only triggered by interaction with peripherals, but
sustained by occupancy.  This is used for kicking the system out of ambient
mode, tracking user sessions, etc.
"""

import rospy
from std_msgs.msg import Bool, Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from evdev_teleport.msg import EvdevEvents
from leap_motion.msg import Frame

DEFAULT_CHECK_INTERVAL = 0.1  # seconds
DEFAULT_DISTANCE_THRESHOLD = 1.0  # meters
PUB_QUEUE_SIZE = 3


class EventTracker():
    """
    Keeps track of sensor readings and provides a method for polling occupancy
    state.
    """
    def __init__(self, distance_threshold, proximity_trigger):
        self.distance_threshold = distance_threshold
        self.proximity_trigger = proximity_trigger
        self.is_active = False
        self.inactive_duration = rospy.Duration(0)
        self.last_occupied_time = rospy.Time.now()
        self.last_distance = None
        self.last_spacenav_twist = None
        self.last_leap_frame = None
        self.last_evdev_event_time = None

    def update(self, timer_event):
        """Update the tracker state according to rospy.Timer data."""
        if timer_event.last_real is not None:
            timer_delta_ns = timer_event.current_real - timer_event.last_real
            timer_delta = timer_delta_ns.to_sec()
        else:
            timer_delta = rospy.Duration(0)

        now = EventTracker.get_current_time()

        o_proximity = self.check_proximity_distance()

        o_spacenav = self.check_spacenav_twist()

        o_leap = self.check_leap_frame()

        o_evdev = EventTracker.check_last_event_time(
            self.last_evdev_event_time,
            timer_delta
        )

        self.is_active = (
            o_proximity or
            o_spacenav or
            o_leap or
            o_evdev
        )

        if self.is_active:
            self.last_occupied_time = rospy.get_rostime()
            self.inactive_duration = rospy.Duration(0)
        else:
            self.inactive_duration = rospy.Duration.from_sec(
                now - self.last_occupied_time.to_sec()
            )

    def handle_proximity_distance_msg(self, msg):
        """Handles an incoming proximity distance message."""
        self.last_distance = msg.range

    def handle_spacenav_twist_msg(self, msg):
        """Handles an incoming SpaceNav twist message."""
        self.last_spacenav_twist = msg

    def handle_leap_frame_msg(self, msg):
        """Handles an incoming LEAP frame message."""
        self.last_leap_frame = msg

    def handle_evdev_event_msg(self, msg):
        """Handles an incoming evdev event message."""
        self.last_evdev_event_time = EventTracker.get_current_time()

    @staticmethod
    def get_current_time():
        """Return absolute time expressed in seconds."""
        return rospy.get_time()

    def check_proximity_distance(self):
        """True if distance measurement indicates occupancy."""
        distance = self.last_distance
        distance_threshold = self.distance_threshold
        proximity_trigger = self.proximity_trigger
        active = self.is_active
        # If no proximity data is available, assume no sensor is hooked up.
        if distance is None:
            return False
        # If proximity trigger is not set, don't trigger new activity.
        if not proximity_trigger and not active:
            return False
        # TODO(mv): ignore last distance if no messages received recently
        return distance < distance_threshold

    def check_spacenav_twist(self):
        """True if the SpaceNav frame indicates occupancy."""
        twist = self.last_spacenav_twist
        # TODO(mv): ignore last twist if no messages received recently
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

    def check_leap_frame(self):
        """True if the LEAP frame indicates occupancy."""
        frame = self.last_leap_frame
        proximity_trigger = self.proximity_trigger
        active = self.is_active
        # TODO(mv): ignore last frame if no messages received recently
        return frame is not None and len(frame.hands) > 0

    @staticmethod
    def check_last_event_time(last_event_time, timer_delta):
        """True if the given time happened in the previous check interval."""
        now = EventTracker.get_current_time()
        return (
            last_event_time is not None and
            now - last_event_time < timer_delta
        )


def main():
    """Creates trackers, subscriptions, publishers, and update timers."""
    rospy.init_node('portal_occupancy')

    check_interval = rospy.get_param(
        '~check_interval',
        DEFAULT_CHECK_INTERVAL
    )
    distance_threshold = rospy.get_param(
        '~distance_threshold',
        DEFAULT_DISTANCE_THRESHOLD
    )

    trackers = {
        'occupancy': EventTracker(
            distance_threshold,
            proximity_trigger=True
        ),
        'interaction': EventTracker(
            distance_threshold,
            proximity_trigger=False
        )
    }

    def make_publishers(k):
        return {
            'is_active': rospy.Publisher(
                '/portal_occupancy/{}/is_active'.format(k),
                Bool,
                queue_size=PUB_QUEUE_SIZE
            ),
            'inactive_duration': rospy.Publisher(
                '/portal_occupancy/{}/inactive_duration'.format(k),
                Duration,
                queue_size=PUB_QUEUE_SIZE
            )
        }

    publishers = {k: make_publishers(k) for k in trackers.iterkeys()}

    def subscribe_trackers(topic, data_class, method):
        def handle_msg(msg):
            map(lambda t: getattr(t, method)(msg), trackers.itervalues())
        rospy.Subscriber(topic, data_class, handle_msg)

    subscribe_trackers(
        '/proximity/distance',
        Range,
        'handle_proximity_distance_msg'
    )
    subscribe_trackers(
        '/spacenav/twist',
        Twist,
        'handle_spacenav_twist_msg'
    )
    subscribe_trackers(
        '/leap_motion/frame',
        Frame,
        'handle_leap_frame_msg'
    )
    subscribe_trackers(
        '/evdev_teleport/event',
        EvdevEvents,
        'handle_evdev_event_msg'
    )

    def publish_topic(tracker, topic, pub):
        val = getattr(tracker, topic)
        # data_class is a member of rospy.topics.Topic
        msg = pub.data_class(val)
        pub.publish(msg)

    def update(timer_event):
        for k, tracker in trackers.iteritems():
            tracker.update(timer_event)
            for topic, pub in publishers[k].iteritems():
                publish_topic(tracker, topic, pub)

    update_timer = rospy.Timer(
        rospy.Duration(check_interval),
        update,
        oneshot=False
    )

    rospy.spin()

    update_timer.shutdown()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
