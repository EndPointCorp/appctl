#!/usr/bin/env python
__author__ = 'flier'
import rospy
import leap_interface
#from leap_motion.msg import Finger
from leap_motion.msg import Frame

# Obviously, this method publishes the data defined in leapros.msg to /leapmotion/data
def sender():
    li = leap_interface.Runner()
    li.setDaemon(True)
    li.start()
    pub_ros   = rospy.Publisher('leap_motion/frame',Frame)
    rospy.init_node('leap_pub')

    while not rospy.is_shutdown():
        hand_direction_   = li.get_hand_direction()
        hand_normal_      = li.get_hand_normal()
        hand_palm_pos_    = li.get_hand_palmpos()
        hand_pitch_       = li.get_hand_pitch()
        hand_roll_        = li.get_hand_roll()
        hand_yaw_         = li.get_hand_yaw()

        frame = li.get_frame()
        msg = Frame()

        if frame:
            msg.id = frame.id
            msg.timestamp = frame.timestamp
            msg.hands = frame.hands
            #msg.pointables = frame.pointables
            msg.interaction_box = frame.interaction_box
            msg.current_frames_per_second = frame.current_frames_per_second
            msg.is_valid = frame.is_valid

        '''
        # Add Fingers to msg.
        hand = li.get_hand()
        def create_finger(f):
            finger = Finger()
            finger.tip_position = f.tip_position
            finger.tip_velocity = f.tip_velocity
            finger.direction = f.direction
            finger.width = f.width
            finger.length = f.length
            finger.is_finger = f.is_finger
            finger.is_tool = f.is_tool
            finger.is_valid = f.is_valid
            finger.touch_zone = f.touch_zone
            finger.touch_distance = f.touch_distance
            finger.stabilized_tip_position = f.stabilized_tip_position
            finger.time_visible = f.time_visible

            return finger

        if hand is not None:
            msg.fingers = [create_finger(f) for f in hand.fingers if f is not None]
        '''

        # We don't publish native data types, see ROS best practices
        # pub.publish(hand_direction=hand_direction_,hand_normal = hand_normal_, hand_palm_pos = hand_palm_pos_, hand_pitch = hand_pitch_, hand_roll = hand_roll_, hand_yaw = hand_yaw_)
        #import pdb; pdb.set_trace()
        pub_ros.publish(msg)
        # Save some CPU time, circa 100Hz publishing.
        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
