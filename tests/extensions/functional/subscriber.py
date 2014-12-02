"""
Helper rostopic subscriber script.
Useful to investigate ROS traffic, analogous to rostopic echo /portal_kiosk/current_pose
Need to adjust topic name and message format class:
    rospy.Subscriber("/portal_kiosk/current_pose", PortalPose, callback)

Running:
    source catkin/devel/setup.bash (makes message format class available)
    python tests/extensions/functional/subscriber.py

"""

import rospy
from std_msgs.msg import String  # also rospy thing

from portal_nav.msg import PortalPose


def callback(msg):
    #rospy.loginfo(rospy.get_caller_id() + " received msg: '%s'" % msg)
    print msg.current_pose.position.x, msg.current_pose.position.y, msg.current_pose.position.y


def subscriber():
    rospy.init_node("subscriber", anonymous=True)
    rospy.Subscriber("/portal_kiosk/current_pose", PortalPose, callback)
    # spin() simply keeps python from exiting until this node is stopped
    print "listening ..."
    rospy.spin()


if __name__ == "__main__":
    subscriber()