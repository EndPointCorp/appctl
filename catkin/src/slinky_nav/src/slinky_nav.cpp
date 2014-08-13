// crbarker@google.com
//
// This is a ROS program to handle joystick-based navigation for slinky.

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "joystick_navigator.h"
#include "slinky_nav/SlinkyPose.h"

// #define DEBUG

class SlinkyNavigatorNode {
 public:
  SlinkyNavigatorNode(void) {}

  // Starts the run loop and does not return until killed.
  void Run(void);

  // Callbacks for ROS topic subscriptions:
  void HandleSpaceNav(const geometry_msgs::Twist::ConstPtr& twist);
  void HandleKioskPose(
      const slinky_nav::SlinkyPose::ConstPtr& slinky_pose);

 private:
  ros::NodeHandle n_;
  ros::Subscriber spacenav_sub_;
  ros::Subscriber kiosk_pose_sub_;
  ros::Publisher joystick_pub_;
  ros::Publisher kiosk_pub_;
  ros::Publisher display_pub_;

  JoystickNavigator kiosk_joystick_navigator_;
};

void SlinkyNavigatorNode::Run(void) {
  kiosk_pub_ = n_.advertise<geometry_msgs::PoseStamped>(
      "/slinky_nav/kiosk_goto_pose", 1);
  display_pub_ = n_.advertise<geometry_msgs::PoseStamped>(
      "/slinky_nav/display_goto_pose", 1);

  kiosk_joystick_navigator_.Init(&kiosk_pub_, &display_pub_);

  // This subscriber takes commands from the SpaceNav.
  spacenav_sub_ = n_.subscribe("/spacenav/twist", 0,
      &SlinkyNavigatorNode::HandleSpaceNav, this);

  // This subscriber gets camera poses back from the kiosk.
  // We have a small Rx queue to make sure we don't lose any.
  kiosk_pose_sub_ = n_.subscribe("/slinky_kiosk/current_pose", 10,
      &SlinkyNavigatorNode::HandleKioskPose, this);

  // This publishes a normalized version of the SpaceNav inputs.
  joystick_pub_ = n_.advertise<geometry_msgs::Twist>("/joystick/twist", 0);

  // Does not return until killed:
  ros::spin();
}

void SlinkyNavigatorNode::HandleSpaceNav(
    const geometry_msgs::Twist::ConstPtr& twist) {
  // The SpaceNav twist values range [-350, 350], so it must be
  // normalized for the joystick code, which expects [-1.0, 1.0].
  static const double kSpaceNavScale = 350.0;

  // Our JoystickNavigator expects a right-hand-rule twist, where
  // X is right/left, Y is forward/backward, and Z is up/down.
  // The SpaceNav ROS driver swaps both linear and angular x/y.
  // It also sign-flips the y for both linear and angular.
  // We correct that here:
  geometry_msgs::Twist normalized_joy;
  normalized_joy.linear.x = twist->linear.y / kSpaceNavScale * -1.0;
  normalized_joy.linear.y = twist->linear.x / kSpaceNavScale;
  normalized_joy.linear.z = twist->linear.z / kSpaceNavScale;
  normalized_joy.angular.x = twist->angular.y / kSpaceNavScale * -1.0;
  normalized_joy.angular.y = twist->angular.x / kSpaceNavScale;
  normalized_joy.angular.z = twist->angular.z / kSpaceNavScale;

  kiosk_joystick_navigator_.ProcessJoy(normalized_joy);
  joystick_pub_.publish(normalized_joy);
}

void SlinkyNavigatorNode::HandleKioskPose(
    const slinky_nav::SlinkyPose::ConstPtr& slinky_pose) {
#ifdef DEBUG
  ROS_INFO("HandleKioskPose curr lat:%lf, lon:%lf, alt:%lf, hdg:%lf, tlt:%lf",
           slinky_pose->current_pose.position.y,
           slinky_pose->current_pose.position.x,
           slinky_pose->current_pose.position.z,
           slinky_pose->current_pose.orientation.z,
           slinky_pose->current_pose.orientation.x);
  ROS_INFO("HandleKioskPose min lat:%lf, lon:%lf, alt:%lf, hdg:%lf, tlt:%lf",
           slinky_pose->pose_minimums.position.y,
           slinky_pose->pose_minimums.position.x,
           slinky_pose->pose_minimums.position.z,
           slinky_pose->pose_minimums.orientation.z,
           slinky_pose->pose_minimums.orientation.x);
  ROS_INFO("HandleKioskPose max lat:%lf, lon:%lf, alt:%lf, hdg:%lf, tlt:%lf",
           slinky_pose->pose_maximums.position.y,
           slinky_pose->pose_maximums.position.x,
           slinky_pose->pose_maximums.position.z,
           slinky_pose->pose_maximums.orientation.z,
           slinky_pose->pose_maximums.orientation.x);
#endif
  kiosk_joystick_navigator_.ProcessCameraMoved(*slinky_pose);
}

/*
 * main()
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "slinky_nav");
  SlinkyNavigatorNode node;
  node.Run();
  return 0;
}
