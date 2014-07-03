// crbarker@google.com
//
// Implements CameraBuffer.

#include "camera_buffer.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>

// #define DEBUG

static const int kQueueDepth = 1;

void CameraBuffer::Init(ros::NodeHandle n, const std::string& topic) {
  n_ = n;
  topic_ = topic;
  pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>(topic_.c_str(),
                                                       kQueueDepth);
}

void CameraBuffer::ProcessCameraMoved(void) {
  if (num_requests_outstanding_ > 0) --num_requests_outstanding_;
  if (last_requested_pose_valid_) {
    SendLastRequestedPose();
  }
}

void CameraBuffer::RequestPose(const geometry_msgs::Pose& pose) {
  last_requested_pose_ = pose;
  last_requested_pose_valid_ = true;
  if (num_requests_outstanding_ < kQueueDepth) {
    SendLastRequestedPose();
  }
}

void CameraBuffer::SendLastRequestedPose(void) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose = last_requested_pose_;
  pose_msg.header.stamp = ros::Time::now();

#ifdef DEBUG
  ROS_INFO(
      "Send %s lat:%lf, lon:%lf, alt:%lf, hdg:%lf, tlt:%lf, q:%d",
      topic_.c_str(),
      pose_msg.pose.position.y,
      pose_msg.pose.position.x,
      pose_msg.pose.position.z,
      pose_msg.pose.orientation.z,
      pose_msg.pose.orientation.x,
      num_requests_outstanding_);
#endif

  ++num_requests_outstanding_;
  last_requested_pose_valid_ = false;
  pose_pub_.publish(pose_msg);
}
