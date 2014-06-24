// crbarker@google.com
//
// The CameraBuffer inserts a lock-step buffer between the navigator and the
// camera.  It remembers requested poses and throttles them, pushing them
// to the camera only when nothing else is pending, or the camera has just
// refreshed.

#ifndef EXPERIMENTAL_ACME_SRC_SLINKY_CATKIN_SRC_SLINKY_NAV_SRC_CAMERA_BUFFER_H_
#define EXPERIMENTAL_ACME_SRC_SLINKY_CATKIN_SRC_SLINKY_NAV_SRC_CAMERA_BUFFER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class CameraBuffer {
 public:
  CameraBuffer()
      : last_requested_pose_valid_(false),
        num_requests_outstanding_(0) {}

  void Init(ros::NodeHandle n, const std::string& topic);

  // Informs the buffer that the camera has moved, so it can dispatch a
  // new pose if one is pending.
  void ProcessCameraMoved(void);

  // If the camera is not busy, sends the pose directly to the camera.
  // Otherwise, remembers the pose to send the next time the camera is free.
  void RequestPose(const geometry_msgs::Pose& pose);

 private:
  // Sends the last_requested_pose_ to the camera.
  void SendLastRequestedPose(void);

  ros::NodeHandle n_;
  std::string topic_;
  ros::Publisher pose_pub_;
  geometry_msgs::Pose last_requested_pose_;
  bool last_requested_pose_valid_;
  int num_requests_outstanding_;
};

#endif  // EXPERIMENTAL_ACME_SRC_SLINKY_CATKIN_SRC_SLINKY_NAV_SRC_CAMERA_BUFFER_H_
