// crbarker@google.com
//
// Implements JoystickNavigator.

#include "joystick_navigator.h"

#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <algorithm>
#include <cmath>

#include "camera_buffer.h"
#include "slinky_nav/SlinkyPose.h"

static const double kPi = 3.141592653589793;
static const double kPoleLat = 90.0 - 0.000001;
static const double kEarthRadius = 6371000;
static const timespec kExpectedInterval = {0, 33333333};  // 30 Hz

JoystickNavigator::JoystickNavigator()
    : under_joy_control_(false),
      camera_buffer_(NULL) {
  // The sensitivities and gutter are hard-coded here.
  // In this case, linear_sensitivity_ refers to linear scaling
  // (vs. quadratic scaling) not linear motion (vs. angular).
  linear_sensitivity_.linear.x = 0.0;  // longitude
  linear_sensitivity_.linear.y = 0.0;  // latitude
  linear_sensitivity_.linear.z = 0.0;  // altitude
  linear_sensitivity_.angular.x = 0.0;  // pitch
  linear_sensitivity_.angular.y = 0.0;  // roll
  linear_sensitivity_.angular.z = 0.0;  // heading

  quadratic_sensitivity_.linear.x = 0.000001;  // longitude
  quadratic_sensitivity_.linear.y = 0.000001;  // latitude
  quadratic_sensitivity_.linear.z = 0.2;  // altitude
  quadratic_sensitivity_.angular.x = 5.0;  // pitch
  quadratic_sensitivity_.angular.y = 0.0;  // roll
  quadratic_sensitivity_.angular.z = 4.0;  // heading

  // If all joystick inputs are all less than the gutter, the input
  // is ignored, and it's assumed that we're under touch/mouse control.
  gutter_.linear.x = 0.001;
  gutter_.linear.y = 0.001;
  gutter_.linear.z = 0.001;
  gutter_.angular.x = 0.001;
  gutter_.angular.y = 0.001;
  gutter_.angular.z = 0.001;
}

void JoystickNavigator::Init(CameraBuffer* camera_buffer) {
  camera_buffer_ = camera_buffer;
  clock_gettime(CLOCK_REALTIME, &last_joy_time_);
}

void JoystickNavigator::ProcessCameraMoved(
    const slinky_nav::SlinkyPose& slinky_pose) {
  last_camera_pose_ = slinky_pose.current_pose;
  pose_minimums_ = slinky_pose.pose_minimums;
  pose_maximums_ = slinky_pose.pose_maximums;
}

void JoystickNavigator::ProcessJoy(const geometry_msgs::Twist& normalized_joy) {
  // The joystick sensitivities are tuned for a nominal input frequency.
  // However, inputs can come faster/slower than that, so we apply further
  // scaling.
  timespec current_time;
  clock_gettime(CLOCK_REALTIME, &current_time);
  timespec diff_time = DiffTimespec(last_joy_time_, current_time);
  last_joy_time_ = current_time;
  double interval_scale = DivTimespec(diff_time, kExpectedInterval);

  // If the entire normalized_joy is at steady state then we exit early.
  if (IsWithinGutter(normalized_joy)) {
    // This breadcrumb tells us to base future navigation on the last received
    // camera pose, not our requested pose.
    under_joy_control_ = false;
    return;
  }
  // Otherwise, we don't gutter any inputs. If we're moving the camera at all,
  // we'll move it based on all inputs, no matter how tiny.

  // If we've been under control of the joystick since the last call, we start
  // from the last pose the joystick requested (preventing stutter in case
  // the camera response gets delayed). However, if this is the first time
  // we're moving since the joystick was in the gutter, we'll start from
  // the last reported camera position.
  geometry_msgs::Pose& starting_pose = last_requested_pose_;
  if (!under_joy_control_) {
    starting_pose = last_camera_pose_;
    under_joy_control_ = true;
  }

  geometry_msgs::Pose ending_pose = starting_pose;
  geometry_msgs::Twist scaled_joy;
  Scale(normalized_joy, interval_scale, &scaled_joy);

  // Altitude:  Adjusts camera altitude per joystick linear Z axis.
  // Altitude is computed first, since it affects scaling of other inputs.
  ending_pose.position.z =
      Clamp(starting_pose.position.z +
          (starting_pose.position.z * scaled_joy.linear.z),
          pose_minimums_.position.z, pose_maximums_.position.z);

  // Heading:  Adjusts compass heading per joystick angular Z axis.
  // This is computed early because it affects the direction of translations.
  // [0, 360), with wrapping.
  ending_pose.orientation.z =
      fmod(starting_pose.orientation.z + 360 - scaled_joy.angular.z, 360);
  ending_pose.orientation.z =
      Clamp(ending_pose.orientation.z,
            pose_minimums_.orientation.z,
            pose_maximums_.orientation.z);

  // Tilt:  Adjusts tilt per joystick angular X axis.
  // TODO(crbarker): clamp the tilt intelligently so we never omit earth.
  ending_pose.orientation.x =
      Clamp(starting_pose.orientation.x + scaled_joy.angular.x,
            pose_minimums_.orientation.x,
            pose_maximums_.orientation.x);

  // Translate:  Translates the camera per joystick linear X/Y axes.
  // Translation speed is dependent on altitude and direction is dependent
  // on heading.
  // Latitude (-90, 90)
  // Longitude [-180, 180), with wrapping as we pass the meridian.
  double heading_radians = ending_pose.orientation.z * kPi / 180;
  double cos_heading = cos(heading_radians);
  double sin_heading = sin(heading_radians);
  double delta_lat =
      (cos_heading * scaled_joy.linear.y) -
      (sin_heading * scaled_joy.linear.x);
  double delta_lon =
      (cos_heading * scaled_joy.linear.x) +
      (sin_heading * scaled_joy.linear.y);

  // The further we are from the ground, the more we should translate x and y.
  double alt_scale = ending_pose.position.z;
  ending_pose.position.y =
      Clamp(starting_pose.position.y + (delta_lat * alt_scale),
            kPoleLat * -1.0,
            kPoleLat);
  ending_pose.position.y =
      Clamp(ending_pose.position.y,
            pose_minimums_.position.y,
            pose_maximums_.position.y);

  ending_pose.position.x =
      fmod((starting_pose.position.x + (delta_lon * alt_scale)) + 180, 360) -
      180;
  ending_pose.position.x =
      Clamp(ending_pose.position.x,
            pose_minimums_.position.x,
            pose_maximums_.position.x);

  last_requested_pose_ = ending_pose;
  camera_buffer_->RequestPose(ending_pose);
}

double JoystickNavigator::Quadratic(double v) {
  return std::abs(v) * v;
}

double JoystickNavigator::Clamp(double v, double min, double max) {
  return std::max(std::min(v, max), min);
}

timespec JoystickNavigator::DiffTimespec(timespec start, timespec end) {
  timespec diff;
  if ((end.tv_nsec - start.tv_nsec) < 0) {
    diff.tv_sec = end.tv_sec - start.tv_sec - 1;
    diff.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  } else {
    diff.tv_sec = end.tv_sec - start.tv_sec;
    diff.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return diff;
}

double JoystickNavigator::DivTimespec(timespec num, timespec denom) {
  double lnum = num.tv_sec * 1000000000 + num.tv_nsec;
  double ldenom = denom.tv_sec * 1000000000 + denom.tv_nsec;
  return lnum / ldenom;
}

// Compares the 'normal' and returns true if anything is outside the gutter.
bool JoystickNavigator::IsWithinGutter(const geometry_msgs::Twist& normal) {
  return (std::abs(normal.linear.x) < gutter_.linear.x) &&
      (std::abs(normal.linear.y) < gutter_.linear.y) &&
      (std::abs(normal.linear.z) < gutter_.linear.z) &&
      (std::abs(normal.angular.x) < gutter_.angular.x) &&
      (std::abs(normal.angular.y) < gutter_.angular.y) &&
      (std::abs(normal.angular.z) < gutter_.angular.z);
}

// Multiplies the 'normal' by linear_sensitivity_, and multiplies the quadratic
// scaled 'normal' by quadratic_sensitivity_.  Then scales the whole thing by
// 'interval_scale.'
void JoystickNavigator::Scale(
    const geometry_msgs::Twist& normal,
    double interval_scale,
    geometry_msgs::Twist* result) {
  result->linear.x = (normal.linear.x * linear_sensitivity_.linear.x +
      Quadratic(normal.linear.x) * quadratic_sensitivity_.linear.x) *
      interval_scale;
  result->linear.y = (normal.linear.y * linear_sensitivity_.linear.y +
      Quadratic(normal.linear.y) * quadratic_sensitivity_.linear.y) *
      interval_scale;
  result->linear.z = (normal.linear.z * linear_sensitivity_.linear.z +
      Quadratic(normal.linear.z) * quadratic_sensitivity_.linear.z) *
      interval_scale;
  result->angular.x = (normal.angular.x * linear_sensitivity_.angular.x +
      Quadratic(normal.angular.x) * quadratic_sensitivity_.angular.x) *
      interval_scale;
  result->angular.y = (normal.angular.y * linear_sensitivity_.angular.y +
      Quadratic(normal.angular.y) * quadratic_sensitivity_.angular.y) *
      interval_scale;
  result->angular.z = (normal.angular.z * linear_sensitivity_.angular.z +
      Quadratic(normal.angular.z) * quadratic_sensitivity_.angular.z) *
      interval_scale;
}