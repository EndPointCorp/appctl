/**
 *
 * @fileoverview Implements joystick-based navigation on tactile maps.
 *
 */

/**
 * @constructor
 * @param {number} lx Linear X
 * @param {number} ly Linear Y
 * @param {number} lz Linear Z
 * @param {number} ax Angular X
 * @param {number} ay Angular Y
 * @param {number} az Angular Z
 */
Twist = function(lx, ly, lz, ax, ay, az) {
  this.linear = {};
  this.linear.x = lx || 0.0;
  this.linear.y = ly || 0.0;
  this.linear.z = lz || 0.0;
  this.angular = {};
  this.angular.x = ax || 0.0;
  this.angular.y = ay || 0.0;
  this.angular.z = az || 0.0;
};

/**
 * @param {CameraBuffer} camerBuffer The camera buffer for the scene.
 * @constructor
 */
JoystickNavigator = function(cameraBuffer) {
  /**
   * The current joystick positions, scaled for use by the navigator.
   * @private
   */
  this.joyScaled_ = new Twist(0, 0, 0, 0, 0, 0);
  this.lastCameraPose = new Pose(0, 0, 0, 0, 0, 0);
  this.lastCommandedPose = new Pose(0, 0, 0, 0, 0, 0);
  this.underJoyControl = false;
  this.cameraBuffer = cameraBuffer;
};

/**
 * Records and responds to the camera being moved, either by joystick
 * or by other inputs.
 * @param {Pose} pose
 */
JoystickNavigator.prototype.processCameraUpdate = function(pose) {
  this.lastCameraPose = pose;
  this.cameraBuffer.processCameraUpdate(pose);
};

/**
 * Updates the navigation based on the normJoy joystick position.
 * @param {Twist} normJoy A normalized [-1.0, 1.0] joystick twist.
 * @param {number} maxTilt The maximum tilt angle, in degrees.
 * @param {number} minAlt The minumum altitude, in meters.
 */
JoystickNavigator.prototype.processJoy = function(normJoy, maxTilt, minAlt) {
  /**
   * Scales the input quadratically, preserving sign.
   * @param {number} v A value to scale quadratically.
   * @return {number} The scaled value.
   */
  var quadratic_ = function(v) {
    return Math.abs(v) * v;
  };

  /**
   * A container for joystick scaling settings.
   * @constructor
   */
  var ScaleSettings = function() {
    // "Linear" sensitivity is as opposed to quadratic, not to be confused with
    // the linear/angular components of the Twist.
    this.linear_sensitivity =
        new Twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    this.quadratic_sensitivity =
        new Twist(0.000001, 0.000001, 0.2, 5.0, 0.0, 4.0);
    // The smallest input from a post-normalization SpaceNav, is ~0.00285.
    this.gutter =
        new Twist(0.001, 0.001, 0.001, 0.001, 0.001, 0.001);
  };

  /**
   * @param {Twist} twist The value to determine if it is within the gutter.
   * @return {boolean} True if it is within the gutter.
   */
  ScaleSettings.prototype.withinGutter = function(twist) {
    return Math.abs(twist.linear.x) < this.gutter.linear.x &&
        Math.abs(twist.linear.y) < this.gutter.linear.y &&
        Math.abs(twist.linear.z) < this.gutter.linear.z &&
        Math.abs(twist.angular.x) < this.gutter.angular.x &&
        Math.abs(twist.angular.y) < this.gutter.angular.y &&
        Math.abs(twist.angular.z) < this.gutter.angular.z;
  };

  /**
   * @param {Twist} norm The normalized input.
   * @param {Twist} result The scaled output.
   */
  ScaleSettings.prototype.scale = function(norm, result) {
    result.linear.x = norm.linear.x * this.linear_sensitivity.linear.x +
        quadratic_(norm.linear.x) * this.quadratic_sensitivity.linear.x;
    result.linear.y = norm.linear.y * this.linear_sensitivity.linear.y +
        quadratic_(norm.linear.y) * this.quadratic_sensitivity.linear.y;
    result.linear.z = norm.linear.z * this.linear_sensitivity.linear.z +
        quadratic_(norm.linear.z) * this.quadratic_sensitivity.linear.z;
    result.angular.x = norm.angular.x * this.linear_sensitivity.angular.x +
        quadratic_(norm.angular.x) * this.quadratic_sensitivity.angular.x;
    result.angular.y = norm.angular.y * this.linear_sensitivity.angular.y +
        quadratic_(norm.angular.y) * this.quadratic_sensitivity.angular.y;
    result.angular.z = norm.angular.z * this.linear_sensitivity.angular.z +
        quadratic_(norm.angular.z) * this.quadratic_sensitivity.angular.z;
  };

  var settings = new ScaleSettings();

  // If the entire normJoy is at steady state then we exit early.
  if (settings.withinGutter(normJoy)) {
    // This breadcrumb tells us to base future navigation on the last received
    // camera pose, not our commanded pose.
    this.underJoyControl = false;
    return;
  }
  // Otherwise, we don't gutter any inputs. If we're moving the camera at all,
  // we'll move it based on all inputs, no matter how tiny.

  // If we've been under control of the joystick since the last call, we start
  // from the last pose the joystick commanded (preventing stutter in case
  // the camera response gets delayed). However, if this is the first time
  // we're moving since the joystick was in the gutter, we'll start from
  // the last reported camera position.
  var startingPose = this.lastCommandedPose;
  if (!this.underJoyControl) {
    startingPose = this.lastCameraPose;
    this.underJoyControl = true;
  }

  settings.scale(normJoy, this.joyScaled_);

  var endingPose = new Pose(startingPose.lat, startingPose.lon,
      startingPose.alt, startingPose.heading, startingPose.tilt,
      startingPose.roll);

  // Generic clamp function, returns v clamped within [min, max].
  var clamp = function(v, min, max) {
    return Math.max(Math.min(v, max), min);
  };

  // Altitude:  Adjusts camera altitude per joystick linear Z axis.
  // Altitude is computed first, since it affects scaling of other inputs.
  // (80, 30000000)
  var clampAlt = function(v) {
    return clamp(v, minAlt, 30000000);
  };
  endingPose.alt =
      clampAlt(startingPose.alt +
               (startingPose.alt * this.joyScaled_.linear.z));

  // Heading:  Adjusts rotation per joystick angular Z axis.
  // This is computed early because it affects the direction of translations.
  // [0, 360), with wrapping.
  var clampHeading = function(v) {
    return v % 360;
  };
  endingPose.heading =
      clampHeading(startingPose.heading - this.joyScaled_.angular.z);

  // Tilt:  Adjusts tilt per joystick angular X axis.
  // [0, [90|180]] 90 for globe, 180 for photosphere
  // TODO(crbarker): clamp the tilt intelligently so we never omit earth.
  var clampTilt = function(v) {
    return clamp(v, 0, maxTilt);
  };
  endingPose.tilt =
      clampTilt(startingPose.tilt + this.joyScaled_.angular.x);

  // Translate:  Translates the camera per joystick linear X/Y axes.
  // Translation speed is dependent on altitude and direction is dependent
  // on heading.
  // (-90, 90)
  // TODO(crbarker): clamp the lat so we avoid gimbal lock at the poles.
  var clampLat = function(v) {
    var poleLat = 90 - 0.0001;
    return clamp(v, -1.0 * poleLat, poleLat);
  };
  // [-180, 180), with wrapping as we pass the meridian.
  var clampLon = function(v) {
    return ((v + 180) % 360) - 180;
  };

  var headingRadians = endingPose.heading * Math.PI / 180;
  var cosHeading = Math.cos(headingRadians);
  var sinHeading = Math.sin(headingRadians);
  var deltaLat =
      (cosHeading * this.joyScaled_.linear.y) -
      (sinHeading * this.joyScaled_.linear.x);
  var deltaLon =
      (cosHeading * this.joyScaled_.linear.x) +
      (sinHeading * this.joyScaled_.linear.y);

  // The further we are from the ground, the more we should translate x and y.
  var altScale = endingPose.alt;
  endingPose.lat = clampLat(startingPose.lat + (deltaLat * altScale));
  endingPose.lon = clampLon(startingPose.lon + (deltaLon * altScale));

  this.lastCommandedPose = endingPose;
  this.cameraBuffer.requestWarpToCameraPose(endingPose);
};

/**
 * @return {Twist} The current state of the joystick twist.
 */
JoystickNavigator.prototype.getState = function() {
    return this.joyScaled_;
}
