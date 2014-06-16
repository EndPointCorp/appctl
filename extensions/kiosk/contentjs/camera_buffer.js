/**
 *
 * @fileoverview Implements a buffer to prevent overdriving the camera view.
 *
 * This shim remembers the latest requestCameraPose() and only pushes
 * it to the tactile API if we've gotten a recent position callback from
 * the camera calling processCameraUpdate(), so we know it's moving.
 * If the camera starts to fall behind, it stops publishing updates,
 * so this shim pauses the requests.
 */

/**
 * @constructor
 * @param {number} lat Latitude in degrees N, [-90, 90]
 * @param {number} lon Longitude in degrees E, [-180, 180]
 * @param {number} alt Altitude (meters)
 * @param {number} heading Heading in degrees, [0-360)
 * @param {number} tilt Tilt in degrees, [0 == down, 90 == horizon)
 * @param {number} roll Roll (unused)
 */
Pose = function(lat, lon, alt, heading, tilt, roll) {
  this.lat = lat || 0.0;
  this.lon = lon || 0.0;
  this.alt = alt || 0.0;
  this.heading = heading || 0.0;
  this.tilt = tilt || 0.0;
  this.roll = roll || 0.0;
};

/**
 * Fully zoomed out space pose.
 * @const
 */
Pose.SPACE_POSE = {
  alt: 36885992.037503954, // High enough to see the stars!
  heading: 0,
  lat: 32.62570525526346,
  lon: -49.106239982910026,
  roll: 0,
  tilt: 0
};

/**
 * @param {acme.Kiosk} kiosk
 * @constructor
 */
CameraBuffer = function(kiosk) {
  this.lastRequested = {
    /** The camera pose to move to. */
    pose: new Pose(0, 0, 0, 0, 0, 0),
    /** True: animate, false: warp. */
    shouldAnimate: false,
    /** True if this request hasn't been sent. */
    valid: false
  };
  /** The camera is ready for the next pose. */
  this.cameraIsReady = false;
  this.kiosk_ = kiosk;
};

/**
 * Calls moveCamera with the requested state.
 *
 * @private
 */
CameraBuffer.prototype.pushRequestedPose_ = function() {
  // This function is now renterent so the flags need to be set before calling
  // moveCamera which could cause us to get called again when tactile tells
  // us that it has moved.
  this.cameraIsReady = false;
  this.lastRequested.valid = false;
  this.kiosk_.moveCamera(this.lastRequested.pose,
      this.lastRequested.shouldAnimate);
};

/**
 * Records and responds to the camera being moved. Called by tactile once the
 * camera has actually moved.
 */
CameraBuffer.prototype.processCameraUpdate = function() {
  this.cameraIsReady = true;
  if (this.lastRequested.valid) {
    this.pushRequestedPose_();
  }
};

/**
 * Animate to the supplied camera pose.
 *
 * @param {Pose} pose The Camera Pose.
 */
CameraBuffer.prototype.requestAnimateToCameraPose = function(pose) {
  this.requestCameraPose_(pose, true);
};

/**
 * Warp to the supplied camera pose.
 *
 * @param {Pose} pose The Camera Pose.
 */
CameraBuffer.prototype.requestWarpToCameraPose = function(pose) {
  this.requestCameraPose_(pose, false);
};

/**
 * Records and responds to the camera being moved. Called by ros in order
 * to request a camera move. The request is not acted upon until the camera
 * confirms it completed it's last move by calling processCameraUpdate.
 *
 * @param {Pose} pose The Camera Pose.
 * @param {boolean} shouldAnimate True/false to choose animate/warp to pose.
 * @private
 */
CameraBuffer.prototype.requestCameraPose_ = function(pose, shouldAnimate) {
  this.lastRequested.pose = pose;
  this.lastRequested.shouldAnimate = shouldAnimate;
  this.lastRequested.valid = true;

  // TODO(daden): Remove this when b/15387310 fix goes live.
  if (true || this.cameraIsReady) {
    this.pushRequestedPose_();
  }
};
