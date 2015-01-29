/**
 * Ambient mode handler.  Runs famous places tours while the system is idle.
 * @constructor
 * @author Matt Vollrath <matt@endpoint.com>
 */
var Ambient = function() {
  /**
   * Time the system must be idle before starting ambient mode (in seconds).
   * @type {Number}
   */
  this.OCCUPANCY_TIMEOUT = 20.0;

  /**
   * Time content will run before cycling (in seconds).
   * @type {Number}
   */
  this.CONTENT_TIMEOUT = 30.0; // seconds

  /**
   * Timeout for allowing a runway content selection from ambient mode (in
   * seconds).
   * @type {Number}
   */
  this.INTERACTION_DELAY = 0.25;

  /**
   * Ambient mode is currently on.
   * @type {boolean}
   */
  this.isAmbient_ = false;

  /**
   * Tracks which famous place is next.
   * @type {Number}
   */
  this.placeIndex_ = 0;

  /**
   * Time of the last runway touch for arbitration.  This is part of preventing
   * a race between famous places selection during ambient mode and the exit
   * event.
   */
  this.lastRunwaySelect_ = Date.now();

  /**
   * Tracks the timer handle for next content activation.
   * @type {Number}
   */
  this.contentTimer_ = null;
};

/**
 * Dispatches a global event.
 * @param {Event} ev The event to be dispatched
 */
Ambient.prototype.dispatchWindowEvent_ = function(ev) {
  window.dispatchEvent(ev);
};

/**
 * Launches the next Famous Places content if the system is still idle.
 */
Ambient.prototype.runContent_ = function() {
  if (!this.isAmbient_) return;

  var contentArray = acme.fpContent[this.placeIndex_];
  this.dispatchWindowEvent_(new CustomEvent('acmeSelectFamousPlaces'));
  this.dispatchWindowEvent_(new CustomEvent(
    'acmeLaunchFamousPlacesContent', {detail: contentArray}));
  this.placeIndex_++;
  if (this.placeIndex_ >= acme.fpContent.length) {
    this.placeIndex_ = 0;
  }

  this.scheduleNextContent_();
};

/**
 * Schedules the next run of content.
 */
Ambient.prototype.scheduleNextContent_ = function() {
  this.contentTimer = setTimeout(
    this.runContent_.bind(this),
    this.CONTENT_TIMEOUT * 1000
  );
};

/**
 * Handles an incoming inactivity duration message.
 * @param {ROS::std_msgs/Duration} msg
 *       The incoming inactivity duration message from ROS.
 */
Ambient.prototype.handleInteractionMessage = function(msg) {
  this.interactionUpdate_(msg.data);
};

/**
 * Handles a runway content selection.  We care about this because if a
 * famous places selection is made during ambient mode, we don't want it to
 * race against a content exit.
 */
Ambient.prototype.handleRunwaySelect = function() {
  this.lastRunwaySelect_ = Date.now();
};

/**
 * Converts a ROS duration to seconds.
 * @param {ROS::std_msgs/Duration} duration
 * @return {Number} seconds
 */
Ambient.prototype.durationToSeconds_ = function(duration) {
  return duration.secs + duration.nsecs * 1.0e-9;
};

/**
 * Updates system idle tracking with a positive or negative sample.
 * @param {object} inactive_duration
 *       How long the Portal has been untouched.
 */
Ambient.prototype.interactionUpdate_ = function(inactive_duration) {
  var inactive_secs = this.durationToSeconds_(inactive_duration);
  if (inactive_secs > this.OCCUPANCY_TIMEOUT) {
    this.startAmbientMode_();
  } else {
    this.stopAmbientMode_();
  }
};

/**
 * Starts ambient mode, immediately, if not already in ambient mode.
 */
Ambient.prototype.startAmbientMode_ = function() {
  if (!this.isAmbient_) {
    console.debug('starting ambient mode');
    this.isAmbient_ = true;
    this.runContent_();
  }
};

/**
 * Stops ambient mode, immediately, if presently in ambient mode.
 */
Ambient.prototype.stopAmbientMode_ = function() {
  var self = this;

  if (this.isAmbient_) {
    console.debug('stopping ambient mode');
    this.isAmbient_ = false;
    clearTimeout(this.contentTimer);
    // only exit content if another runway selection has not been made
    setTimeout(function() {
      var now = Date.now();

      if ((self.lastRunwaySelect_ + self.INTERACTION_DELAY * 1000) < now) {
        self.dispatchWindowEvent_(new CustomEvent('acmeExitContent'));
      }
    }, this.INTERACTION_DELAY * 1000);
  }
};
