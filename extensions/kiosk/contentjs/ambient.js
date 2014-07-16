/**
 * Ambient mode state.
 * @constructor
 */
var Ambient = function() {
  this.OCCUPANCY_TIMEOUT = 20000; // milliseconds
  this.JOYSTICK_GUTTER = 0.001;
  this.CONTENT_TIMEOUT = 30000; // milliseconds
  this.isOccupied = false;
  this.lastOccupancy = Date.now();
  this.placeIndex = 0;

  // add listeners for mouse and touchscreen interaction
  window.addEventListener('touchstart', this.handleTouch.bind(this), true);
  window.addEventListener('mousedown', this.handleTouch.bind(this), true);
};

/**
 * Dispatches a global event.
 * @param {Event} ev The event to be dispatched
 */
Ambient.prototype._dispatch = function(ev) {
  window.dispatchEvent(ev);
};

/**
 * Launches the next Famous Places content if the system is still idle.
 */
Ambient.prototype.runContent = function() {
  if (this.isOccupied) return;

  var contentArray = acme.fpContent[this.placeIndex];
  this._dispatch(new CustomEvent('acmeSelectFamousPlaces'));
  this._dispatch(new CustomEvent(
    'acmeLaunchFamousPlacesContent', {detail: contentArray}));
  this.placeIndex++;
  if (this.placeIndex >= acme.fpContent.length) {
    this.placeIndex = 0;
  }

  this._scheduleNextContent();
};

/**
 * Schedules the next run of content.
 */
Ambient.prototype._scheduleNextContent = function() {
  setTimeout(this.runContent.bind(this), this.CONTENT_TIMEOUT);
};

/**
 * Handles an incoming proximity message.
 * @param {ROS::std_msgs/Bool} msg The incoming presence message from ROS.
 */
Ambient.prototype.handlePresenceMessage = function(msg) {
  this._presenceUpdate(msg.data);
};

/**
 * Handles an incoming joystick message.
 * @param {ROS::geometry_msgs/Pose} msg The incoming pose message from ROS.
 */
Ambient.prototype.handleJoystickMessage = function(msg) {
  var presence = false;

  if (
    Math.abs(msg.linear.x) > this.JOYSTICK_GUTTER ||
    Math.abs(msg.linear.y) > this.JOYSTICK_GUTTER ||
    Math.abs(msg.linear.z) > this.JOYSTICK_GUTTER ||
    Math.abs(msg.angular.x) > this.JOYSTICK_GUTTER ||
    Math.abs(msg.angular.y) > this.JOYSTICK_GUTTER ||
    Math.abs(msg.angular.z) > this.JOYSTICK_GUTTER
  ) {
    presence = true;
  }

  this._presenceUpdate(presence);
};

/**
 * Handles a touch interaction.
 */
Ambient.prototype.handleTouch = function() {
  this._presenceUpdate(true);
};

/**
 * Updates system idle tracking with a positive or negative sample.
 * @param {Boolean} presence A presence sample.
 */
Ambient.prototype._presenceUpdate = function(presence) {
  var now = Date.now();

  if (presence) {
    this.lastOccupancy = now;
    if (!this.isOccupied) {
      this._stopAmbientMode();
    }
  } else if (this.isOccupied &&
      !presence &&
      (now - this.lastOccupancy) > this.OCCUPANCY_TIMEOUT) {
    this._startAmbientMode();
  }
};

/**
 * Starts ambient mode, immediately.
 */
Ambient.prototype._startAmbientMode = function() {
  console.debug('starting ambient mode');
  this.isOccupied = false;
  this.runContent();
};

/**
 * Stops ambient mode, immediately.
 */
Ambient.prototype._stopAmbientMode = function() {
  console.debug('stopping ambient mode');
  this.isOccupied = true;
  this._dispatch(new CustomEvent('acmeExitContent'));
};
