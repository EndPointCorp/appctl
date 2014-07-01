var Ambient = function() {
  this.OCCUPANCY_TIMEOUT = 20000; // milliseconds
  this.JOYSTICK_GUTTER = 0.001;
  this.CONTENT_TIMEOUT = 30000; // milliseconds
  this.isOccupied = false;
  this.lastOccupancy = Date.now();
  this.placeIndex = 0;
}

Ambient.prototype._dispatch = function(ev) {
  window.dispatchEvent(ev);
}

Ambient.prototype.runContent = function() {
  if (this.isOccupied) return;

  var contentArray = acme.fpContent[this.placeIndex];
  this._dispatch(new CustomEvent('acmeLaunchFamousPlacesContent', {detail: contentArray}));
  this.placeIndex++;
  if (this.placeIndex >= acme.fpContent.length) {
    this.placeIndex = 0;
  }

  this._scheduleNextContent();
}

Ambient.prototype._scheduleNextContent = function() {
  setTimeout(this.runContent.bind(this), this.CONTENT_TIMEOUT);
}

Ambient.prototype.handlePresenceMessage = function(msg) {
  this._presenceUpdate(msg.data);
}

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
}

Ambient.prototype._presenceUpdate = function(presence) {
  var now = Date.now();

  if (presence) {
    this.lastOccupancy = now;
    if (!this.isOccupied) {
      this._stopAmbientMode();
    }
  } else if (this.isOccupied && !presence && (now - this.lastOccupancy) > this.OCCUPANCY_TIMEOUT) {
    this._startAmbientMode();
  }
}

Ambient.prototype._startAmbientMode = function() {
  console.debug('starting ambient mode');
  this.isOccupied = false;
  this.runContent();
}

Ambient.prototype._stopAmbientMode = function() {
  console.debug('stopping ambient mode');
  this.isOccupied = true;
  this._dispatch(new CustomEvent('acmeExitContent'));
}
