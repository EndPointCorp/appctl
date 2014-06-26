var Ambient = function() {
  this.OCCUPANCY_TIMEOUT = 5000; // milliseconds
  this.JOYSTICK_GUTTER = 0.1;
  this.CONTENT_TIMEOUT = 30000; // milliseconds
  this.isOccupied = false;
  this.lastOccupancy = Date.now();
  this.placeIndex = 0;
}

Ambient.prototype.runContent = function() {
  if (this.isOccupied) return;

  var contentArray = acme.fpContent[this.placeIndex];
  //window.dispatchEvent(new CustomEvent('acmeLaunchFamousPlacesContent', {detail: contentArray}));
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
  var values = [
    msg.linear.x, msg.linear.y, msg.linear.z,
    msg.angular.x, msg.angular.y, msg.angular.z
  ];
  var len = values.length;
  for (var i=0; i < len; i++) {
    if (Math.abs(values[i]) > this.JOYSTICK_GUTTER) {
      presence = true;
      break;
    }
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
  document.dispatchEvent(new CustomEvent('acmeZoomOutToEarth'));
}
