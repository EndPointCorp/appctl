// Only load once and keep everything private.
var acme = acme || {};

/** The api for interfacing with acme tactile. */
acme.messageAPI = acme.messageAPI || (function() {
  console.log('Loading ACME Display API.');

  /*
   * Mirrors the API found on the acme object and assists in mapping parameters
   * TODO(mamacker): Replace this with inspection-based api to remove code
   * duplication.
   * @constructor
   */
  var MessageAPI = function() {
  };

  /*
   * Listen to messages from the extension and call the appropriate functions.
   * @param e CustomEvent from the extension.
   */
  MessageAPI.prototype.handleEvent = function(e) {
    var request = e.detail, context = this;
//     console.log('acme.MessageAPI');
//     console.log(request);

    if (request.context && acme.hasOwnProperty(request.context)) {
      context = acme[request.context];
    }
    if (typeof context[request.method] != 'function') {
      console.error('Method[' + request.method +
          '] does not exist on context[' + request.context + ']');
      return;
    }
    var method = context[request.method];
    method.apply(context, request.args);
  };

  MessageAPI.prototype.setLargeDisplayMode = function() {
    console.log('acme.setLargeDisplayMode');
    acme.setLargeDisplayMode();
  };

  MessageAPI.prototype.moveCamera = function(pose, shouldAnimate) {
    var acmepose = new acme.CameraPose(pose.lat, pose.lon, pose.alt,
        pose.heading, pose.tilt, pose.roll);
    // 2nd param is a call back that we don't care about.
    acme.moveCamera(acmepose, undefined, shouldAnimate);
  };

  MessageAPI.prototype.addRunwayContent = function(content) {
    acme.addCustomRunwayContent(content);
  };

  MessageAPI.prototype.launchRunwayContent = function(content) {
    acme.launchRunwayContent(content);
  };

  MessageAPI.prototype.exitTitleCard = function() {
    acme.exitTitleCard();
  };

  MessageAPI.prototype.toLocationEvent = function(screenX, screenY) {
    acme.toLocationEvent(screenX, screenY);
  };

  MessageAPI.prototype.elevationQuery = function(lat, lng) {
    var elem = document.getElementById('acmeElevationResult');
    if (!elem) {
      // Maps V3 API not initialized yet.
      return;
    }

    if (!acme.elevationService) {
      acme.elevationService = new google.maps.ElevationService();
    }

    var request = {
      locations: [new google.maps.LatLng(lat, lng)]
    };
    acme.elevationService.getElevationForLocations(request, function(res, s) {
      if (s == google.maps.ElevationStatus.OK) {
        elem.innerHTML = '//' + JSON.stringify(res[0]);
        var ev = new CustomEvent('acmeElevationResult');
        window.dispatchEvent(ev);
      } else {
        console.error('Elevation query:', r, 'status:', s);
      }
    });
  };

  var messageAPI = new MessageAPI();
  document.addEventListener('acme-event',
      messageAPI.handleEvent.bind(messageAPI), true);
  return messageAPI;
})();

/**
 * Callback for initialization of legacy Maps API V3.
 */
acme.MapsV3Init = acme.MapsV3Init || function() {
  var s = document.createElement('script');
  s.id = 'acmeElevationResult';
  (document.head || document.documentElement).appendChild(s);
};
