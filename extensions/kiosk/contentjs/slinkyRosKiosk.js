console.log('Slinky Kiosk');

/* Acme Namespace */
var acme = acme || {};

/**
 * @constructor
 */
acme.Util = function() {
};

/**
 * @param {string} filePath The extension file path of the script to inject.
 */
acme.Util.injectScript = function(filePath) {
  // Inject content script.
  var s = document.createElement('script');
  s.src = chrome.extension.getURL(filePath);
  (document.head || document.documentElement).appendChild(s);
  s.onload = function() {
      s.parentNode.removeChild(s);
  };
};

acme.Util.injectScript('contentjs/inject.js');

/**
* Keep this in sync with core:tactile.acme.InputSupport_
* @enum {number}
* @private
*/
var InputSupport_ = {
  NONE: 0,
  DISABLED: 1,
  NO_ZOOM: 2,
  NO_ZOOM_NO_PAN: 3
};

/**
 * Lazy set the large display mode after we get our first stable camera update.
 * @constructor
 */
acme.Kiosk = function() {
  /** @type {boolean} */
  this.hasInitialized = false;
};

/**
 * Zoom out all to put the whole earth in the view.
 */
acme.Kiosk.prototype.addZoomOutToEarthButton = function() {
  var button = document.createElement('button');
  button.className = 'acme-zoom-out-earth';
  button.setAttribute('onclick',
      'javascript:document.dispatchEvent(new CustomEvent("acmeZoomOutToEarth"))'
      );
  var runway = document.querySelector('.widget-runway-thumbstrip-background');
  runway.appendChild(button);
};

/**
 * Inject all of the scripts and stylesheets used by this extension.
 */
acme.Kiosk.prototype.initOnce = function() {
  if (this.hasInitialized) { return; }

  console.log('acme.Kiosk.initOnce');
  this.addZoomOutToEarthButton();
  this.setKioskMode();

  this.hasInitialized = true;
};

/**
 * Send a custom event to the page.
 *
 * @param {Object.<{context: string, method: string, args: Array}>} request
 * @private
 */
acme.Kiosk.prototype.sendCustomEvent_ = function(request) {
    var kioskRequest = new CustomEvent('acme-kiosk', {
      'detail': request,
      'canBubble': false,
      'cancelable': false
    });
    document.dispatchEvent(kioskRequest);
};


/** Set the Kiosk Mode. */
acme.Kiosk.prototype.setKioskMode = function() {
  this.sendCustomEvent_({
    method: 'setKioskMode'
  });
};

/**
 * Move Camera.
 * @param {Pose} pose The Camera Pose.
 * @param {boolean} shouldAnimate True/false to choose animate/warp to pose.
 */
acme.Kiosk.prototype.moveCamera = function(pose, shouldAnimate) {
  this.sendCustomEvent_({
    method: 'moveCamera',
    args: [pose, shouldAnimate]
  });
};

/** Handle a getPhotoSet call from the runway. */
acme.Kiosk.prototype.getPhotoSetHandler = function() {
  var content = [{
    id: 'YouTube-itgyMHwS01Q',
    title: 'Explore the world with Lonely Planet',
    subtitle: 'YouTube',
    thumbnailUrl: 'http://img.youtube.com/vi/itgyMHwS01Q/0.jpg',
    url: 'https://www.youtube.com/watch?v=itgyMHwS01Q',
    label: 'YouTube: Explore the world with Lonely Planet'
  }];
  // TODO(daden): Add back in custom content when doing famous places.
  content.length = 0;
  this.sendCustomEvent_({
    method: 'addRunwayContent',
    args: [content]
  });
};

/** The ACME Kiosk object. */
acme.kiosk = new acme.Kiosk();
var cameraBuffer = new CameraBuffer(acme.kiosk);
var joystickNavigator = new JoystickNavigator(cameraBuffer);

var slinkyRosKiosk = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

var spaceNavListener = new ROSLIB.Topic({
  ros: slinkyRosKiosk,
  name: '/spacenav/twist',
  messageType: 'geometry_msgs/Twist',
  throttle_rate: 30
});

var joystickTopic = new ROSLIB.Topic({
      ros: slinkyRosKiosk,
      name: '/joystick/twist',
      messageType: 'geometry_msgs/Twist',
      throttle_rate: 30
    });

var globeViewTopic = new ROSLIB.Topic({
  ros: slinkyRosKiosk,
  name: '/globe/view',
  messageType: 'geometry_msgs/PoseStamped',
  throttle_rate: 30
});

var runwayContentTopic = new ROSLIB.Topic({
  ros: slinkyRosKiosk,
  name: '/globe/runway',
  // TODO(daden): quick hack to get the string across, we should create our own
  // ROS message for passing this data across in the future.
  messageType: 'std_msgs/String'
});

var dumpUpdateToScreen = function(message) {
  var stringifiedMessage = JSON.stringify(message);
  var debugArea = document.getElementById('slinkydebug');
  if (!debugArea) {
    debugArea = document.createElement('div');
    debugArea.id = 'slinkydebug';
    debugArea.class = 'acme-slinky-debug';
    document.body.appendChild(debugArea);
  }
  debugArea.textContent = stringifiedMessage;
};

var runwayActionRestrictions = InputSupport_.NONE;

spaceNavListener.subscribe(function(twist) {
      var maxTilt = 90;
      var minAlt = 80;

      switch (runwayActionRestrictions) {
        case InputSupport_.DISABLED:
          return;
        case InputSupport_.NO_ZOOM:
          twist.linear.z = 0;
          break;
        case InputSupport_.NO_ZOOM_NO_PAN:
          twist.linear.x = 0;
          twist.linear.y = 0;
          twist.linear.z = 0;

          // HACK(paulby) assume this is a photosphere
          maxTilt = 180;
          minAlt = 0;
          break;
        default:
          break;
      }

      // The SpaceNav twist values range [-350, 350], so it must be
      // normalized for the joystick code, which expects [-1.0, 1.0].
      var SPACE_NAV_SCALE = 350.0;
      // The SpaceNav ROS driver swaps both linear and angular x/y.
      // It also sign-flips the y for both linear and angular.
      var normalizedJoy = new Twist(
        twist.linear.y / SPACE_NAV_SCALE * -1.0,
        twist.linear.x / SPACE_NAV_SCALE,
        twist.linear.z / SPACE_NAV_SCALE,
        twist.angular.y / SPACE_NAV_SCALE * -1.0,
        twist.angular.x / SPACE_NAV_SCALE,
        twist.angular.z / SPACE_NAV_SCALE);

      joystickNavigator.processJoy(normalizedJoy, maxTilt, minAlt);
      joystickTopic.publish(new ROSLIB.Message(normalizedJoy));
    });

var publishGlobeView = function(pose) {
  var viewUpdate = new ROSLIB.Message({
     pose: {
       position: {
         x: pose.lon,
         y: pose.lat,
         z: pose.alt
       },
       orientation: {
         x: pose.tilt,
         y: pose.roll,
         z: pose.heading,
         w: 0
       }
     }
   });
  globeViewTopic.publish(viewUpdate);
};

/**
 * Handle updates from the tactile camera.
 * @param {Object} cameraEvent The most recent camera event.
 */
var cameraUpdateHandler = function(cameraEvent) {
  acme.kiosk.initOnce();
  joystickNavigator.processCameraUpdate(cameraEvent.detail);
  publishGlobeView(cameraEvent.detail);
};

var runwayContentClickHandler = function(e) {
  var camera = e.detail;
  var customData = e.detail.customData;

  console.log('runwayContentClickedHandler');
  runwayActionRestrictions = customData[2];

  // TODO(daden): Create a method on the large display extension.
  //window.location.href = 'javascript:' +
  //    'acme.launchRunwayContent(' + customData + ');';

  // TODO(daden): Use a real ROS object to get a header, camera pose, and
  // the custom content data.  Stringifying and parsing the JSON isn't great.
  var runwayMsg = new ROSLIB.Message({
    data: 'click!!' + JSON.stringify(customData)
  });
  runwayContentTopic.publish(runwayMsg);
};

var runwayContentExitHandler = function(e) {
  var camera = e.detail;
  var customData = e.detail.customData;

  console.log('runwayContentExitHandler');
  runwayActionRestrictions = InputSupport_.NONE;

  // TODO(daden): Generate an ESC keydown event on the large display extension.
  // AcmeKeyboard.keydown(27);
  var runwayMsg = new ROSLIB.Message({
    data: 'exit!!' + JSON.stringify(customData)
  });
  runwayContentTopic.publish(runwayMsg);
};

// Setup the sound effects.
soundFX = new SoundFX();
joystickTopic.subscribe(soundFX.handlePoseChange.bind(soundFX));

window.addEventListener('acmeCameraUpdate', cameraUpdateHandler, true);
window.addEventListener('acmeStableCameraUpdate', cameraUpdateHandler, true);
window.addEventListener('acmeGetPhotoSet',
    acme.kiosk.getPhotoSetHandler.bind(acme.kiosk), true);
window.addEventListener('acmeContentClicked', runwayContentClickHandler, true);
window.addEventListener('acmeContentOnExit', runwayContentExitHandler, true);

var zoomOutToEarth = function() {
  cameraBuffer.requestAnimateToCameraPose(Pose.SPACE_POSE);
};
// document.dispatchEvent(new CustomEvent('acmeZoomOutToEarth'))
window.addEventListener('acmeZoomOutToEarth', zoomOutToEarth, true);
