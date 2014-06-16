console.log('Slinky Large Display');

/* Acme Namespace */
var acme = acme || {};

/**
 * @constructor
 */
acme.Util = function() {
};

/**
 * @param {string} filePath The location of the file to inject.
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
 * Send a custom event to the page.
 *
 * @param {Object.<{context: string, method: string, args: Array}>} request
 */
acme.Util.sendCustomEvent = function(request) {
    var eventRequest = new CustomEvent('acme-event', {
      'detail': request,
      'canBubble': false,
      'cancelable': false
    });
    document.dispatchEvent(eventRequest);
};

/**
 * Responsible for large display business logic.
 * @constructor
 */
acme.Display = function() {
  /** @type [boolean] */
  this.hasInitialized = false;
};

/**
 * Does all of the needed initial setup.  Also
 * ensures it is only run once.
 */
acme.Display.prototype.initOnce = function() {
  if (this.hasInitialized) return;

  console.log('Display initialized.');
  this.setLargeDisplayMode();

  this.hasInitialized = true;
};

/**
 * Calls into the page to indicate it should set itself up as a
 * "large display."
 */
acme.Display.prototype.setLargeDisplayMode = function() {
  acme.Util.sendCustomEvent({
    method: 'setLargeDisplayMode'
  });
};


/** The ACME Display object. */
acme.display = new acme.Display();
var cameraBuffer = new CameraBuffer(acme.display);

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


var dumpUpdateToScreen = function(message) {
  var stringifiedMessage = JSON.stringify(message);
  var debugArea = document.getElementById('slinkydebug');
  if (!debugArea) {
    debugArea = document.createElement('div');
    debugArea.id = 'slinkydebug';
    debugArea.style.position = 'fixed';
    debugArea.style.bottom = '90px';
    debugArea.style.left = '0px';
    debugArea.style.height = '30px';
    debugArea.style.width = '100%';
    debugArea.style.zIndex = '99999';
    debugArea.style.backgroundColor = 'rgba(242, 242, 242, 0.7)';
    document.body.appendChild(debugArea);
  }
  debugArea.textContent = stringifiedMessage;
};


var ignoreCameraUpdates = false;

var cameraUpdateHandler = function(cameraEvent) {
  // Lazy set the large display mode after we get our first
  // stable camera update.
  acme.display.initOnce();

  if (!ignoreCameraUpdates) {
    cameraBuffer.processCameraUpdate(cameraEvent.detail);
  }
};

/**
 * Move Camera.
 * @param {Pose} pose The Camera Pose.
 * @param {boolean} shouldAnimate True/false to choose animate/warp to pose.
 */
acme.Display.prototype.moveCamera = function(pose, shouldAnimate) {
  acme.Util.sendCustomEvent({
      method: 'moveCamera',
      args: [pose, shouldAnimate]
  });
};

var getPointInfo = function(screenX, screenY) {
  acme.Util.sendCustomEvent({
      method: 'toLocationEvent',
      args: [screenX, screenY]
  });
};

var startsWith = function startsWith(s, str) {
  return s.slice(0, str.length) == str;
};

/* Add tactile event listeners. */
window.addEventListener('acmeCameraUpdate', cameraUpdateHandler, true);
window.addEventListener('acmeStableCameraUpdate', cameraUpdateHandler, true);
window.addEventListener('acmeCameraCallback', cameraUpdateHandler, true);


/** Add ROS Subscriptions.  These are almost last to prevent exception from
 * killing the event listeners when ROS is down. */
var slinkyRosDisplay = new ROSLIB.Ros({
  url: 'ws://miso.mtv:9090'
});

// Globe View Topic listens and publishes camera updates.
var globeViewTopic = new ROSLIB.Topic({
  ros: slinkyRosDisplay,
  name: '/globe/view',
  messageType: 'geometry_msgs/PoseStamped',
  throttle_rate: 30
});

var globeViewSubscriber = function(message) {
  dumpUpdateToScreen(message);
  var pose = new Pose(message.pose.position.y,
                      message.pose.position.x,
                      message.pose.position.z,
                      message.pose.orientation.z,
                      message.pose.orientation.x,
                      message.pose.orientation.y);
  cameraBuffer.requestWarpToCameraPose(pose);
};
globeViewTopic.subscribe(globeViewSubscriber);


var runwayContentTopic = new ROSLIB.Topic({
  ros: slinkyRosDisplay,
  name: '/globe/runway',
  // TODO(daden): quick hack to get the string across, we should create our own
  // ROS message for passing this data across in the future.
  messageType: 'std_msgs/String'
});


var runwayHandler = function(clicked, customData, runwayImageType) {
  if (clicked && runwayImageType == InputSupport_.DISABLED) {
    ignoreCameraUpdates = true;
  } else {
    ignoreCameraUpdates = false;
  }
};


var runwayContentSubscriber = function(message) {
  console.log(message);
  var runwayContentEvents = {
    CLICK: 'click!!',
    EXIT: 'exit!!'
  };
  var data = message.data;
  if (startsWith(data, runwayContentEvents.CLICK)) {
    var customDataStr = data.slice(
        runwayContentEvents.CLICK.length, data.length);
    var dataObj = JSON.parse(customDataStr);
    var wireVersion = dataObj[0];
    var customData;
    if (wireVersion == 1) {
      customData = dataObj[1];
      var runwayImageType = dataObj[2];
      runwayHandler(true, customData, runwayImageType);
    } else {
      // TODO(paulby) remove this else block once tactile push happens.
      customData = dataObj;
      // For the time being assume we are doing a helicopter tour.
      runwayHandler(true, customData, true);
    }

    acme.Util.sendCustomEvent({
        method: 'launchRunwayContent',
        args: [customData]
    });
  } else if (startsWith(data, runwayContentEvents.EXIT)) {
    runwayHandler(false, null, InputSupport_.NONE);
    acme.Util.sendCustomEvent({
        method: 'exitTitleCard'
    });
  } else {
    console.error('Unhandled runway content data: ' + data);
  }
};
runwayContentTopic.subscribe(runwayContentSubscriber);

var leapListener = new ROSLIB.Topic({
  ros: slinkyRosDisplay,
  name: '/leap_motion/frame',
  messageType: 'leap_motion/Frame',
  throttle_rate: 30
});
// TODO(daden): detect if webGL is available before trying
// to load the hand.  If webGL isn't available this code crashes.
// Init the hand last so if it fails to load it doesn't crash.
var handOverlay = new HandOverlay();
handOverlay.init3js();
leapListener.subscribe(handOverlay.processLeapMessage);
window.addEventListener('acmeScreenLocation',
    handOverlay.processHandGeoLocationEvent.bind(handOverlay), true);
