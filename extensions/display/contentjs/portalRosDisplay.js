console.log('Portal Large Display');

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

/*
 * Load the style overrides.
 */
var portalStyleOverrides = document.createElement('link');
portalStyleOverrides.setAttribute('rel', 'stylesheet');
portalStyleOverrides.setAttribute('type', 'text/css');
portalStyleOverrides.setAttribute('href',
    chrome.extension.getURL('css/acme_display.css'));
document.getElementsByTagName('head')[0].appendChild(portalStyleOverrides);

/**
 * @constructor
 * @param {number} lat Latitude in degrees N, (-90, 90).
 * @param {number} lon Longitude in degrees E, [-180, 180).
 * @param {number} alt Altitude (meters).
 * @param {number} heading Heading in degrees, [0-360).
 * @param {number} tilt Tilt in degrees, [0 == down, 90 == horizon).
 * @param {number} roll Roll (unused).
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
 * Planet values.
 */
Planet = {
  EARTH: 1,
  MOON: 2,
  MARS: 3
};

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
  var debugArea = document.getElementById('portaldebug');
  if (!debugArea) {
    debugArea = document.createElement('div');
    debugArea.id = 'portaldebug';
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

var publishDisplayCurrentPose = function(pose) {
  var portalPose = new ROSLIB.Message({
    current_pose: {
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

  if (handOverlay) {
    handOverlay.setCurrentCameraPose(pose);
  }
  portalDisplayCurrentPoseTopic.publish(portalPose);
};

var cameraUpdateHandler = function(cameraEvent) {
  // Lazy set the large display mode after we get our first
  // stable camera update.
  acme.display.initOnce();

  publishDisplayCurrentPose(cameraEvent.detail);
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
var portalRosDisplay = new ROSLIB.Ros({
  url: 'wss://42-b:9090'
});

// Globe View Topic listens and publishes camera updates.
var navigatorListener = new ROSLIB.Topic({
  ros: portalRosDisplay,
  name: '/portal_nav/display_goto_pose',
  messageType: 'geometry_msgs/PoseStamped',
  throttle_rate: 30,
  queue_length: 2
});

var portalDisplayCurrentPoseTopic = new ROSLIB.Topic({
  ros: portalRosDisplay,
  name: '/portal_display/current_pose',
  messageType: 'portal_nav/PortalPose'
});

portalDisplayCurrentPoseTopic.advertise();

// Controls whether or not we listen to moveto events.  When on a tour
// we should not listen to moveto events.
var ignoreCameraUpdates = false;

navigatorListener.subscribe(function(rosPoseStamped) {
  if (!ignoreCameraUpdates) {
    var pose = new Pose(rosPoseStamped.pose.position.y,  // lat
                        rosPoseStamped.pose.position.x,  // lon
                        rosPoseStamped.pose.position.z,  // alt
                        rosPoseStamped.pose.orientation.z,  // heading
                        rosPoseStamped.pose.orientation.x,  // tilt
                        rosPoseStamped.pose.orientation.y);  // roll
    acme.display.moveCamera(pose, false);
  }
});


var runwayContentTopic = new ROSLIB.Topic({
  ros: portalRosDisplay,
  name: '/portal_kiosk/runway',
  // TODO(daden): quick hack to get the string across, we should create our own
  // ROS message for passing this data across in the future.
  messageType: 'std_msgs/String'
});

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
    var customData = JSON.parse(customDataStr);
    var sceneContentArray = customData[1];
    var runwayImageType = customData[2];

    var planetChange = sceneContentArray[7];

    // Check to see if this is the type of runway element that should
    // not use the pose information coming from anywhere.
    if (!planetChange && runwayImageType == InputSupport_.DISABLED) {
      ignoreCameraUpdates = true;
    } else {
      ignoreCameraUpdates = false;
    }

    // disable HUD unless changing planet to Earth
    if (planetChange && planetChange == Planet.EARTH) {
      handOverlay.enabled = true;
    } else {
      handOverlay.enabled = false;
    }

    // disable spacenav feedback unless changing planets
    if (runwayImageType != InputSupport_.NONE && !planetChange) {
      spacenavFeedback.enabled = false;
    }

    acme.Util.sendCustomEvent({
        method: 'launchRunwayContent',
        args: [sceneContentArray]
    });
  } else if (startsWith(data, runwayContentEvents.EXIT)) {
    // we assume there is no runway content on Moon or Mars
    handOverlay.enabled = true;
    spacenavFeedback.enabled = true;
    ignoreCameraUpdates = false;
    acme.Util.sendCustomEvent({
        method: 'exitTitleCard'
    });
  } else {
    console.error('Unhandled runway content data: ' + data);
  }
};
runwayContentTopic.subscribe(runwayContentSubscriber);

var populationService = new ROSLIB.Service({
  ros: portalRosDisplay,
  name: '/geodata/population',
  serviceType: 'geodata/GeodataQuery'
});

window.addEventListener('acmePopulationQuery', function(ev) {
  var populationRequest = new ROSLIB.ServiceRequest({
    layer: 'population',
    point: {
      latitude: ev.detail.latitude,
      longitude: ev.detail.longitude,
      altitude: 0
    },
    radius: ev.detail.radius
  });

  populationService.callService(
    populationRequest,
    ev.detail.callback,
    function(err) {
      console.error('querying population service:', err);
    }
  );
}, true);

/**
 * A common WebGL environment for visual modules.
 */
acme.glEnvironment = new PortalGLEnvironment();

var leapListener = new ROSLIB.Topic({
  ros: portalRosDisplay,
  name: '/leap_motion/frame',
  messageType: 'leap_motion/Frame',
  throttle_rate: 30
});

// TODO(daden): detect if webGL is available before trying
// to load the hand.  If webGL isn't available this code crashes.
// Init the hand last so if it fails to load it doesn't crash.
var handOverlay = new HandOverlay(acme.glEnvironment);
handOverlay.init3js();
leapListener.subscribe(handOverlay.processLeapMessage);
window.addEventListener('acmeScreenLocation',
    handOverlay.processHandGeoLocationEvent.bind(handOverlay), true);

var spacenavListener = new ROSLIB.Topic({
  ros: portalRosDisplay,
  name: '/spacenav/twist',
  messageType: 'geometry_msgs/Twist',
  throttle_rate: 30
});

var spacenavFeedback = new SpacenavFeedback(acme.glEnvironment);
spacenavFeedback.init();
spacenavListener.subscribe(
  spacenavFeedback.processSpacenavMessage.bind(spacenavFeedback)
);

