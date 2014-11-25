console.log('Portal Kiosk');

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

/*
 * Load the style overrides.
 */
var portalStyleOverrides = document.createElement('link');
portalStyleOverrides.setAttribute('rel', 'stylesheet');
portalStyleOverrides.setAttribute('type', 'text/css');
portalStyleOverrides.setAttribute('href',
    chrome.extension.getURL('css/acme_kiosk.css'));
document.getElementsByTagName('head')[0].appendChild(portalStyleOverrides);

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
 * Planet values.
 */
Planet = {
  EARTH: 1,
  MOON: 2,
  MARS: 3
};

/**
 * Lazy set the large display mode after we get our first stable camera update.
 * @constructor
 */
acme.Kiosk = function() {
  /** @type {boolean} */
  this.hasInitialized = false;
};

acme.Kiosk.prototype.hideZoomButtons = function() {
  document.getElementById('zoom').style.visibility="hidden";
}

acme.Kiosk.prototype.showZoomButtons = function() {
  document.getElementById('zoom').style.visibility="visible";
}

/**
 * Zoom out all to put the whole earth in the view.
 */
acme.Kiosk.prototype.addZoomOutToEarthButton = function() {
  var button = document.createElement('button');
  button.className = 'acme-zoom-out-earth';
  /*
   * JUST the background-image style, so we can use an asset from within the
   * extension. ALL other style attributes sould be handled in the extension
   * CSS
   */
  var zoomIcon = chrome.extension.getURL('images/zoomOut.jpg');
  button.setAttribute('style', 'background-image:url(\'' + zoomIcon + '\');');
  button.setAttribute('onclick',
      'javascript:document.dispatchEvent(new CustomEvent("acmeZoomOutToEarth"))'
      );
  var runway = document.querySelector('.widget-runway-thumbstrip-background');
  if (runway != null) {
    runway.appendChild(button);
  } else {
    setTimeout(this.addZoomOutToEarthButton.bind(this), 100);
    return;
  }
};

/**
 * Inject all of the scripts and stylesheets used by this extension.
 */
acme.Kiosk.prototype.initOnce = function() {
  if (this.hasInitialized) { return; }

  console.log('acme.Kiosk.initOnce');
  this.addZoomOutToEarthButton();
  this.createFamousPlacesRunway();
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


/**
 * Create the famous places runway.
 */
acme.Kiosk.prototype.createFamousPlacesRunway = function() {
  var poiRunway = document.querySelector('.widget-runway-subview-card-list');
  // Check to see if we were created too soon.
  if (!poiRunway) {
    var self = this;
    var tryAgain = function() {
      self.createFamousPlacesRunway();
    };
    setTimeout(tryAgain, 100);
    return;
  }

  poiRunway.id = 'acme-points-of-interest';

  var famousPlacesRunway = document.createElement('ul');
  famousPlacesRunway.id = 'acme-famous-places';
  famousPlacesRunway.className = 'acme-famous-places ' +
      'widget-runway-subview-card-list ' +
      'widget-runway-all-cards ';
  famousPlacesRunway.style.cssText = 'display: none !important;';

  var famousPlacesButton = document.createElement('button');
  famousPlacesButton.id = 'acme-famous-places-button';
  famousPlacesButton.className = 'acme-famous-places-button';
  famousPlacesButton.innerHTML = 'Famous Places';
  famousPlacesButton.setAttribute('onclick',
      'javascript:acmeExt.selectFamousPlacesButton();');

  var poiButton = document.createElement('button');
  poiButton.id = 'acme-poi-button';
  poiButton.className = 'acme-poi-button acme-button-selected';
  poiButton.innerHTML = 'Points of Interest';
  poiButton.setAttribute('onclick',
      'javascript:acmeExt.selectPOIButton();');

  var viewcardStrip = document.querySelector('.app-viewcard-strip');
  var viewcard = document.querySelector('#viewcard');
  viewcardStrip.insertBefore(poiButton, viewcard);
  viewcardStrip.insertBefore(famousPlacesButton, viewcard);

  var innerRunway = document.querySelector('.widget-runway-tray-wrapper');
  innerRunway.appendChild(famousPlacesRunway);

  this.addFamousPlacesRunwayContent();
};


/**
 * Add the famous places runway content.
 */
acme.Kiosk.prototype.addFamousPlacesRunwayContent = function() {
  var famousPlacesRunway = document.querySelector('#acme-famous-places');

  this.removeFamousPlacesRunwayContent();
  for (var i in acme.fpContent) {
    famousPlacesRunway.appendChild(this.createRunwayCard(acme.fpContent[i]));
  }
};

/**
 * Remove the famous places runway content.
 */
acme.Kiosk.prototype.removeFamousPlacesRunwayContent = function() {
  var famousPlacesRunway = document.querySelector('#acme-famous-places');
  var children = famousPlacesRunway.children;
  for (var i = children.length - 1; i >= 0; i--) {
    children[i].remove();
  }
}

/** @type {string} */
acme.Kiosk.RUNWAY_CARD_TEMPLATE =
    '<span class="widget-runway-card-view-pane widget-runway-card-view-pane-visible widget-runway-card-view-pane-foreground">' +
    '  <li class="widget-runway-card">' +
    '    <button aria-label="{{title}}" class="widget-runway-card-button" onclick="javascript:acmeExt.launchFamousPlacesContent({{array}});">' +
    '      <div class="widget-runway-card-background-flicker-hack-clip">' +
    '        <div class="widget-runway-card-background-flicker-hack-wrapper">' +
    '          <img src="{{imgUrl}}" class="widget-runway-card-background-flicker-hack">' +
    '        </div>' +
    '      </div>' +
    '      <div class="widget-runway-card-caption-wrapper">' +
    '        <div class="widget-runway-card-caption">' +
    '          <div class="widget-runway-card-caption-icon widget-runway-card-caption-icon-{{tourType}}"></div>' +
    '          <label class="widget-runway-card-caption-text">{{title}}</label>' +
    '        </div>' +
    '      </div>' +
    '      <div class="widget-runway-card-highlight" style="display:none"></div>' +
    '    </button>' +
    '  </li>' +
    '</span>';


/**
 * Create a single Runway Card from the card data.
 *
 * @param {Object} cardData The card data.
 * @return {HtmlElement} The card view container.
 */
acme.Kiosk.prototype.createRunwayCard = function(cardData) {
  var title = cardData[4][3];
  var tourType = function(contentType) {
    switch (contentType) {
      case 18:
        return 'earth-tour';
      case 11:
        return 'photo-sphere';
      default:
        console.error('Unknown Content Type');
    }
  };
  var imgUrl = cardData[4][6][0];
  var array = JSON.stringify(cardData).replace(/"/g, '\'');

  var card = acme.Kiosk.RUNWAY_CARD_TEMPLATE
      .replace(/{{title}}/g, title)
      .replace(/{{tourType}}/g, tourType(cardData[4][2]))
      .replace(/{{imgUrl}}/g, imgUrl)
      .replace(/{{array}}/g, array);

  var cardViewContainer = document.createElement('span');
  cardViewContainer.className = 'widget-runway-subview-card-view-container';
  cardViewContainer.innerHTML = card;

  return cardViewContainer;
};


/** The ACME Kiosk object. */
acme.kiosk = new acme.Kiosk();

var portalRosKiosk = new ROSLIB.Ros({
  url: 'wss://42-b:9090'
});

var joystickTopic = new ROSLIB.Topic({
  ros: portalRosKiosk,
  name: '/joystick/twist',
  messageType: 'geometry_msgs/Twist',
  throttle_rate: 30
});

var navigatorListener = new ROSLIB.Topic({
  ros: portalRosKiosk,
  name: '/portal_nav/kiosk_goto_pose',
  messageType: 'geometry_msgs/PoseStamped',
  throttle_rate: 30,
  queue_length: 2
});

var portalKioskCurrentPoseTopic = new ROSLIB.Topic({
  ros: portalRosKiosk,
  name: '/portal_kiosk/current_pose',
  messageType: 'portal_nav/PortalPose'
});

portalKioskCurrentPoseTopic.prevPublish = portalKioskCurrentPoseTopic.publish;
portalKioskCurrentPoseTopic.publish = function(obj) {
  portalKioskCurrentPoseTopic.prevPublish(obj);
};

var runwayContentTopic = new ROSLIB.Topic({
  ros: portalRosKiosk,
  name: '/portal_kiosk/runway',
  // TODO(daden): quick hack to get the string across, we should create our own
  // ROS message for passing this data across in the future.
  messageType: 'std_msgs/String'
});

runwayContentTopic.advertise();

runwayContentTopic.prevPublish = runwayContentTopic.publish;
runwayContentTopic.publish = function(obj) {
  runwayContentTopic.prevPublish(obj);
}

var proximityPresenceTopic = new ROSLIB.Topic({
  ros: portalRosKiosk,
  name: '/proximity/presence',
  messageType: 'std_msgs/Bool'
});

var dumpUpdateToScreen = function(message) {
  var stringifiedMessage = JSON.stringify(message);
  var debugArea = document.getElementById('portaldebug');
  if (!debugArea) {
    debugArea = document.createElement('div');
    debugArea.id = 'portaldebug';
    debugArea.className = 'acme-portal-debug';
    document.body.appendChild(debugArea);
  }
  debugArea.textContent = stringifiedMessage;
};

var runwayActionRestrictions = InputSupport_.NONE;

var handleRosPoseChange = function(rosPoseStamped) {
  // ignore pose changes if input is supposed to be disabled
  if (runwayActionRestrictions === InputSupport_.DISABLED) {
    return;
  }

  var pose = new Pose(rosPoseStamped.pose.position.y,  // lat
                      rosPoseStamped.pose.position.x,  // lon
                      rosPoseStamped.pose.position.z,  // alt
                      rosPoseStamped.pose.orientation.z,  // heading
                      rosPoseStamped.pose.orientation.x,  // tilt
                      rosPoseStamped.pose.orientation.y);  // roll
  acme.kiosk.moveCamera(pose, false);
};

var publishKioskCurrentPose = function(pose) {
  // In normal navigation mode, we tell the navigator that it can use the
  // full range of motion:
  var EARTH_RADIUS = 6378100; //meters
  var EPSILON = 0.000001;
  var NOMINAL_LAT_MIN = -90.0 + EPSILON;
  var NOMINAL_LAT_MAX = 90.0 - EPSILON;
  var NOMINAL_LON_MIN = -180.0;
  var NOMINAL_LON_MAX = 180.0;
  var NOMINAL_ALT_MIN = 80;
  var NOMINAL_ALT_MAX = Pose.SPACE_POSE.alt;
  var NOMINAL_TILT_MIN = 0.0;
  var NOMINAL_TILT_MAX = Math.asin(
      EARTH_RADIUS / (EARTH_RADIUS + pose.alt)
    ) * (180 / Math.PI) + 9;
  var NOMINAL_HDG_MIN = 0.0;
  var NOMINAL_HDG_MAX = 360.0;

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
    },
    pose_minimums: {
      position: {
        x: NOMINAL_LON_MIN,
        y: NOMINAL_LAT_MIN,
        z: NOMINAL_ALT_MIN
      },
      orientation: {
        x: NOMINAL_TILT_MIN,
        y: 0,
        z: NOMINAL_HDG_MIN,
        w: 0
      }
    },
    pose_maximums: {
      position: {
        x: NOMINAL_LON_MAX,
        y: NOMINAL_LAT_MAX,
        z: NOMINAL_ALT_MAX
      },
      orientation: {
        x: NOMINAL_TILT_MAX,
        y: 0,
        z: NOMINAL_HDG_MAX,
        w: 0
      }
    }
  });

  // In some constrained modes, we change the min/max.
  switch (runwayActionRestrictions) {
    case InputSupport_.DISABLED:
      // All pose settings fixed to current.
      portalPose.pose_minimums.position.x =
          portalPose.pose_maximums.position.x =
          portalPose.current_pose.position.x;
      portalPose.pose_minimums.position.y =
          portalPose.pose_maximums.position.y =
          portalPose.current_pose.position.y;
      portalPose.pose_minimums.position.z =
          portalPose.pose_maximums.position.z =
          portalPose.current_pose.position.z;
      portalPose.pose_minimums.orientation.x =
          portalPose.pose_maximums.orientation.x =
          portalPose.current_pose.orientation.x;
      portalPose.pose_minimums.orientation.z =
          portalPose.pose_maximums.orientation.z =
          portalPose.current_pose.orientation.z;
      break;

    case InputSupport_.NO_ZOOM:
      // Alt is fixed to current.
      portalPose.pose_minimums.position.z =
          portalPose.pose_maximums.position.z =
          portalPose.current_pose.position.z;
      break;

    case InputSupport_.NO_ZOOM_NO_PAN:
      // Lat, Lon, and Alt are fixed to current.
      portalPose.pose_minimums.position.x =
          portalPose.pose_maximums.position.x =
          portalPose.current_pose.position.x;
      portalPose.pose_minimums.position.y =
          portalPose.pose_maximums.position.y =
          portalPose.current_pose.position.y;
      portalPose.pose_minimums.position.z =
          portalPose.pose_maximums.position.z =
          portalPose.current_pose.position.z;

      // HACK(paulby) assume this is a photosphere.
      // Tilt is allowed up to 180 in a photosphere.
      portalPose.pose_maximums.orientation.x = 180;
      break;

    default:
      break;
  }

  if (runwayActionRestrictions == InputSupport_.NONE) acme.kiosk.showZoomButtons();
  else acme.kiosk.hideZoomButtons();

  portalKioskCurrentPoseTopic.publish(portalPose);
};

/**
 * Handle updates from the tactile camera.
 * @param {Object} cameraEvent The most recent camera event.
 */
var cameraUpdateHandler = function(cameraEvent) {
  acme.kiosk.initOnce();
  publishKioskCurrentPose(cameraEvent.detail);
};

var runwayContentClickHandler = function(e) {
  var camera = e.detail;
  var customData = e.detail.customData;

  // TODO(paulby) remove once the tactile is pushed.
  if (typeof customData === 'string') {
    customData = JSON.parse(customData);
  }

  console.log('runwayContentClickedHandler');
  runwayActionRestrictions = customData[2];

  // Check to see if this is a planet action.  If so, remove the nav
  // restriction and drop points of interest.
  if (customData[1] && customData[1][0] == 3) {
    // disable sound on the moon
    if (customData[1][7] != Planet.MOON) {
      //soundFX.enable();
    } else {
      //soundFX.disable();
    }
    // Its a planet shift.  Allow the nav.
    runwayActionRestrictions = InputSupport_.NONE;

    // If this is a non-earth planet no "Famous places."
    if (customData[1][7] != Planet.EARTH) {
      acme.kiosk.removeFamousPlacesRunwayContent();
    } else {
      acme.kiosk.addFamousPlacesRunwayContent();
    }
  } else {
    // disable sound if there are any special input restrictions
    if (runwayActionRestrictions == InputSupport_.NONE) {
      //soundFX.enable();
    } else {
      //soundFX.disable();
    }
    // Since this is not a planet action.  Make sure the famous
    // places content exists when we're not in space.
    if (!document.location.href.match(/space/)) {
      acme.kiosk.addFamousPlacesRunwayContent();
    }
  }

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

  // TODO(paulby) remove once the tactile is pushed.
  if (typeof customData === 'string') {
    customData = JSON.parse(customData);
  }
  runwayActionRestrictions = InputSupport_.NONE;
  //soundFX.enable();

  // TODO(daden): Generate an ESC keydown event on the large display extension.
  // AcmeKeyboard.keydown(27);
  var runwayMsg = new ROSLIB.Message({
    data: 'exit!!' + JSON.stringify(customData)
  });
  runwayContentTopic.publish(runwayMsg);
};

//var soundFX = new SoundFX();
navigatorListener.subscribe(function(rosPoseStamped) {
  //soundFX.handlePoseChange(rosPoseStamped);
  handleRosPoseChange(rosPoseStamped);
});

/*
var ambient = new Ambient();
proximityPresenceTopic.subscribe(ambient.handlePresenceMessage.bind(ambient));
joystickTopic.subscribe(ambient.handleJoystickMessage.bind(ambient));
*/

window.addEventListener('acmeCameraUpdate', cameraUpdateHandler, true);
window.addEventListener('acmeStableCameraUpdate', cameraUpdateHandler, true);
window.addEventListener('acmeGetPhotoSet',
    acme.kiosk.getPhotoSetHandler.bind(acme.kiosk), true);
window.addEventListener('acmeContentClicked', runwayContentClickHandler, true);
window.addEventListener('acmeContentOnExit', runwayContentExitHandler, true);

var zoomOutToEarth = function() {
  acme.kiosk.moveCamera(Pose.SPACE_POSE, true);
};
// document.dispatchEvent(new CustomEvent('acmeZoomOutToEarth'))
window.addEventListener('acmeZoomOutToEarth', zoomOutToEarth, true);

var exitContent = function() {
  acme.kiosk.sendCustomEvent_({
    method: 'exitTitleCard'
  });
};
window.addEventListener('acmeExitContent', exitContent, true);

var launchFamousPlacesContent = function(e) {
  var place = e.detail;

  acme.kiosk.sendCustomEvent_({
    method: 'launchFamousPlacesContent',
    args: [place]
  });
};
window.addEventListener(
  'acmeLaunchFamousPlacesContent', launchFamousPlacesContent, true
);

var selectFamousPlaces = function() {
  acme.kiosk.sendCustomEvent_({
    method: 'selectFamousPlacesButton'
  });
};
window.addEventListener('acmeSelectFamousPlaces', selectFamousPlaces, true);

// prevent browser from picking up touch gestures
var preventDefaultHandler = function(e) {
  e.preventDefault();
};
window.addEventListener('touchmove', preventDefaultHandler, false);


// This event handler should not allow for pinch events in the street view mode.
document.addEventListener('touchmove', function(e) {
  console.log('touchmove');
  console.log('runwayActionRestriction is: ' + runwayActionRestrictions);

  if (e.touches.length > 1 && runwayActionRestrictions != 0) {
    console.log('Pinch events not allowed');
    e.preventDefault();
    e.stopPropagation();
  }
}, true);
