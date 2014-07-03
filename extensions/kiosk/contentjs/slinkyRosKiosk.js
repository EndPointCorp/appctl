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

acme.fpContent = [
  [4, ['0x132f61b6638ccc9f:0x9559ad432c2467a0', 0, null, null, 'Arch of Constantine', 848.10455, 18, '//g0.gstatic.com/heli/image/0x132f61b6638ccc9f_0x9559ad432c2467a0.jpg', [null, null, null, null, null, '', []], 'Via di San Gregorio, Rome, Italy', null, null, [], ''], null, null, ['0x132f61b6638ccc9f:0x9559ad432c2467a0', 0, 18, 'Arch of Constantine', 'Via di San Gregorio, Rome, Italy', 848.10455, ['//g0.gstatic.com/heli/image/0x132f61b6638ccc9f_0x9559ad432c2467a0.jpg', 'Arch of Constantine'], null, [[3, 12.490644464962784, 41.889732371812144], [0, 90, 0], null, 75], '_0-jU9bTJsWJjAKrt4DAAQ', '0CAwQzCcoBg', null, null, null, null, [[['0x132f61b6638ccc9f:0x9559ad432c2467a0']], [['0x132f61b6638ccc9f:0x9559ad432c2467a0']]], null, ['Via di San Gregorio, Rome, Italy']]],
  [4, ['0x60196290556df7cf:0x8d5003885b877511', 0, null, null, 'Mt. Fuji', 864.9559, 18, '//g0.gstatic.com/heli/image/0x60196290556df7cf_0x8d5003885b877511.jpg', [null, null, null, null, null, '', []], 'Fujiyama', null, null, [], ''], null, null, ['0x60196290556df7cf:0x8d5003885b877511', 0, 18, 'Mt. Fuji', 'Fujiyama', 864.9559, ['//g0.gstatic.com/heli/image/0x60196290556df7cf_0x8d5003885b877511.jpg', 'Mt. Fuji'], null, [[3, 138.72778318005282, 35.36055376635301], [0, 90, 0], null, 75], '0lijU5a5C8WJjAKrt4DAAQ', '0CCcQzCcoFQ', null, null, null, null, [[['0x60196290556df7cf:0x8d5003885b877511']], [['0x60196290556df7cf:0x8d5003885b877511']]], null, ['Fujiyama']]],
  //[4,['0x8747e1e59ab82d8d:0xb32b17af1d5c42d',0,null,null,'Moab',881.25494,18,'//g0.gstatic.com/heli/image/0x8747e1e59ab82d8d_0xb32b17af1d5c42d.jpg',[null,null,null,null,null,'',[]],null,null,null,[],''],null,null,['0x8747e1e59ab82d8d:0xb32b17af1d5c42d',0,18,'Moab',null,881.25494,['//g0.gstatic.com/heli/image/0x8747e1e59ab82d8d_0xb32b17af1d5c42d.jpg','Moab'],null,[[3,-109.54984286927848,38.573314645206366],[0,90,0],null,75],'SF6jU_zcLMWJjAKrt4DAAQ','0CAcQzCcoAw',null,null,null,null,[[['0x8747e1e59ab82d8d:0xb32b17af1d5c42d']],[['0x8747e1e59ab82d8d:0xb32b17af1d5c42d']]]]],
  [4, ['0xd0cbf7d62978dd3:0xc3bed493e3ac6bbd', 0, null, null, 'Rock of Gibraltar', 853.2115, 18, '//g0.gstatic.com/heli/image/0xd0cbf7d62978dd3_0xc3bed493e3ac6bbd.jpg', [null, null, null, null, null, '', []], null, null, null, [], ''], null, null, ['0xd0cbf7d62978dd3:0xc3bed493e3ac6bbd', 0, 18, 'Rock of Gibraltar', null, 853.2115, ['//g0.gstatic.com/heli/image/0xd0cbf7d62978dd3_0xc3bed493e3ac6bbd.jpg', 'Rock of Gibraltar'], null, [[3, -5.343499215592419, 36.144107820079036], [0, 90, 0], null, 75], 'lFmjU9SyA8WJjAKrt4DAAQ', '0CBcQzCcoCw', null, null, null, null, [[['0xd0cbf7ddd3b0ff7:0x10c2257a5c95e67']], [['0xd0cbf7ddd3b0ff7:0x10c2257a5c95e67']]]]],
  [4, ['0x478f336499c0d2f1:0x1d00ff8937290620', 0, null, null, 'Matterhorn', 861.6677, 18, '//g0.gstatic.com/heli/image/0x478f336499c0d2f1_0x1d00ff8937290620.jpg', [null, null, null, null, null, '', []], null, null, null, [], ''], null, null, ['0x478f336499c0d2f1:0x1d00ff8937290620', 0, 18, 'Matterhorn', null, 861.6677, ['//g0.gstatic.com/heli/image/0x478f336499c0d2f1_0x1d00ff8937290620.jpg', 'Matterhorn'], null, [[3, 7.658448685050038, 45.97643282175824], [0, 90, 0], null, 75], 'RWGjU47GAcWJjAKrt4DAAQ', '0CBMQzCcoBw', null, null, null, null, [[['0x478f3368cbb9ecd9:0x9826458cace55849']], [['0x478f3368cbb9ecd9:0x9826458cace55849']]]]]
];

/**
 * Add the famous places runway content.
 */
acme.Kiosk.prototype.addFamousPlacesRunwayContent = function() {
  var famousPlacesRunway = document.querySelector('#acme-famous-places');

  for (var i in acme.fpContent) {
    famousPlacesRunway.appendChild(this.createRunwayCard(acme.fpContent[i]));
  }
};


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

var slinkyRosKiosk = new ROSLIB.Ros({
  url: 'ws://master:9090'
});

var joystickTopic = new ROSLIB.Topic({
  ros: slinkyRosKiosk,
  name: '/joystick/twist',
  messageType: 'geometry_msgs/Twist',
  throttle_rate: 30
});

var navigatorListener = new ROSLIB.Topic({
  ros: slinkyRosKiosk,
  name: '/slinky_nav/kiosk_goto_pose',
  messageType: 'geometry_msgs/PoseStamped'
});

var slinkyKioskCurrentPoseTopic = new ROSLIB.Topic({
  ros: slinkyRosKiosk,
  name: '/slinky_kiosk/current_pose',
  messageType: 'slinky_nav/SlinkyPose'
});

var runwayContentTopic = new ROSLIB.Topic({
  ros: slinkyRosKiosk,
  name: '/slinky_kiosk/runway',
  // TODO(daden): quick hack to get the string across, we should create our own
  // ROS message for passing this data across in the future.
  messageType: 'std_msgs/String'
});

var proximityPresenceTopic = new ROSLIB.Topic({
  ros: slinkyRosKiosk,
  name: '/proximity/presence',
  messageType: 'std_msgs/Bool'
});

var dumpUpdateToScreen = function(message) {
  var stringifiedMessage = JSON.stringify(message);
  var debugArea = document.getElementById('slinkydebug');
  if (!debugArea) {
    debugArea = document.createElement('div');
    debugArea.id = 'slinkydebug';
    debugArea.className = 'acme-slinky-debug';
    document.body.appendChild(debugArea);
  }
  debugArea.textContent = stringifiedMessage;
};

var runwayActionRestrictions = InputSupport_.NONE;

navigatorListener.subscribe(function(rosPoseStamped) {
  var pose = new Pose(rosPoseStamped.pose.position.y,  // lat
                      rosPoseStamped.pose.position.x,  // lon
                      rosPoseStamped.pose.position.z,  // alt
                      rosPoseStamped.pose.orientation.z,  // heading
                      rosPoseStamped.pose.orientation.x,  // tilt
                      rosPoseStamped.pose.orientation.y);  // roll
  acme.kiosk.moveCamera(pose, false);
});

var publishKioskCurrentPose = function(pose) {
  // In normal navigation mode, we tell the navigator that it can use the
  // full range of motion:
  var EPSILON = 0.000001;
  var NOMINAL_LAT_MIN = -90.0 + EPSILON;
  var NOMINAL_LAT_MAX = 90.0 - EPSILON;
  var NOMINAL_LON_MIN = -180.0;
  var NOMINAL_LON_MAX = 180.0;
  var NOMINAL_ALT_MIN = 80;
  var NOMINAL_ALT_MAX = Pose.SPACE_POSE.alt;
  var NOMINAL_TILT_MIN = 0.0;
  var NOMINAL_TILT_MAX = 90.0;
  var NOMINAL_HDG_MIN = 0.0;
  var NOMINAL_HDG_MAX = 360.0;

  var slinkyPose = new ROSLIB.Message({
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
      slinkyPose.pose_minimums.position.x =
          slinkyPose.pose_maximums.position.x =
          slinkyPose.current_pose.position.x;
      slinkyPose.pose_minimums.position.y =
          slinkyPose.pose_maximums.position.y =
          slinkyPose.current_pose.position.y;
      slinkyPose.pose_minimums.position.z =
          slinkyPose.pose_maximums.position.z =
          slinkyPose.current_pose.position.z;
      slinkyPose.pose_minimums.orientation.x =
          slinkyPose.pose_maximums.orientation.x =
          slinkyPose.current_pose.orientation.x;
      slinkyPose.pose_minimums.orientation.z =
          slinkyPose.pose_maximums.orientation.z =
          slinkyPose.current_pose.orientation.z;
      break;

    case InputSupport_.NO_ZOOM:
      // Alt is fixed to current.
      slinkyPose.pose_minimums.position.z =
          slinkyPose.pose_maximums.position.z =
          slinkyPose.current_pose.position.z;
      break;

    case InputSupport_.NO_ZOOM_NO_PAN:
      // Lat, Lon, and Alt are fixed to current.
      slinkyPose.pose_minimums.position.x =
          slinkyPose.pose_maximums.position.x =
          slinkyPose.current_pose.position.x;
      slinkyPose.pose_minimums.position.y =
          slinkyPose.pose_maximums.position.y =
          slinkyPose.current_pose.position.y;
      slinkyPose.pose_minimums.position.z =
          slinkyPose.pose_maximums.position.z =
          slinkyPose.current_pose.position.z;

      // HACK(paulby) assume this is a photosphere.
      // Tilt is allowed up to 180 in a photosphere.
      slinkyPose.pose_maximums.orientation.x = 180;
      break;

    default:
      break;
  }

  slinkyKioskCurrentPoseTopic.publish(slinkyPose);
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
  // restriction.
  if (customData[1] && customData[1][0] == 3) {
    // Its a planet shift.  Allow the nav.
    runwayActionRestrictions = InputSupport_.NONE;
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
  console.log('runwayContentExitHandler');
  runwayActionRestrictions = InputSupport_.NONE;

  // TODO(daden): Generate an ESC keydown event on the large display extension.
  // AcmeKeyboard.keydown(27);
  var runwayMsg = new ROSLIB.Message({
    data: 'exit!!' + JSON.stringify(customData)
  });
  runwayContentTopic.publish(runwayMsg);
};

var soundFX = new SoundFX();
navigatorListener.subscribe(soundFX.handlePoseChange.bind(soundFX));

var ambient = new Ambient();
proximityPresenceTopic.subscribe(ambient.handlePresenceMessage.bind(ambient));
joystickTopic.subscribe(ambient.handleJoystickMessage.bind(ambient));

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
}
window.addEventListener('acmeExitContent', exitContent, true);

var launchFamousPlacesContent = function(e) {
  var place = e.detail;

  acme.kiosk.sendCustomEvent_({
    method: 'launchFamousPlacesContent',
    args: [place]
  });
}
window.addEventListener('acmeLaunchFamousPlacesContent', launchFamousPlacesContent, true);

var selectFamousPlaces = function() {
  acme.kiosk.sendCustomEvent_({
    method: 'selectFamousPlacesButton'
  });
}
window.addEventListener('acmeSelectFamousPlaces', selectFamousPlaces, true);

// prevent browser from picking up touch gestures
var preventDefaultHandler = function(e) {
  e.preventDefault();
}
window.addEventListener('touchmove', preventDefaultHandler, false);
