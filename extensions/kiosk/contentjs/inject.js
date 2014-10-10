// Only load once and keep everything private.
var acmeExt = acmeExt || {};

/** The api for interfacing with acme tactile. */
acmeExt.messageAPI = acmeExt.messageAPI || (function() {
  console.log('Loading ACME Kiosk API.');

  var MessageAPI = function() {
  };

  /*
   * Listen to messages from the extension and call the appropriate functions.
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

  MessageAPI.prototype.setKioskMode = function() {
    console.log('acme.setKioskMode');
    acme.setKioskMode();
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

  MessageAPI.prototype.exitTitleCard = function() {
    acme.exitTitleCard();
  };

  MessageAPI.prototype.selectFamousPlacesButton = function() {
    acmeExt.selectFamousPlacesButton();
  };

  MessageAPI.prototype.launchFamousPlacesContent = function(place) {
    acmeExt.launchFamousPlacesContent(place);
  };

  var messageAPI = new MessageAPI();
  document.addEventListener('acme-kiosk',
      messageAPI.handleEvent.bind(messageAPI), true);
  return messageAPI;
})();

/**
 * Handles user selecting the POI tab.
 */
acmeExt.selectPOIButton = function() {
  fp = document.querySelector('#acme-famous-places');
  poi = document.querySelector('#acme-points-of-interest');
  fp.style.cssText = 'display: none !important;';
  poi.style.cssText = '';

  fpb = document.querySelector('#acme-famous-places-button');
  poib = document.querySelector('#acme-poi-button');
  poib.classList.add('acme-button-selected');
  fpb.classList.remove('acme-button-selected');
};

/**
 * Handles user selecting the Famous Places tab.
 */
acmeExt.selectFamousPlacesButton = function() {
  fp = document.querySelector('#acme-famous-places');
  poi = document.querySelector('#acme-points-of-interest');
  fp.style.cssText = '';
  poi.style.cssText = 'display: none !important;';

  fpb = document.querySelector('#acme-famous-places-button');
  poib = document.querySelector('#acme-poi-button');
  poib.classList.remove('acme-button-selected');
  fpb.classList.add('acme-button-selected');
  acme.setKioskMode(); //TODO(kiel): temp hack to force-show runway
};

/**
 * Handles user selection of Famous Places content.
 * @param {Array} array Famous Places content, lifted from Tactile.
 */
acmeExt.launchFamousPlacesContent = function(array) {
  console.log(array);
  array[array.length] = 'iskiosk';
  acme.launchRunwayContent(array);
  var wireProtocol = 1;
  //TODO(kiel): when other types are introduced besides Earth Tours, implement
  //  the appropriate type handlers.
  var inputType = 1; // 1 means disabled
  var contentClickedEvent = new CustomEvent('acmeContentClicked', {
    'detail': {'customData': [wireProtocol, array, inputType]},
    'canBubble': false,
    'cancelable': false
  });

  document.dispatchEvent(contentClickedEvent);
};
