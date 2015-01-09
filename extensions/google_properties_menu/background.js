/**
 * Background communication handler for Google Properties Menu content scripts.
 * This class provides a persistent rosbridge connection for menu injections
 * across tab location changes.
 *
 * @author Szymon Guz <szymon@endpoint.com>
 * @author Matt Vollrath <matt@endpoint.com>
 */
var MenuBackground = (function () {
  /** Connection to the rosbridge server. */
  this.portalRos = new ROSLIB.Ros({
    url: 'wss://42-b:9090'
  });

  this.portalRos.on('error', function(error) {
    console.error('ROS error:', error);
  });

  /** Topic for changing the large display url. */
  this.displaySwitchTopic = new ROSLIB.Topic({
    ros: this.portalRos,
    name: '/display/switch',
    messageType: 'std_msgs/String'
  });
  this.displaySwitchTopic.advertise();

  /** Topic for switching appctl mode. */
  this.modeTopic = new ROSLIB.Topic({
    ros: this.portalRos,
    name: '/appctl/mode',
    messageType: 'appctl/Mode'
  });
  this.modeTopic.advertise();

  /** A service providing Portal configuration. */
  this.configSvc = new ROSLIB.Service({
    ros: this.portalRos,
    name: '/portal_config/query',
    messageType: 'portal_config/PortalConfig'
  });

  /** A static request to use when retrieving configuration. */
  this.configRequest = new ROSLIB.ServiceRequest({});

  /**
   * Retrieves configuration from the portal_config service.
   * @param {function} cb
   *       Callback to run with the configuration response.
   */
  this.getConfig = function(cb) {
    this.configSvc.callService(configRequest, function(response) {
      if (! 'json' in response) {
        throw 'Configuration response was missing the "json" key!';
      }
      var config = JSON.parse(response.json);
      cb(config);
    });
  };

  /**
   * Sends an appctl mode change to the topic.
   * @param {string} mode
   *       The mode to switch to.
   */
  this.sendMode = function(mode) {
    var modeMsg = new ROSLIB.Message({mode: mode});
    console.debug('Switching to mode', modeMsg.mode);
    modeTopic.publish(modeMsg);
  };

  /**
   * Sends a large display url change to the topic.
   * @param {string} url
   *       The url to switch to.
   */
  this.sendURL = function(url) {
    console.log('Trying to switch display to', url);
    var switchMsg = new ROSLIB.Message({data: url});
    displaySwitchTopic.publish(switchMsg);
  };

  /**
   * Handles a change message from a content script.  Sends appropriate data
   * to the large display url and appctl mode topics.
   * @see https://developer.chrome.com/extensions/runtime#event-onMessage
   * @param msg
   * @param sender
   * @param sendResponse
   * @return {boolean} False, because an async callback is not expected.
   */
  this.handleContentChangeMessage = function(msg, sender, sendResponse) {
    if ('display_url' in msg) {
      this.sendURL(msg.display_url);
    }
    if ('mode' in msg) {
      this.sendMode(msg.mode);
    }
    return false;
  };

  /**
   * Handles a config message from a content script.  Forwards the result of
   * a configuration request back to the sender.
   * @see https://developer.chrome.com/extensions/runtime#event-onMessage
   * @param msg
   * @param sender
   * @param sendResponse
   * @return {boolean} True, because an async callback is expected.
   */
  this.handleContentConfigMessage = function(msg, sender, sendResponse) {
    this.getConfig(function(config) {
      sendResponse({config: config});
    });
    return true;
  };

  /**
   * Handlers for messages coming from content scripts.  Parameters and return
   * value according to the callback of chrome.runtime.onMessage event.  Each
   * handler must return true if an async callback is expected, otherwise
   * false.
   * @see https://developer.chrome.com/extensions/runtime#event-onMessage
   */
  this.contentMessageHandlers = {
    'change': this.handleContentChangeMessage.bind(this),
    'config': this.handleContentConfigMessage.bind(this)
  };

  /**
   * Receives a message from a content script, validates, and sends it to the
   * appropriate handler.
   * @see https://developer.chrome.com/extensions/runtime#event-onMessage
   * @param msg
   * @param sender
   * @param sendResponse
   * @return {boolean} The value from the handler.  Must be true if an async
   *         response is expected.
   */
  this.receiveContentMessage = function(msg, sender, sendResponse) {
    if (! 'type' in msg) {
      throw 'Got a message with no type!';
    }
    if (! msg.type in this.contentMessageHandlers) {
      throw 'Got a message with an unrecognized type!';
    }

    return this.contentMessageHandlers[msg.type](
      msg.data,
      sender,
      sendResponse
    );
  };

  /* Subscribe the message receiver. */
  chrome.runtime.onMessage.addListener(this.receiveContentMessage.bind(this));
})();
