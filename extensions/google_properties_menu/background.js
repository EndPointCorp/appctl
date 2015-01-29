/**
 * Background communication handler for Google Properties Menu content scripts.
 * This class provides a persistent rosbridge connection for menu injections
 * across tab location changes.
 *
 * @author Szymon Guz <szymon@endpoint.com>
 * @author Matt Vollrath <matt@endpoint.com>
 */
function MenuBackground() {
  /**
   * Initializes the background module by connecting all the ROS and Chrome
   * messaging pieces.
   */
  this.init = function() {
    /* Connection to the rosbridge server. */
    this.portalRos = new ROSLIB.Ros({
      url: 'wss://42-b:9090'
    });

    this.portalRos.on('error', function(error) {
      console.error('ROS error:', error);
    });

    /* Topic for changing the large display url. */
    this.displaySwitchTopic = new ROSLIB.Topic({
      ros: this.portalRos,
      name: '/display/switch',
      messageType: 'std_msgs/String'
    });
    this.displaySwitchTopic.advertise();

    /* Topic for switching appctl mode. */
    this.modeTopic = new ROSLIB.Topic({
      ros: this.portalRos,
      name: '/appctl/mode',
      messageType: 'appctl/Mode'
    });
    this.modeTopic.advertise();

    /* Topic for notifying session changes. */
    this.sessionTopic = new ROSLIB.Topic({
      ros: this.portalRos,
      name: '/statistics/session',
      messageType: 'statistics/Session'
    });
    this.sessionTopic.advertise();

    /* A service providing Portal configuration. */
    this.configSvc = new ROSLIB.Service({
      ros: this.portalRos,
      name: '/portal_config/query',
      messageType: 'portal_config/PortalConfig'
    });

    /* A request to use when retrieving configuration. */
    this.configRequest = new ROSLIB.ServiceRequest({});

    /* Subscribe the message receiver. */
    chrome.runtime.onMessage.addListener(this.receiveContentMessage_.bind(this));

    /*
     * Handlers for messages coming from content scripts.  Parameters and
     * return value according to the callback of chrome.runtime.onMessage
     * event.  Each handler must return true if an async callback is expected,
     * otherwise false.
     * @see https://developer.chrome.com/extensions/runtime#event-onMessage
     */
    this.contentMessageHandlers_ = {
      'change': this.handleContentChangeMessage_.bind(this),
      'config': this.handleContentConfigMessage_.bind(this)
    };
  };

  /**
   * Retrieves configuration from the portal_config service.
   * @param {function} cb
   *       Callback to run with the configuration response.
   * @private
   */
  this.getConfig_ = function(cb) {
    this.configSvc.callService(this.configRequest, function(response) {
      if (!response.hasOwnProperty('json')) {
        throw new GooglePropertiesMenuError(
           'Configuration response was missing the "json" key!'
        );
      }
      var config = JSON.parse(response.json);
      cb(config);
    });
  };

  /**
   * Sends an appctl mode change to the topic.
   * @param {string} mode
   *       The mode to switch to.
   * @private
   */
  this.sendMode_ = function(mode) {
    var modeMsg = new ROSLIB.Message({mode: mode});
    console.debug('Switching to mode', modeMsg.mode);
    this.modeTopic.publish(modeMsg);
  };

  /**
   * Sends a large display url change to the topic.
   * @param {string} url
   *       The url to switch to.
   * @private
   */
  this.sendURL_ = function(url) {
    console.log('Trying to switch display to', url);
    var switchMsg = new ROSLIB.Message({data: url});
    this.displaySwitchTopic.publish(switchMsg);
  };

  /**
   * Sends a session start triggered by menu item selection to the topic.
   * @param {string} id
   *       The id of the menu item.
   * @private
   */
  this.sendSessionStart_ = function(id) {
    var now = (Date.now() / 1000) | 0;
    console.log('Starting a session of', id, 'at', now, 'seconds');
    // TODO(mv): send id too when the node supports it
    var sessionMsg = new ROSLIB.Message({
      start_ts: now
    });
    this.sessionTopic.publish(sessionMsg);
  };

  /**
   * Handles a change message from a content script.  Sends appropriate data
   * to the large display url and appctl mode topics.
   * @see https://developer.chrome.com/extensions/runtime#event-onMessage
   * @param msg
   * @param sender
   * @param sendResponse
   * @return {boolean} False, because an async callback is not expected.
   * @private
   */
  this.handleContentChangeMessage_ = function(msg, sender, sendResponse) {
    if (msg.hasOwnProperty('display_url')) {
      this.sendURL_(msg.display_url);
    }
    if (msg.hasOwnProperty('mode')) {
      this.sendMode_(msg.mode);
    }
    this.sendSessionStart_(msg.id);
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
   * @private
   */
  this.handleContentConfigMessage_ = function(msg, sender, sendResponse) {
    this.getConfig_(function(config) {
      sendResponse({config: config});
    });
    return true;
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
   * @private
   */
  this.receiveContentMessage_ = function(msg, sender, sendResponse) {
    if (!msg.hasOwnProperty('type')) {
      throw new GooglePropertiesMenuError(
        'Got a message with no type!'
      );
    }
    if (!this.contentMessageHandlers_.hasOwnProperty(msg.type)) {
      throw new GooglePropertiesMenuError(
        'Got a message with an unrecognized type!'
      );
    }

    return this.contentMessageHandlers_[msg.type](
      msg.data,
      sender,
      sendResponse
    );
  };
}

var menuBackground = new MenuBackground();
menuBackground.init();
