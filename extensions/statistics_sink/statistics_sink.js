/**
 * This application listens for statistics messages from other extensions and
 * sends them to a statsd server.
 *
 * <p>
 * The location of the statsd server and the name of the sending host must be
 * configured via managed preferences.
 *
 * <pre><code>
 * {
 *   "statsdHost": {string},
 *   "statsdPort": {Number},
 *   "myHost": {string}
 * }
 * </code></pre>
 *
 * <p>
 * Message from extensions have the format:
 *
 * <pre><code>
 * {
 *   "name": {string},
 *   "value": {Number},
 *   "type": {string}, # c, g, ms, or s according to statsd metric types
 *   "rate": {Number} [optional]
 * }
 * </code></pre>
 *
 * @see https://github.com/etsy/statsd/blob/master/docs/metric_types.md
 *
 * @constructor
 * @param {object} config The system configuration from managed preferences.
 * @author Matt Vollrath <matt@endpoint.com>
 */
function StatisticsSink(config) {
  console.log('initializing StatisticsSink with config', config);

  /**
   * UDP socket handle for sending to statsd server.
   */
  var socketId = null;

  /**
   * Hostname of the statsd server.
   */
  var statsdHost = config.statsdHost;

  /**
   * Port of the statsd server.
   */
  var statsdPort = config.statsdPort;

  /**
   * Hostname of the local machine.
   */
  var myHost = config.myHost;

  /**
   * Converts a string to an ArrayBuffer.
   *
   * @param {string} str
   * @return {ArrayBuffer}
   */
  var str2ab = function(str) {
    var buf = new ArrayBuffer(str.length);
    var bufView = new Uint8Array(buf);
    for (var i = 0; i < str.length; i++) {
      bufView[i] = str.charCodeAt(i);
    }
    return buf;
  };

  /**
   * Sends a message to a UDP socket.
   *
   * @param {Number} socketId Socket handle.
   * @param {string} msg Message to send to the socket.
   */
  var sendToSocket = function(socketId, msg) {
    var arrayBuffer = str2ab(msg);

    chrome.sockets.udp.send(
      socketId,
      arrayBuffer,
      statsdHost,
      statsdPort,
      function(sendInfo) {
        if (sendInfo.bytesSent != arrayBuffer.byteLength) {
          console.error('sending data:', chrome.runtime.lastError.message);
        }
      }
    );
  };

  /**
   * Converts an incoming statistics message to a statsd string.
   *
   * @param {object} msg A statistics message from another extension.
   * @return {string} statsd representation of the statistics message.
   */
  var msgToStatsd = function(msg) {
    var str = myHost + '.' + msg.name;
    str += ':' + msg.value;
    str += '|' + msg.type;
    if (msg.rate) {
      str += '|@' + msg.rate;
    }

    return str;
  };

  /**
   * Handles an incoming stats message.
   *
   * @param {object} msg A statistics message from another extension.
   * @param {chrome.runtime.MessageSender} sender Message sender.
   */
  var handleStatsMessage = function(msg, sender) {
    var str = msgToStatsd(msg);

    sendToSocket(socketId, str);
  };

  /**
   * Handles the creation of a UDP socket.
   *
   * @param {chrome.sockets.udp.SocketInfo} socketInfo
   */
  var handleSocketCreate = function(socketInfo) {
    socketId = socketInfo.socketId;

    chrome.sockets.udp.bind(
      socketId,
      '0.0.0.0',
      0,
      function(result) {
        if (result < 0) {
          console.error('binding socket:', chrome.runtime.lastError.message);
        }
      }
    );
  };

  // Listen for external messages.
  chrome.runtime.onMessageExternal.addListener(handleStatsMessage);

  // Create and bind the UDP socket.
  chrome.sockets.udp.create({}, handleSocketCreate);
}

var instance;

// Load preferences and initialize the statistics sink.
chrome.storage.managed.get({
  statsdHost: '127.0.0.1',
  statsdPort: 8125,
  myHost: 'localhost'
}, function(config) {
  instance = new StatisticsSink(config);
});
