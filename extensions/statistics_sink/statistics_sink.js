/*
 * This application listens for statistics messages from other extensions and
 * sends them to a statsd server.
 *
 * The location of the statsd server and the name of the sending host must be
 * configured via managed preferences.
 *
 * {
 *   "statsdHost": {string},
 *   "statsdPort": {Number},
 *   "myHost": {string}
 * }
 *
 * Message from extensions have the format:
 *
 * {
 *   "name": {string},
 *   "value": {Number},
 *   "type": {string}, # c, g, ms, or s according to statsd metric types
 *   "rate": {Number} [optional]
 * }
 *
 * @see {@link https://github.com/etsy/statsd/blob/master/docs/metric_types.md}
 *
 * @param {object} config The system configuration from managed preferences.
 */
function StatisticsSink(config) {
  console.log('initializing with config', config);

  var socketId;

  /**
   * Converts a string to an ArrayBuffer.
   *
   * @param {string} str
   * @return {ArrayBuffer}
   */
  function str2ab(str) {
    var buf = new ArrayBuffer(str.length);
    var bufView = new Uint8Array(buf);
    for (var i = 0; i < str.length; i++) {
      bufView[i] = str.charCodeAt(i);
    }
    return buf;
  }

  /**
   * Sends a message to a UDP socket.
   *
   * @param {Number} socketId Socket handle.
   * @param {string} msg Message to send to the socket.
   */
  function sendToSocket(socketId, msg) {
    var arrayBuffer = str2ab(msg);

    chrome.sockets.udp.send(
      socketId,
      arrayBuffer,
      config.statsdHost,
      config.statsdPort,
      function(sendInfo) {
        if (sendInfo.bytesSent != arrayBuffer.byteLength) {
          console.error('sending data:', chrome.runtime.lastError.message);
        }
      }
    );
  }

  /**
   * Converts an incoming statistics message to a statsd string.
   *
   * @param {object} msg A statistics message from another extension.
   * @return {string} statsd representation of the statistics message.
   */
  function msgToStatsd(msg) {
    var str = config.myHost + '.' + msg.name;
    str += ':' + msg.value;
    str += '|' + msg.type;
    if (msg.rate) {
      str += '|@' + msg.rate;
    }

    return str;
  }

  // Listen for external messages.
  chrome.runtime.onMessageExternal.addListener(function(msg, sender) {
    var str = msgToStatsd(msg);

    sendToSocket(socketId, str);
  });

  // Create and bind the UDP socket.
  chrome.sockets.udp.create({}, function(socketInfo) {
    socketId = socketInfo.socketId;
    var arrayBuffer = str2ab('foo\n');

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
  });
}

// Load preferences and initialize the statistics sink.
chrome.storage.managed.get({
  statsdHost: '127.0.0.1',
  statsdPort: 8125,
  myHost: 'unknownHost'
}, function(config) {
  StatisticsSink(config);
});
