/**
 * Helpers for StatsD messaging.
 */
var StatsD = StatsD || {
  /**
   * The port for messaging back to the extension.
   * @private
   */
  port: chrome.extension.connect(),

  /**
   * Verify that an object is a string.
   * @private
   * @param {object} s Object to be validated.
   * @return {boolean} True if it is a valid string.
   */
  _isString: function(s) {
    return toString.call(s) == '[object String]';
  },

  /**
   * Verify that an object is a Number.
   * @private
   * @param {object} n Object to be validated.
   * @return {boolean} True if it is a valid Number.
   */
  _isNumber: function(n) {
    return !isNaN(parseFloat(n)) && isFinite(n);
  },

  /**
   * Generic, parent constructor for a StatsD object.
   * @private
   * @namespace
   * @constructor
   * @param {string} name The name of the metric.
   * @param {string} type The string representation of the statsd metric type.
   */
  _primitive: function(name, type) {
    if (!StatsD._isString(name)) {
      throw 'Invalid name to statsd object';
    }

    if (!StatsD._isString(type)) {
      throw 'Invalid type to statsd object';
    }

    var stat_msg = {
      name: name,
      type: type
    };

    /**
     * Sends an update to the extension.
     * @public
     * @memberof _primitive
     * @param {Number} value The value of the metric.
     * @param {Number} [rate] Update frequency.
     */
    function update(value, rate) {
      if (!StatsD._isNumber(value)) {
        throw 'Invalid value to statsd object';
      }

      if (rate && (!StatsD._isNumber(rate) || rate <= 0)) {
        throw 'Invalid rate to statsd object';
      }

      stat_msg['value'] = value;
      stat_msg['rate'] = rate ? rate : 1.0;

      StatsD.port.postMessage(stat_msg);
    }

    return { update: update };
  },

  /**
   * A StatsD Counter metric.
   * @constructor
   * @param {string} name The name of the metric.
   */
  Counter: function(name) {
    return new StatsD._primitive(name, 'c');
  },

  /**
   * A StatsD Timing metric.
   * @constructor
   * @param {string} name The name of the metric.
   */
  Timing: function(name) {
    return new StatsD._primitive(name, 'ms');
  },

  /**
   * A StatsD Gauge metric.
   * @constructor
   * @param {string} name The name of the metric.
   */
  Gauge: function(name) {
    return new StatsD._primitive(name, 'g');
  },

  /**
   * A StatsD Set metric.
   * @constructor
   * @param {string} name The name of the metric.
   */
  Set: function(name) {
    return new StatsD._primitive(name, 's');
  }
};
