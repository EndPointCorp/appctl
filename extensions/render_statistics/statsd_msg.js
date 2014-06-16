/**
 * Helpers for StatsD messaging.
 *
 * <p>
 * This code is intended to be injected into a tab by an extension.
 *
 * @static
 * @namespace
 * @author Matt Vollrath <matt@endpoint.com>
 */
var StatsD = (function() {
  /**
   * A port for messaging back to the extension.
   */
  var port = chrome.extension.connect();

  /**
   * Verify that an object is a string.
   * @param {object} s Object to be validated.
   * @return {boolean} True if it is a valid string.
   */
  var _isString = function(s) {
    return toString.call(s) == '[object String]';
  };

  /**
   * Verify that an object is a Number.
   * @param {object} n Object to be validated.
   * @return {boolean} True if it is a valid Number.
   */
  var _isNumber = function(n) {
    return !isNaN(parseFloat(n)) && isFinite(n);
  };

  /**
   * Generic, parent constructor for a StatsD object.
   * @constructor
   * @param {string} name The name of the metric.
   * @param {string} type The string representation of the statsd metric type.
   */
  var Primitive = function(name, type) {
    if (!_isString(name)) {
      throw 'Invalid name to statsd object';
    }

    if (!_isString(type)) {
      throw 'Invalid type to statsd object';
    }

    this.stat_msg = {
      name: name,
      type: type
    };

    /**
     * Sends an update to the extension.
     * @param {Number} value The value of the metric.
     * @param {Number} [rate] Update frequency.
     */
    this.update = function(value, rate) {
      if (!_isNumber(value)) {
        throw 'Invalid value to statsd object';
      }

      if (rate && (!_isNumber(rate) || rate <= 0)) {
        throw 'Invalid rate to statsd object';
      }

      this.stat_msg['value'] = value;
      this.stat_msg['rate'] = rate ? rate : 1.0;

      port.postMessage(this.stat_msg);
    };
  };

  /**
   * A StatsD Counter metric.
   * @extends StatsD-Primitive
   * @constructor
   * @param {string} name The name of the metric.
   */
  var Counter = function(name) {
    return new Primitive(name, 'c');
  };

  /**
   * A StatsD Timing metric.
   * @extends StatsD-Primitive
   * @constructor
   * @param {string} name The name of the metric.
   */
  var Timing = function(name) {
    return new Primitive(name, 'ms');
  };

  /**
   * A StatsD Gauge metric.
   * @extends StatsD-Primitive
   * @constructor
   * @param {string} name The name of the metric.
   */
  var Gauge = function(name) {
    return new Primitive(name, 'g');
  };

  /**
   * A StatsD Set metric.
   * @extends StatsD-Primitive
   * @constructor
   * @param {string} name The name of the metric.
   */
  var Set = function(name) {
    return new Primitive(name, 's');
  };

  return {
    Counter: Counter,
    Timing: Timing,
    Gauge: Gauge,
    Set: Set
  };
})();
