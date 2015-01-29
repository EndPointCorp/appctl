/**
 * An error in Google Properties Menu operation.
 *
 * @param {string} message
 *       A description in the error.
 * @author Matt Vollrath <matt@endpoint.com>
 */
function GooglePropertiesMenuError(message) {
  this.name = 'GooglePropertiesMenuError';
  this.message = message || 'Unspecified error';
}
GooglePropertiesMenuError.prototype = new Error();
GooglePropertiesMenuError.prototype.constructor = GooglePropertiesMenuError;
