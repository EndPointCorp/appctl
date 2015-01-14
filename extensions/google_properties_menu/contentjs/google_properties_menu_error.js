/**
 * An error in Google Properties Menu operation.
 *
 * @param {string} msg
 *       A description in the error.
 * @param {object} context
 *       The error context.
 * @author Matt Vollrath <matt@endpoint.com>
 */
function GooglePropertiesMenuError(msg, context) {
  this.msg = msg;
  this.context = context;
}
