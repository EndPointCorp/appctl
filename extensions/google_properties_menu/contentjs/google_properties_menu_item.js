/**
 * An item of Google Properties Menu content.  Validates required keys.
 *
 * Required attrs:
 *   id - canonical content name, all lower alphanumeric string.
 *   name - display name.
 *   icon - path to icon image in this extension.
 * Optional attrs:
 *   kiosk_url - url to switch kiosk page to when item is selected.
 *   display_url - url to switch the display to when item is selected.
 *   action - string indicating a special action to take when item is selected.
 *   mode - appctl mode for the item.
 *
 * @param {object} attrs
 *       Attributes from the configuration.
 * @author Matt Vollrath <matt@endpoint.com>
 */
function GooglePropertiesMenuItem(attrs) {
  if(obj == null || typeof(obj) != 'object') {
    throw new GooglePropertiesMenuError(
      'GooglePropertiesMenuItem requires an object as first parameter!',
      attrs
    );
  }

  function requiredKey(key) {
    if (!attrs.hasOwnProperty(key)) {
      throw new GooglePropertiesMenuError(
        'GooglePropertiesMenuItem requires "{}" property!'.replace('{}', key),
        attrs
      );
    }
  }

  requiredKey('id');
  requiredKey('name');
  requiredKey('icon');

  for (var k in attrs) {
    if (attrs.hasOwnProperty(k)) {
      this[k] = attrs[k];
    }
  }
}
