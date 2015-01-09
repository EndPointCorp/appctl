/**
 * Controls the Google Properties Menu and communication with the extension
 * background page.
 *
 * @author Szymon Guz <szymon@endpoint.com>
 * @author Matt Vollrath <matt@endpoint.com>
 */
var GooglePropertiesMenu = function () {
  /**
   * Fetches configuration from the background page and runs it back to the
   * callback.
   * @param cb {function}
   *       The callback to hit with the configuration data.
   */
  this.fetchConfigData = function(cb) {
    chrome.runtime.sendMessage({type: 'config'}, function(msg) {
      var config = msg.config;
      if ((! 'portal' in config) || (! 'morefun_items' in config.portal)) {
        throw 'Could not find morefun_items in config.portal!';
      }

      var contentItems = config.portal.morefun_items;
      cb(contentItems);
    });
  };

  this.handleContentAction = function (content) {
    if (! 'action' in content) {
      return;
    }

    // TODO(mv): make a structure of action handlers

    // Show the arcade page if the action is provided.
    if (content.action == 'showDoodles') {
      window.location = chrome.extension.getURL('pages/doodles.html');
    }
  };

  this.sendContentToBackground = function(content) {
    var msg = {
      type: 'change',
      data: content
    };
    chrome.runtime.sendMessage(msg);
  }

  /**
   * Handles a content selection from user interaction with the menu.
   * @param e
   *       The touch/click event.
   */
  this.handleContentSelection = function (e) {
    var i = e.target.getAttribute('index');

    if (!i) {
      throw 'Content element had no index!';
    }

    var content = this.contentItems[i];

    // Notify the background page of the selection so it can send messages to
    // the ROS network.

    this.sendContentToBackground(content);

    // Local actions can be taken after sending the selection to the background
    // page.

    // Process custom actions.
    this.handleContentAction(content);

    // Change the page if a kiosk_url is provided.
    if ('kiosk_url' in content) {
      window.location = content.kiosk_url;
    }
  };

  this.handleDoodleSelection = function(e) {
    var url = e.target.getAttribute('switch_url');
    if (!url) {
      throw 'Doodle element had no switch_url!';
    }
    this.sendContentToBackground({display_url: url});
  }

  /**
   * Creates a DOM list from the given list of content items.
   * @param items {object}
   *       A list of content items from the Portal configuration.
   */
  this.createElementsList = function(items) {

    var ul = document.createElement('ul');
    for (var i in items) {

      var name = items[i].name;
      var icon = chrome.extension.getURL(items[i].icon);

      var li = document.createElement('li');
      li.setAttribute('index', i);

      var img = document.createElement('img');
      img.setAttribute('src', icon);
      img.setAttribute('index', i);

      var span = document.createElement('span');
      span.innerText = name;
      span.setAttribute('index', i);

      li.appendChild(img);
      li.appendChild(span);

      li.addEventListener(
        'touchstart',
        this.handleContentSelection.bind(this),
        true
      );
      li.addEventListener(
        'click',
        this.handleContentSelection.bind(this),
        true
      );

      ul.appendChild(li);
    }

    return ul;
  };

  /**
   * Initializes the menu given a list of content items.
   * @param items {object}
   *       A list of content items from the Portal configuration.
   */
  this.initializeMoreFunMenu = function(items) {

    var onclick = function() {
      document.getElementById('morefun_items').style.visibility = 'visible';
    };

    var i = document.createElement('img');
    i.setAttribute('src', chrome.extension.getURL('images/icon_grid_grey.png'));

    var s = document.createElement('div');
    s.id = 'morefun';
    var span = document.createElement('span');
    span.innerText = 'More fun';
    s.addEventListener('click', onclick, true);
    s.addEventListener('touchstart', onclick, true);
    s.appendChild(span);
    s.appendChild(i);

    document.body.appendChild(s);

    var d = document.createElement('div');
    d.id = 'morefun_items';

    var t = document.createElement('img');
    t.setAttribute('src', chrome.extension.getURL("images/arrow.png"));
    t.setAttribute('id', "arrow");

    d.appendChild(t);

    d.appendChild(this.createElementsList(items));
    d.style.visibility = 'hidden';

    document.body.appendChild(d);

    // and we should close the morefun_items window when clicked anywhere else
    var closeHandler = function(e) {
      if (e.target.id != 'morefun_items')
        document.getElementById('morefun_items').style.visibility = 'hidden';
    };
    document.body.addEventListener('click', closeHandler, true);
    document.body.addEventListener('touchstart', closeHandler, true);
  };

  /* Fetch configuration and populate the menu. */
  this.fetchConfigData(function(items) {
    this.contentItems = items;
    this.initializeMoreFunMenu(items);
  }.bind(this));
};

var propertiesMenu = new GooglePropertiesMenu();
