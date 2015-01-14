/**
 * Controls the Google Properties menus and communication with the extension
 * background page.
 *
 * @author Szymon Guz <szymon@endpoint.com>
 * @author Matt Vollrath <matt@endpoint.com>
 */
function GooglePropertiesMenu() {
  /**
   * Instance has been initialized?
   * @type {boolean}
   * @private
   */
  this.ready_ = false;

  /**
   * Configuration from the portal_config service.
   * @type {object}
   * @private
   */
  this.config_ = {};

  /**
   * Items in the More Fun menu.
   * @type {GooglePropertiesMenuItem[]}
   * @private
   */
  this.moreFunItems_ = [];

  /**
   * Items in the Doodle menu.
   * @type {GooglePropertiesMenuItem[]}
   * @private
   */
  this.doodleItems_ = [];

  /**
   * Fetches configuration from the background page and runs it back to the
   * callback.
   * @param {function} cb
   *       The callback to hit with the configuration data.
   * @private
   */
  this.fetchConfigData_ = function(cb) {
    chrome.runtime.sendMessage({type: 'config'}, function(msg) {
      var config = msg.config;

      cb(config);
    });
  };

  /**
   * Handles custom actions which can be tagged onto either More Fun or Doodle
   * content items.
   * @param {GooglePropertiesMenuItem} content
   *       An item of More Fun or Doodle content.
   * @private
   */
  this.handleContentAction_ = function (content) {
    if (!content.hasOwnProperty('action')) {
      return;
    }

    // TODO(mv): make a structure of action handlers

    // Show the arcade page if the action is provided.
    if (content.action == 'showDoodles') {
      window.location = chrome.extension.getURL('pages/doodles.html');
    }
  };

  /**
   * Sends a content item to the extension background page for transmission
   * to the ROS backplane.
   * @param {GooglePropertiesMenuItem} content
   *       An item of More Fun or Doodle content.
   * @private
   */
  this.sendSelectionToRos_ = function(content) {
    var msg = {
      type: 'change',
      data: content
    };
    chrome.runtime.sendMessage(msg);
  };

  /**
   * Handles common work needed by both More Fun and Doodle content.
   * @param {GooglePropertiesMenuItem} content
   *       An item of More Fun or Doodle content.
   * @private
   */
  this.handleContentSelection_ = function(content) {
    // Notify the background page of the selection so it can send messages to
    // the ROS network.

    this.sendSelectionToRos_(content);

    // Local actions can be taken after sending the selection to the background
    // page.

    // Process custom actions.
    this.handleContentAction_(content);

    // Change the page if a kiosk_url is provided.
    if (content.hasOwnProperty('kiosk_url')) {
      window.location = content.kiosk_url;
    }
  };

  /**
   * Handles a More Fun content selection from user interaction with the menu.
   * @param {event} e
   *       The touch/click event.
   */
  this.handleMoreFunSelection = function (e) {
    if (!e.hasOwnProperty('target') || ! e.target.getAttribute('index')) {
      throw new GooglePropertiesMenuError(
        'More Fun element had no index!',
        e
      );
    }
    var i = e.target.getAttribute('index');

    if (!this.moreFunItems_.hasOwnProperty(i)) {
      throw new GooglePropertiesMenuError(
        'Bad More Fun index!',
        [e, this.moreFunItems_]
      );
    }
    var content = this.moreFunItems_[i];

    this.handleContentSelection_(content);
  };

  /**
   * Handles a Doodle content selection from user interaction with the menu.
   * Requires jQuery in the environment.
   * @param {event} e
   *       The touch/click event.
   */
  this.handleDoodleSelection = function(e) {
    if (!$) {
      throw new GooglePropertiesMenuError(
        'Can\'t handle Doodle selection without jQuery injection!',
        e
      );
    }

    if (!e.hasOwnProperty('target') || ! e.target.getAttribute('index')) {
      throw new GooglePropertiesMenuError(
        'Doodle element had no index!',
        e
      );
    }
    var i = e.target.getAttribute('index');

    if (!this.doodleItems_.hasOwnProperty(i)) {
      throw new GooglePropertiesMenuError(
        'Bad Doodle index!',
        [e, this.doodleItems_]
      );
    }
    var content = this.doodleItems_[i];

    $('.game').removeClass('selected');
    $('.description').hide();
    $(e).addClass('selected');
    $('#choose_game').hide();
    $('#{}_selected').replace('{}', content.id).show();

    this.handleContentSelection_(content);
  };

  /**
   * Adds the Doodle content items to the page.  Must occur after init().
   */
  this.addDoodleMenuToPage = function() {
    if (!this.ready_) {
      throw new GooglePropertiesMenuError(
        'Properties menu not initialized yet!',
        this
      );
    }

    if (!this.config_.hasOwnProperty('portal')) {
      throw new GooglePropertiesMenuError(
        'No portal section in config!',
        this.config_
      );
    }
    if (!this.config_.portal.hasOwnProperty('doodle_items') {
      throw new GooglePropertiesMenuError(
        'No doodle_items in Portal config!',
        this.config_
      );
    }
    this.doodleItems_ = this.config_.portal.doodle_items.map(function(i) {
      return new GooglePropertiesMenuItem(i);
    });

    var container = document.createElement('div');
    for (var i in items) {
      var item = items[i];

      var name = item.name;
      var icon = chrome.extension.getURL(item.icon);

      var d = document.createElement('div');
      d.setAttribute('index', i);

      var img = document.createElement('img');
      img.className = 'no-pointer';
      img.setAttribute('src', icon);
      img.setAttribute('index', i);

      var span = document.createElement('span');
      span.className = 'no-pointer';
      span.innerText = name;
      span.setAttribute('index', i);

      d.appendChild(img);
      d.appendChild(span);

      d.addEventListener(
        'touchstart',
        this.handleDoodleSelection.bind(this),
        true
      );
      d.addEventListener(
        'click',
        this.handleDoodleSelection.bind(this),
        true
      );

      container.appendChild(d);
    }

    var middle = document.getElementById('middle');
    middle.appendChild(container);
  };

  /**
   * Creates a DOM list from the given list of More Fun content items.
   * @param {GooglePropertiesMenuItem[]} items
   *       A list of content items from the Portal configuration.
   * @private
   */
  this.createMoreFunElementList_ = function(items) {
    var ul = document.createElement('ul');
    for (var i in items) {
      var item = items[i];

      var name = item.name;
      var icon = chrome.extension.getURL(item.icon);

      var li = document.createElement('li');
      li.setAttribute('index', i);

      li.addEventListener(
        'touchstart',
        this.handleMoreFunSelection.bind(this),
        true
      );
      li.addEventListener(
        'click',
        this.handleMoreFunSelection.bind(this),
        true
      );
    }

    return ul;
  };

  /**
   * Adds the More Fun menu to the page.  Must occur after init().
   */
  this.addMoreFunMenuToPage = function() {
    if (!this.ready_) {
      throw new GooglePropertiesMenuError(
        'Properties menu not initialized yet!',
        this
      );
    }

    if (!this.config_.hasOwnProperty('portal') {
      throw new GooglePropertiesMenuError(
        'No portal section in config!',
        this.config_
      );
    }
    if (!this.config_.portal.hasOwnProperty('morefun_items') {
      throw new GooglePropertiesMenuError(
        'No More Fun items in Portal config!',
        this.config_
      );
    }
    this.moreFunItems_ = this.config_.portal.morefun_items.map(function(i) {
      return new GooglePropertiesMenuItem(i);
    });

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

    d.appendChild(this.createMoreFunElementList_(items));
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

  /**
   * Initializes the Google Properties Menu module.
   * @param {function} cb
   *       Callback to run when the module is ready.
   */
  this.init = function(cb) {
    var cb_ = cb;
    /* Fetch configuration and populate the menu. */
    this.fetchConfigData_(function(config) {
      this.config_ = config;
      this.ready_ = true;
      cb_();
    }.bind(this));
  };
}

var acmePropertiesMenu = new GooglePropertiesMenu();
acmePropertiesMenu.init(function() {
  /* Always add the More Fun menu to the page. */
  acmePropertiesMenu.addMoreFunMenuToPage();

  /* Add the Doodle content if it's the right page. */
  if (document.title == 'Doodle Selection') {
    acmePropertiesMenu.addDoodleMenuToPage();
  }
});

