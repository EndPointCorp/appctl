/**
 * An App for collecting render statistics from any active tab.
 * @static
 * @namespace
 * @author Matt Vollrath <matt@endpoint.com>
 */
var RenderStatisticsApp = (function() {
  /**
   * The extension id of the statistics sink.
   */
  var SINK_ID = 'gpdloooncmepbcfklhgbjiodfnamgjdj';

  /**
   * Validates a tab as something we want to track rendering on.
   *
   * Intended to filter out chrome:// and anything else we don't want to track.
   * @param {chrome.tabs.Tab} tab The tab to validate.
   * @return {boolean} True if it's valid.
   */
  function validateTab(tab) {
    return /^https?:\/\//.test(tab.url);
  }

  /**
   * Injects a script from this app's directory into a tab.
   * @param {chrome.tabs.Tab} tab The tab to inject into.
   * @param {string} file The filename of the script to inject.
   * @param {function} [callback] To be called when injection is complete.
   */
  function injectScript(tab, file, callback) {
    chrome.tabs.executeScript(tab.id, { file: file }, callback);
  }

  /**
   * Injects code into a tab.
   * @param {chrome.tabs.Tab} tab The tab to inject into.
   * @param {string} code The code to inject.
   * @param {function} [callback] To be called when injection is complete.
   */
  function injectCode(tab, code, callback) {
    chrome.tabs.executeScript(tab.id, { code: code }, callback);
  }

  /**
   * Initializes render statistics for a tab.
   * @param {chrome.tabs.Tab} tab The tab to initialize.
   * @param {function} [callback] To be called when initialization is complete.
   */
  function initRenderStatistics(tab, callback) {
    if (!validateTab(tab)) {
      return;
    }

    console.log('init for', tab);
    injectScript(tab, 'statsd_msg.js', function() {
      injectScript(tab, 'init_counter.js', callback);
    });
  }

  /**
   * Starts render statistics for a tab.
   * @param {chrome.tabs.Tab} tab The tab to start tracking.
   * @param {function} [callback] To be called when finished.
   */
  function startRenderStatistics(tab, callback) {
    if (!validateTab(tab)) {
      return;
    }

    injectCode(tab, 'RenderStatistics.getInstance().start();', callback);
  }

  /**
   * Stops render statistics for a tab.
   * @param {chrome.tabs.Tab} tab The tab to stop tracking.
   * @param {function} [callback] To be called when finished.
   */
  function stopRenderStatistics(tab, callback) {
    if (!validateTab(tab)) {
      return;
    }

    injectCode(tab, 'RenderStatistics.getInstance().stop();', callback);
  }

  /**
   * Starts or stops render statistics for a tab based on its state.
   * @param {chrome.tabs.Tab} tab The tab to check.
   * @param {function} [callback] To be called when finished.
   */
  function controlRenderStatistics(tab, callback) {
    if (tab.active) {
      startRenderStatistics(tab, callback);
    } else {
      stopRenderStatistics(tab, callback);
    }
  }

  /**
   * Runs a function on all tabs.
   * @param {function} fn To be run on all tabs.
   */
  function allTabs(fn) {
    chrome.tabs.query({}, function(tabs) {
      for (var t in tabs) {
        fn(tabs[t]);
      }
    });
  }

  // Handle tab creation.
  chrome.tabs.onCreated.addListener(function(tab) {
    initRenderStatistics(tab, function() {
      controlRenderStatistics(tab);
    });
  });

  // Handle tab updates, including url changes.
  chrome.tabs.onUpdated.addListener(function(tabId, change, tab) {
    if (change.status == 'loading') {
      initRenderStatistics(tab, function() {
        controlRenderStatistics(tab);
      });
    }
  });

  // Handle tab activation by controlling all tabs.
  chrome.tabs.onActivated.addListener(function(tab) {
    allTabs(controlRenderStatistics);
  });

  // Listen for statistics messages from tabs.
  chrome.runtime.onConnect.addListener(function(port) {
    console.log('connected', port);
    port.onMessage.addListener(function(msg) {
      chrome.runtime.sendMessage(SINK_ID, msg);
    });
  });

  // Initialize all tabs.
  allTabs(initRenderStatistics);
})();
