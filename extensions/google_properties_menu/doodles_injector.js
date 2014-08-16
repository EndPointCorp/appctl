
console.log('doodles injector');

var injectScript = function(filePath) {
  // Inject content script.
  var s = document.createElement('script');
  s.src = chrome.extension.getURL(filePath);
  (document.head || document.documentElement).appendChild(s);
  s.onload = function() {
      s.parentNode.removeChild(s);
  };
};

injectScript("libs/eventemitter2.min.js"),
injectScript("libs/roslib.min.js"),
injectScript('google_properties_menu.js');
