console.log('doodles injector');

var scriptsForLoading = [
  "libs/jquery-2.1.1.min.js",
  "libs/eventemitter2.min.js",
  "libs/roslib.min.js",
  'contentjs/google_properties_menu.js',
  'contentjs/doodles.js'
];

var injectScript = function(index) {
  // Inject content script.
  if (index == scriptsForLoading.length) {
    console.log("INJECTOR - all scripts loaded");
    return;
  }
  var filePath = scriptsForLoading[index];
  console.log("INJECTOR - loading file " + filePath);
  var s = document.createElement('script');
  s.src = chrome.extension.getURL(filePath);
  (document.head || document.documentElement).appendChild(s);

  s.onload = function() {
    s.parentNode.removeChild(s);
    console.log("INJECTOR - file loaded " + filePath);
    injectScript(index + 1);
  };
};

injectScript(0);

/*
var injectScript = function(filePath) {
  // Inject content script.
  var s = document.createElement('script');
  s.src = chrome.extension.getURL(filePath);
  (document.head || document.documentElement).appendChild(s);
  s.onload = function() {
    s.parentNode.removeChild(s);
  };
};

injectScript("libs/jquery-2.1.1.min.js");
injectScript("libs/eventemitter2.min.js");
injectScript("libs/roslib.min.js");
injectScript('contentjs/google_properties_menu.js');
injectScript('contentjs/doodles.js');
*/
