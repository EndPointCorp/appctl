/*
 * This injector loads scripts one by one, so we are sure that the next will be
 * loaded after the first is loaded
 */
var scriptsForLoading = [
  'libs/eventemitter2.min.js',
  'libs/roslib.min.js',
  'contentjs/pacman.js'
];

var injectScript = function(index) {
  // Inject content script.

  if (index == scriptsForLoading.length) {
    console.log('INJECTOR - all scripts loaded');
    return;
  }
  var filePath = scriptsForLoading[index];
  console.log('INJECTOR - loading file ' + filePath);

  var s = document.createElement('script');
  s.src = chrome.extension.getURL(filePath);
  (document.head || document.documentElement).appendChild(s);
  s.onload = function() {
    s.parentNode.removeChild(s);
    console.log('INJECTOR - file loaded ' + filePath);
    injectScript(index + 1);
  };
};

injectScript(0);

