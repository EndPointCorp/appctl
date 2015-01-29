(function() {

var port = chrome.runtime.connect();

function showOnboard() {
  port.postMessage({show: true});
}

function hideOnboard() {
  port.postMessage({show: false});
}

// Adds callbacks to the search field.
// onclick - shows keyboard
// onblur  - hides keyboard
function addCallbacks() {
  /* This is needed because Chrome tries loading this plugin when
   * DOM is not ready yet, `even with run_at: document_end`
   */
  var searchbox = document.querySelector('#searchboxinput');
  var password = document.querySelector('#password');


  if (document.readyState !== 'complete' || !searchbox || !password) {
    setTimeout(addCallbacks, 100);
  } else {

    searchbox.addEventListener('click', showOnboard);
    searchbox.addEventListener('touchstart', showOnboard);
    searchbox.addEventListener('blur', hideOnboard);

    password.addEventListener('click', showOnboard);
    password.addEventListener('touchstart', showOnboard);
    password.addEventListener('blur', hideOnboard);

  }
}

addCallbacks();

})();
