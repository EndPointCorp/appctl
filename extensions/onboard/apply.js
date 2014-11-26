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
  console.log('adding callbacks');

  /* This is needed because Chrome tries loading this plugin when
   * DOM is not ready yet, `even with run_at: document_end`
   */
  var tx = document.querySelector('#searchboxinput');

  if (document.readyState !== 'complete' || !tx) {
    console.log('Document is not ready yet, onboard is going to sleep for a while.');
    setTimeout(addCallbacks, 100);
  } else {
    console.log('adding onboard event');
    tx.addEventListener('click', showOnboard);
    tx.addEventListener('touchstart', showOnboard);
    tx.addEventListener('blur', hideOnboard);
  }
}

console.log('Loading Onboard Extension');
addCallbacks();
