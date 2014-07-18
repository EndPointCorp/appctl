/*
 * Onboard on-screen keyboard integration
 */

var port = chrome.extension.connect();

function showOnboard() {
  console.log('Showing Onboard keyboard');
  port.postMessage(true);
}

function hideOnboard() {
  console.log('Hiding Onboard keyboard');
  port.postMessage(false);
}

function addCallbacks() {
  console.log('adding callbacks');
  console.log(document.readyState);

  /* This is needed because Chrome tries loading this plugin when
   * DOM is not ready yet, `even with run_at: document_end`
   */
  if (document.readyState !== 'complete') {
    console.log('Document is not ready yet, going to sleep for a while.');
    setTimeout(addCallbacks, 500);
    return;
  }

  var tx = document.querySelector('#searchboxinput');
  if (tx) {
    console.log('adding onboard event');
    tx.addEventListener('click', showOnboard);
    tx.addEventListener('blur', hideOnboard);
  }
}

console.log('Loading Onboard Extension');
addCallbacks();
