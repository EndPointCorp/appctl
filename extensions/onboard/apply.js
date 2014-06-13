/*
 * Onboard on-screen keyboard integration
 */
var ws = new WebSocket('ws://localhost:9999/websocket');

function showOnboard() {
  console.log('Showing Onboard keyboard');
  ws.send(JSON.stringify({'action': 'showKeyboard'}));
}

function hideOnboard() {
  console.log('Hiding Onboard keyboard');
  ws.send(JSON.stringify({'action': 'hideKeyboard'}));
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
  var tx = document.querySelectorAll('textarea');
  var size = 0;
  if (tx) size = tx.length;
  console.log('got ' + size + ' textarea elements');
  for (var i = 0; i < tx.length; i++) {
    console.log('adding onboard event');
    tx[i].addEventListener('focus', showOnboard);
    tx[i].addEventListener('blur', hideOnboard);
  }
  /*
   * At this moment here is every input changed.
   * It should be changed to input[type=text] after
   * changing the input type for the lgcms
   */
  tx = document.querySelectorAll('input');
  size = 0;
  if (tx) size = tx.length;
  console.log('got ' + tx.length + ' input elements');
  for (var i = 0; i < tx.length; i++) {
    console.log('adding onboard event');
    tx[i].addEventListener('focus', showOnboard);
    tx[i].addEventListener('blur', hideOnboard);
  }

}

console.log('Loading Onboard Extension');
addCallbacks();
