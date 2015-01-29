// Saves options to chrome.storage
function save_options() {
  var wsUrl = document.getElementById('wsUrl').value;
  var rate = Number(document.getElementById('rate').value);
  var socketDebug = document.getElementById('socketDebug').checked;

  chrome.storage.sync.set({
    wsUrl: wsUrl,
    rate: rate,
    socketDebug: socketDebug
  }, function() {
    // Update status to let user know options were saved.
    var status = document.getElementById('status');
    status.textContent = 'Options saved.';
    setTimeout(function() {
      status.textContent = '';
    }, 750);
  });
}

// Restores select box and checkbox state using the preferences
// stored in chrome.storage.
function restore_options() {
  // Use default value color = 'red' and likesColor = true.
  chrome.storage.sync.get({
    wsUrl: 'ws://localhost:9090',
    rate: 60,
    socketDebug: false
  }, function(items) {
    document.getElementById('wsUrl').value = items.wsUrl;
    document.getElementById('rate').value = items.rate;
    document.getElementById('socketDebug').checked = items.socketDebug;
  });
}

document.addEventListener('DOMContentLoaded', restore_options);
document.getElementById('save').addEventListener('click',
    save_options);
