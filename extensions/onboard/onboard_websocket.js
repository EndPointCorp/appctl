var OnboardApp = (function() {
  var ws = new WebSocket('ws://master:9090');

  ws.onopen = function() {
    console.log('Advertising onboard topic');
    ws.send(JSON.stringify(
      {'op': 'advertise',
        'topic': 'onboard/visibility',
        'type': 'std_msgs/Bool'}
    ));
  };

  chrome.runtime.onConnect.addListener(function(port) {
    console.log('connected', port);
    port.onMessage.addListener(function(msg) {
      console.log('got msg', msg, 'from', port);
      ws.send(JSON.stringify(
        {'op': 'publish',
          'topic': 'onboard/visibility',
          'msg': {'data': msg}}
      ));
    });
  });
})();
