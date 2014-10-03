var portalRos = new ROSLIB.Ros({
  url: 'wss://42-b:9090'
});

var displaySwitchTopic = new ROSLIB.Topic({
  ros: portalRos,
  name: '/display/switch',
  messageType: 'std_msgs/String'
});

displaySwitchTopic.advertise();

var sendURL = function(url) {
  console.log('Trying to switch display to', url);
  var msg = new ROSLIB.Message({data: url});
  displaySwitchTopic.publish(msg);
};

chrome.runtime.onMessage.addListener(
  function(request) {
    sendURL(request.url);
  }
);

