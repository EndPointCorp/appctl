var MODES = {
  'file:///mnt/earthtime/data-visualization-tools/examples/webgl-timemachine/landsat.html': 'timelapse',
  'http://10.42.41.210:9988/comGoogleGigapixelControl/index.html': 'gigapixel',
  'default': 'tactile'
}

var portalRos = new ROSLIB.Ros({
  url: 'wss://42-b:9090'
});

var displaySwitchTopic = new ROSLIB.Topic({
  ros: portalRos,
  name: '/display/switch',
  messageType: 'std_msgs/String'
});

var modeTopic = new ROSLIB.Topic({
  ros: portalRos,
  name: '/appctl/mode',
  messageType: 'appctl/Mode'
});

displaySwitchTopic.advertise();
modeTopic.advertise();

var sendMode = function(url) {
  var modeMsg;

  if (url in MODES) {
    modeMsg = new ROSLIB.Message({mode: MODES[url]});
  } else {
    modeMsg = new ROSLIB.Message({mode: MODES['default']});
  }
  console.log('Switching to mode', modeMsg.mode);
  modeTopic.publish(modeMsg);
};

var sendURL = function(url) {
  console.log('Trying to switch display to', url);
  var switchMsg = new ROSLIB.Message({data: url});
  displaySwitchTopic.publish(switchMsg);
};

chrome.runtime.onMessage.addListener(
  function(request) {
    sendMode(request.url);
    sendURL(request.url);
  }
);

