var MIN_FREQ = 30;

var toneGen = new ToneGenerator('sine');
toneGen.setGain(0);

function yToGain(y) {
  return Math.max(0, (500 - y) / 100);
}

function zToFreq(z) {
  return Math.max(MIN_FREQ, MIN_FREQ + (350 - z) / 8);
}

function handleFrame(frameInstance) {
  // TODO: use interactionBox instead of hard-coding absolutes
  var hands = frameInstance.hands;
  var numHands = hands.length;

  if (numHands <= 1) {
    toneGen.setGain(0);
    return;
  }

  // TODO: handle more than two hands
  var leftHand = hands[0];
  var rightHand = hands[1];

  if (rightHand.palm_position.x < leftHand.palm_position.x) {
    leftHand = hands[1];
    rightHand = hands[0];
  }

  var gain = yToGain(leftHand.palm_position.y);
  var freq = zToFreq(rightHand.palm_position.z);

  toneGen.setGain(gain);
  toneGen.setFreq(freq);
}

chrome.storage.sync.get({
  wsUrl: 'ws://localhost:9090',
  rate: 60,
  socketDebug: false
}, function(items) {
  var wsUrl = items.wsUrl;
  var rate = items.rate;
  var socketDebug = items.socketDebug;

  var socket = new WebSocket(wsUrl);

  // subscribe to rosbridge topic
  // https://github.com/RobotWebTools/rosbridge_suite/blob/groovy-devel/ROSBRIDGE_PROTOCOL.md

  socket.onopen = function(ev) {
    console.log('socket connected');
    socket.send(JSON.stringify({
      op: 'subscribe',
      topic: '/leap_motion/frame',
      type: 'leap_motion/Frame',
      throttle_rate: Math.round(1000 / rate)
    }));
    toneGen.start();
  };

  socket.onmessage = function(ev) {
    var data = JSON.parse(ev.data);

    if (socketDebug) console.log(data);

    handleFrame(data.msg);
  };
});

