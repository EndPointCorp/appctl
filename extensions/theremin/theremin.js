// lower quality, higher performance settings for Audiolet.
// the library wasn't made for this sort of tone generation!
var AUDIO_SAMPLE_RATE = 4096;
var AUDIO_CHANNELS = 1;
var AUDIO_BUFFER_SIZE = 2048;

var MIN_FREQ = 30;

var Synth = function(audiolet, frequency) {
  AudioletGroup.apply(this, [audiolet, 0, 1]);
  this.wave = new Sine(this.audiolet, frequency);
  this.modulator = new Square(this.audiolet, 2 * frequency);
  this.modulatorMulAdd = new MulAdd(this.audiolet, frequency / 2, frequency);

  this.modulator.connect(this.modulatorMulAdd);
  this.modulatorMulAdd.connect(this.wave);
  this.gain = new Gain(this.audiolet);

  this.wave.connect(this.gain);
  this.gain.connect(this.outputs[0]);

  this.setGain = function(amp) {
    this.gain.gain.setValue(amp);
  }

  this.setFreq = function(freq) {
    this.wave.frequency.setValue(freq);
    this.modulator.frequency.setValue(freq*2);
    this.modulatorMulAdd.mul.setValue(freq/2);
    this.modulatorMulAdd.add.setValue(freq);
  }

  this.setGain(0);
};
extend(Synth, AudioletGroup);

var AudioletApp = function() {
  this.audiolet = new Audiolet(
    AUDIO_SAMPLE_RATE,
    AUDIO_CHANNELS,
    AUDIO_BUFFER_SIZE
  );
  this.synth = new Synth(this.audiolet, MIN_FREQ);
  this.synth.connect(this.audiolet.output);
};
var audio = new AudioletApp();

function handleFrame(frameInstance) {
  // TODO: use interactionBox instead of hard-coding absolutes
  var hands = frameInstance.hands;
  var numHands = hands.length;

  if (numHands <= 1) {
    audio.synth.setGain(0);
    return;
  }

  // TODO: handle more than two hands
  var leftHand = hands[0];
  var rightHand = hands[1];

  if (rightHand.palm_position.x < leftHand.palm_position.x) {
    leftHand = hands[1];
    rightHand = hands[0];
  }

  var gain = Math.max(0, (500 - leftHand.palm_position.y) / 100);
  var freq = Math.max(MIN_FREQ, MIN_FREQ + (350 - rightHand.palm_position.z) / 8);

  audio.synth.setGain(gain);
  audio.synth.setFreq(freq);
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
      throttle_rate: Math.round(1000/rate)
    }));
  };

  socket.onmessage = function(ev) {
    var data = JSON.parse(ev.data);

    if (socketDebug) console.log(data);

    handleFrame(data.msg);
  };
});

