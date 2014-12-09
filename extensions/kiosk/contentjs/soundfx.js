/**
 * Deal with loading and playing audio clips in response to requested movements.
 */

var EARTH_RADIUS = 6371000; // meters from center
var EARTH_ATMOSPHERE_CEILING = 480000; // meters from surface
var ATMOSPHERE_FALLOFF = 6; // exponential falloff rate for atmospheric density
var BOOST_START_LEVEL = 0.5; // play boost when level exceeds this value
var BOOST_END_LEVEL = 0.25; // end boost when level exceeds this value
var BOOST_GAIN = 1.0; // gain level of boost effect
var HOVER_TIMEOUT = 200; // ms, hover fx after no movement for this interval
var HOVER_LEVEL = 0.12; // ambient level
var HUM_GAIN_MIN = 0.04; // minimum hum level
var HUM_GAIN_MAX = 0.3; // maximum hum level
var HUM_GAIN_SCALE = 0.2; // multiply hum gain by this factor
var HUM_PAN_SCALE = 1.0; // scale hum panning by this factor
var HUM_FREQ_MIN = 30; // minimum (idle) hum frequency
var HUM_FREQ_FACTOR = 10; // multiply hum frequency by this factor

/** Shim for audio context.
 * @ignore
*/
window.AudioContext = window.AudioContext || window.webkitAudioContext;

/**
 * Container for single sound effect, able to isolate a section and/or loop.
 * TODO(arshan): Should we support end < begin by looping past the end?
 * TODO(arshan): What is the floor for durationMs?
 * NOTE: see the sox output for the wav clips at the bottom of file.
 * @constructor
 * @param {AudioContext} context
 * @param {string} src_file
 * @param {int} begin
 * @param {int} end
 * @param {bool} loop
 */
SoundEffect = function(context, src_file, begin, end, loop) {

  this.context = context;
  this.loaded = false;

  this.gainNode = this.context.createGain();
  this.panNode = this.context.createPanner();

  // source -> gain -> pan -> destination
  this.gainNode.connect(this.panNode);
  this.panNode.connect(this.context.destination);

  this.gainNode.gain.value = 0;

  this.startMs = begin;
  this.durationMs = end - begin;

  var request = new XMLHttpRequest();
  request.open('GET', src_file, true);
  request.responseType = 'arraybuffer';

  var self = this;
  request.onload = function() {
    self.context.decodeAudioData(request.response, function(buffer) {
      self.buffer = buffer;
      self.loaded = true;
      if (self.loop) {
        self.start();
      }
    },
    function(err) {
      console.error(err);
    });
  };

  request.send();

  this.loop = loop;
  //this.loop = false;
  this.event_thread = 0;
  this.playing = false;

  /*
  // support crossfade
  this.xfade = 0;
  this.xratio = 0;
  this.xincr = 0.05;
  this.xstep = 100;
  this.crossfading = false;
  // Keep track of the event thread for the cross fading.
  this.xthread = 0;
  */
};

/**
 * Change the volume of the clip.
 * @param {float} float_val volume level [0, 1]
 */
SoundEffect.prototype.setVolume = function(float_val) {
  float_val = Math.max(0, Math.min(1, float_val));
  this.gainNode.gain.value = float_val;
};

/**
 * Pan the clip.
 * @param {float} panX left/right pan [-1, 1]
 * @param {float} panY up/down pan [-1, 1]
 * @param {float} panZ forward/back pan [-1, 1]
 */
SoundEffect.prototype.setPan = function(panX, panY, panZ) {
  panX = panX ? Math.max(-1, Math.min(1, panX)) : 0;
  panY = panY ? Math.max(-1, Math.min(1, panY)) : 0;
  panZ = panZ ? Math.max(-1, Math.min(1, panZ)) : 0;
  this.panNode.setPosition(panX, panY, panZ);
};

/**
 * Support to crossfade the soundeffect to a different audio clip.
 * Moves from one to the other symetrically over time.
 * @param {SoundEffect} fx The effect to fade to.
 * @param {float} incr How much to slide from one to the other.
 * @param {int} step The duration of time in ms to take per step.
 * @param {int} offset How long to wait in ms before starting.
 */
SoundEffect.prototype.crossfadeTo = function(fx, incr, step, offset) {
  if (typeof offset === 'undefined') offset = 0;

  this.xfade = fx;
  this.xratio = 0;
  this.xincr = incr;
  this.xstep = step;

  fx.stop();
  fx.setVolume(0);
  fx.start();

  // Crossfade maybe janky to any loop that is too short a clip
  // since the clip will be over.
  this.crossfading = true;
  var self = this;
  this.xthread = setTimeout(function() {
    self.xfade_();
  }, offset);
};

/**
 * Internal call for crossfading.
 * @private
 */
SoundEffect.prototype.xfade_ = function() {
  var self = this;
  this.xthread = setTimeout(function() {
    self.xratio += self.xincr;
    self.setVolume(1 - self.xratio);
    self.xfade.setVolume(self.xratio);
    if (self.xratio < 1.0) {
      self.xfade_();
    }
    else {
      // The crossfade is done.
      self.stop();
      self.setVolume(1);
      self.crossfading = false;
    }
  }, this.xstep);
};

/**
 * If a looping effect this will play indefinitely, otherwise it will
 * naturally stop at the end time.
 */
SoundEffect.prototype.start = function() {
  // If clip is looping just let it keep playing.
  // Should this behaviour hold true for non-loop clips?
  if (this.playing) return;
  this.playing = true;
  this.start_();
};

/**
 * Internal call for running the clip.
 * @private
 */
SoundEffect.prototype.start_ = function() {
  /*
  if (this.audio.readyState == 0) return; // Consider an error
  this.audio.currentTime = this.startMs / 1000;
  var self = this;
  this.event_thread = setTimeout(function() {
    if (!self.loop) {
      self.stop();
    }
  }, this.durationMs);
  */
  this.source = this.context.createBufferSource();
  this.source.buffer = this.buffer;
  this.source.connect(this.gainNode);
  this.source.loop = this.loop;
  this.source.start(0);
  var self = this;
  this.source.onended = function() {
    self.playing = false;
  }
};

/**
 * Premptive stop of the audio clip.
 */
SoundEffect.prototype.stop = function() {
  //this.audio.pause();
  this.source.stop(); // web audio api can only start() once!
  this.playing = false;
  //this.crossfading = false;
  // NOTE: might have to stop the crossfade target too?
  //clearTimeout(this.event_thread);
};

/**
 * Sound FX control module.
 * @constructor
 */
SoundFX = function() {
  this.context = new AudioContext();
  this.lastPose = null;
  this.lastUpdateTime = 0;
  this.lastSeq = 0;
  this.hoverTimer = null;
  this.boosting = false;
  this.enabled = true;

  this.hum = new ToneGenerator(this.context);
  this.hum.setFreq(HUM_FREQ_MIN);
  this.hum.start();

  // Preload the sound clips.
  // TODO(arshan): Better to load these out of a config file?
  this.largestart = new SoundEffect(
    this.context,
    chrome.extension.getURL('sounds/largestart.wav'),
                            0, 1217, false);
  this.largestart.setVolume(BOOST_GAIN);

  this.largeidle = new SoundEffect(
    this.context,
    chrome.extension.getURL('sounds/largeidle.wav'),
                            0, 27282, true);

  this.smallstart = new SoundEffect(
    this.context,
    chrome.extension.getURL('sounds/smallstart.wav'),
                            0, 2000, false);
  this.smallstart.setVolume(BOOST_GAIN);

  this.smallidle = new SoundEffect(
    this.context,
    chrome.extension.getURL('sounds/smallidle.wav'),
                             0, 14003, true);

  this.cutoff = new SoundEffect(
    this.context,
    chrome.extension.getURL('sounds/cutoff.wav'),
                             0, 1994, false);

};

/**
 * Respond to the incoming pose message, with sound if appropriate.
 * @param {Object} stampedPose The portal stamped pose msg object.
 */
SoundFX.prototype.handlePoseChange = function(stampedPose) {

  // skip duplicates
  // TODO(mv): figure out why there are duplicates
  var seq = stampedPose.header.seq;
  if (seq == this.lastSeq)
    return;
  this.lastSeq = seq;

  var now = stampedPose.header.stamp.secs +
            stampedPose.header.stamp.nsecs / 1000000000;
  var dt = now - this.lastUpdateTime;
  this.lastUpdateTime = now;

  if (!this.enabled) return;

  function toRadians(n) {
    return n * Math.PI / 180.0;
  }

  var pose = stampedPose.pose;
  var lastPose = this.lastPose || stampedPose.pose;
  var distanceToEarthCenter = pose.position.z + EARTH_RADIUS;

  // convert to radians for trig functions
  var lat = toRadians(pose.position.y);
  var lastLat = toRadians(lastPose.position.y);
  var dLat = toRadians(pose.position.y - lastPose.position.y);
  var dLng = toRadians(pose.position.x - lastPose.position.x);

  // altitude change in meters
  var dAlt = pose.position.z - lastPose.position.z;
  // greater thrust required to move "up"
  dAlt *= (dAlt > 0) ? 1.5 : 0.5;

  this.lastPose = pose;

  // equirectangular approximation -- performance > accuracy
  var x = dLng * Math.cos((lat + lastLat) / 2);
  var y = dLat;
  var dLateral = Math.sqrt(x * x + y * y) * distanceToEarthCenter;

  var speed = (dLateral + Math.abs(dAlt)) / dt; // m/s, theoretically
  var val = Math.sqrt(speed / 100000) * 5;

  // atmospheric coefficient
  var atmosphereCoeff = 0;
  if (pose.position.z < EARTH_ATMOSPHERE_CEILING) {
    // TODO(mv): reduce O
    var linearAtmosphere = Math.abs(pose.position.z - EARTH_ATMOSPHERE_CEILING);
    atmosphereCoeff = Math.pow(linearAtmosphere, ATMOSPHERE_FALLOFF) /
      Math.pow(EARTH_ATMOSPHERE_CEILING, ATMOSPHERE_FALLOFF);
  }
  val *= atmosphereCoeff;

  // panning away from movement vectors
  var tiltular = toRadians(pose.orientation.x) + Math.atan2(dLateral, dAlt);
  var lateral = toRadians(pose.orientation.z) + Math.atan2(dLat, dLng);
  // 3D pan values, poorly supported irl
  //var panX = -Math.cos(lateral);
  //var panZ = Math.sin(lateral) * Math.cos(tiltular);
  //var panY = Math.sin(lateral) * Math.cos(tiltular);

  // simple stereo pan
  var stereoPan = -Math.cos(lateral) * Math.sin(tiltular);

  // Now play the corresponding sound effects.
  this.update(val, stereoPan, 1.0, 0);
};

/**
 * Handles incoming SpaceNav updates.
 * @param {object} twist
 */
SoundFX.prototype.handleNavTwist = function(twist) {
  if (!this.enabled) {
    this.hum.setGain(0);
    return;
  }

  var x = twist.linear.x;
  var y = twist.linear.y;
  var z = twist.linear.z;
  var val = Math.sqrt(z * z + Math.sqrt(x * x + y * y));

  var humFreq = HUM_FREQ_MIN + val * HUM_FREQ_FACTOR;
  var humPan = x * HUM_PAN_SCALE;
  var humGain = Math.min(HUM_GAIN_MIN + val * (HUM_GAIN_MAX - HUM_GAIN_MIN), HUM_GAIN_MAX) * HUM_GAIN_SCALE;

  this.hum.setFreq(humFreq);
  this.hum.setPan(humPan);
  this.hum.setGain(humGain);
};

/**
 * Plays appropriate sound effects for the incoming speed.
 * @param {float} level The amount of air friction [0, 1]
 * @param {float} panX Side to side panning component [-1, 1]
 * @param {float} panY Up to down panning component [-1, 1]
 * @param {float} panZ Forward to back panning component [-1, 1]
 */
SoundFX.prototype.update = function(level, panX, panY, panZ) {
  this.largeidle.setVolume(level);
  this.largeidle.setPan(panX, panY, panZ);

  if (level > BOOST_START_LEVEL) {
    if (!this.boosting) {
      this.boosting = true;
      this.largestart.start();
    }
  } else if (this.boosting && level < BOOST_END_LEVEL) {
    this.boosting = false;
    this.largestart.stop();
  }

  if (level > HOVER_LEVEL) {
    clearTimeout(this.hoverTimer);
    var self = this;
    this.hoverTimer = setTimeout(function() {
      self.hover();
    }, HOVER_TIMEOUT);

  } else {
    clearTimeout(this.hoverTimer);
    this.hover();
  }
};

/**
 * Sets hover gain.
 */
SoundFX.prototype.hover = function() {
  if (!this.enabled) {
    return;
  }

  // only play hover sound within the atmosphere
  if (this.lastPose.position.z < EARTH_ATMOSPHERE_CEILING) {
    this.largeidle.setVolume(HOVER_LEVEL);
    this.largeidle.setPan(0, 1.0, 0);
  } else {
    this.largeidle.setVolume(0);
  }
};

/**
 * Enables sound updates from camera motion.
 */
SoundFX.prototype.enable = function() {
  this.enabled = true;
};

/**
 * Disables sound updates from camera motion.
 */
SoundFX.prototype.disable = function() {
  this.enabled = false;
  this.hum.setGain(0);
  this.largeidle.setVolume(0);
};

/*
Just for informations sake, and possibly fingerprinting, the sox output
for the clips used in the sound effects.

>> sox largeidle.wav -n stat
Samples read:           2619096
Length (seconds):     27.282250
Scaled by:         2147483647.0
Maximum amplitude:     0.474805
Minimum amplitude:    -0.538968
Midline amplitude:    -0.032082
Mean    norm:          0.070921
Mean    amplitude:     0.000013
RMS     amplitude:     0.090059
Maximum delta:         0.114961
Minimum delta:         0.000000
Mean    delta:         0.019609
RMS     delta:         0.024728
Rough   frequency:         2097
Volume adjustment:        1.855

>> sox largestart.wav -n stat
Samples read:            116920
Length (seconds):      1.217917
Scaled by:         2147483647.0
Maximum amplitude:     1.000000
Minimum amplitude:    -0.833501
Midline amplitude:     0.083250
Mean    norm:          0.132635
Mean    amplitude:    -0.002388
RMS     amplitude:     0.188895
Maximum delta:         0.460690
Minimum delta:         0.000000
Mean    delta:         0.026345
RMS     delta:         0.040748
Rough   frequency:         1647
Volume adjustment:        1.000

>> sox smallidle.wav -n stat
Samples read:           1344348
Length (seconds):     14.003625
Scaled by:         2147483647.0
Maximum amplitude:     0.051898
Minimum amplitude:    -0.041315
Midline amplitude:     0.005291
Mean    norm:          0.010027
Mean    amplitude:     0.000019
RMS     amplitude:     0.012588
Maximum delta:         0.017117
Minimum delta:         0.000000
Mean    delta:         0.002672
RMS     delta:         0.003365
Rough   frequency:         2041
Volume adjustment:       19.269

>> sox smallstart.wav -n stat
Samples read:            192000
Length (seconds):      2.000000
Scaled by:         2147483647.0
Maximum amplitude:     0.399003
Minimum amplitude:    -0.308831
Midline amplitude:     0.045086
Mean    norm:          0.032232
Mean    amplitude:    -0.000193
RMS     amplitude:     0.052041
Maximum delta:         0.102391
Minimum delta:         0.000000
Mean    delta:         0.006791
RMS     delta:         0.011959
Rough   frequency:         1755
Volume adjustment:        2.506

>>  sox cutoff.wav -n stat
Samples read:            191424
Length (seconds):      1.994000
Scaled by:         2147483647.0
Maximum amplitude:     0.159753
Minimum amplitude:    -0.160580
Midline amplitude:    -0.000413
Mean    norm:          0.023638
Mean    amplitude:    -0.000226
RMS     amplitude:     0.033527
Maximum delta:         0.133299
Minimum delta:         0.000000
Mean    delta:         0.017119
RMS     delta:         0.024671
Rough   frequency:         5621
Volume adjustment:        6.227

*/
