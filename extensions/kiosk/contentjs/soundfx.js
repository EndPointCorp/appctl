/**
 * Deal with loading and playing audio clips in response to requested movements.
 */

var EARTH_RADIUS = 6371000; // meters from center
var EARTH_ATMOSPHERE_CEILING = 480000; // meters from surface
var ATMOSPHERE_FALLOFF = 16; // exponential falloff rate for atmospheric density
var FLYING_GAIN_SCALE = 1.0;
var FLYING_PAN_SCALE = 1.0;
var FLYING_CEILING_MOD = 1.0;
var FLYING_HOVER_LEVEL = 0.04;
var SUBFLYING_GAIN_SCALE = 0.6;
var SUBFLYING_PAN_SCALE = 0.75;
var SUBFLYING_CEILING_MOD = 4.0;
var SUBFLYING_HOVER_LEVEL = 0.08;
var BOOST_START_SPEED = 200000; // play boost when speed exceeds this value
var BOOST_END_SPEED = 20000; // end boost when speed is below this value
var BOOST_GAIN = 0.0; // gain level of boost effect
var HOVER_DEADZONE = 0.02; // hovering when stick inside this zone

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
  this.panNode.setPosition(0, 0, 0);

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
  this.event_thread = 0;
  this.playing = false;
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
 * @param {float} panX left/right
 * @param {float} panY up/down
 * @param {float} panZ forward/back
 */
SoundEffect.prototype.setPan = function(panX, panY, panZ) {
  this.panNode.setPosition(panX, panY, panZ);
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
  this.source.stop(); // web audio api can only start() once!
  this.playing = false;
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

  /*
  this.boost = new SoundEffect(
    this.context,
    chrome.extension.getURL('sounds/boost.wav'),
                            0, 2435, false);
  this.boost.setVolume(BOOST_GAIN);
  */
  this.boost = this.largestart;

  this.flying = new SoundEffect(
    this.context,
    chrome.extension.getURL('sounds/flying.wav'),
                            0, 27282, true);
  this.subflying = new SoundEffect(
    this.context,
    chrome.extension.getURL('sounds/subflying.wav'),
                            0, 6500, true);

};

/**
 * Get the atmospheric density at the given altitude.
 * @param {Number} altitude
 * @param {Number} ceilingMod Modify ceiling by this coefficient.
 */
SoundFX.prototype.getAtmosphereCoeff = function(altitude, ceilingMod) {
  ceilingMod = ceilingMod || 1.0;
  var ceiling = EARTH_ATMOSPHERE_CEILING * ceilingMod;

  var atmosphereCoeff = 0;
  if (altitude < ceiling) {
    var linearAtmosphere = Math.abs(altitude - ceiling);
    atmosphereCoeff = Math.pow(linearAtmosphere, ATMOSPHERE_FALLOFF) /
      Math.pow(ceiling, ATMOSPHERE_FALLOFF);
  }

  return atmosphereCoeff;
}

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

  /*
  // equirectangular approximation -- performance > accuracy
  var x = dLng * Math.cos((lat + lastLat) / 2);
  var y = dLat;
  var dLateral = Math.sqrt(x * x + y * y) * distanceToEarthCenter;

  var speed = (dLateral + Math.abs(dAlt)) / dt; // m/s, theoretically
  var altitude = pose.position.z;

  if (speed > BOOST_START_SPEED && altitude < EARTH_ATMOSPHERE_CEILING) {
    if (!this.boosting) {
      this.boosting = true;
      //this.boost.setVolume(BOOST_GAIN);
      this.boost.start();
    }
  } else if (this.boosting && speed < BOOST_END_SPEED) {
    this.boosting = false;
    //this.boost.setVolume(0);
    this.boost.stop();
  }
  */
};

/**
 * Handles incoming SpaceNav updates.
 * @param {object} twist
 */
SoundFX.prototype.handleNavTwist = function(twist) {
  if (!this.lastPose || !this.enabled) {
    return;
  }

  var x = twist.linear.x;
  var y = twist.linear.y;
  var z = twist.linear.z;

  var hovering = (
    Math.abs(x) < HOVER_DEADZONE &&
    Math.abs(y) < HOVER_DEADZONE &&
    Math.abs(z) < HOVER_DEADZONE
  );

  var altitude = this.lastPose.position.z;
  var valPrime = Math.sqrt(z * z + Math.sqrt(x * x + y * y));

  var atmosphere = this.getAtmosphereCoeff(altitude, FLYING_CEILING_MOD);
  var val = valPrime * atmosphere;
  if (!hovering) {
    this.flying.setVolume(val * FLYING_GAIN_SCALE);
    this.flying.setPan(-x * FLYING_PAN_SCALE, 0, 1.0 - Math.abs(x));
  } else {
    this.flying.setVolume(FLYING_HOVER_LEVEL * FLYING_GAIN_SCALE * atmosphere);
  }

  var subAtmosphere = this.getAtmosphereCoeff(altitude, SUBFLYING_CEILING_MOD);
  var subVal = valPrime * subAtmosphere;
  if (!hovering) {
    this.subflying.setVolume(subVal * SUBFLYING_GAIN_SCALE);
    this.subflying.setPan(-x * SUBFLYING_PAN_SCALE, 0, 1.0 - Math.abs(x));
  } else {
    this.subflying.setVolume(SUBFLYING_HOVER_LEVEL * SUBFLYING_GAIN_SCALE * subAtmosphere);
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
  this.flying.setVolume(0);
  this.subflying.setVolume(0);
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

>> sox flying.wav -n stat
Samples read:           2406294
Length (seconds):     27.282245
Scaled by:         2147483647.0
Maximum amplitude:     0.431601
Minimum amplitude:    -0.390338
Midline amplitude:     0.020631
Mean    norm:          0.062867
Mean    amplitude:    -0.000000
RMS     amplitude:     0.080379
Maximum delta:         0.109544
Minimum delta:         0.000000
Mean    delta:         0.018213
RMS     delta:         0.022940
Rough   frequency:         2003
Volume adjustment:        2.317

>> sox subflying.wav -n stat
Samples read:            573300
Length (seconds):      6.500000
Scaled by:         2147483647.0
Maximum amplitude:     0.977160
Minimum amplitude:    -0.869215
Midline amplitude:     0.053973
Mean    norm:          0.168438
Mean    amplitude:     0.000428
RMS     amplitude:     0.210275
Maximum delta:         0.495182
Minimum delta:         0.000000
Mean    delta:         0.087508
RMS     delta:         0.110032
Rough   frequency:         3672
Volume adjustment:        1.023

*/
