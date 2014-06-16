
/**
 * Deal with loading and playing audio clips in response to requested movements.
 */

/**
 * Container for single sound effect, able to isolate a section and/or loop.
 * TODO(arshan): Should we support end < begin by looping past the end?
 * TODO(arshan): What is the floor for duration_ms?
 * NOTE: see the sox output for the wav clips at the bottom of file.
 * @constructor
 */
SoundEffect = function(src_file, begin, end, loop) {

    this.start_ms = begin;
    this.duration_ms = end - begin;

    // Prepare the audio resource.
    this.audio = new Audio(src_file);
    this.audio.preload = "auto";
    this.loop = loop;
    // this.audio.volume = 1.0;
    this.event_thread = 0;
    this.playing = false;

    // support crossfade
    this.xfade = 0;
    this.xratio = 0;
    this.xincr  = 0.05;
    this.xstep  = 100;
    this.crossfading = false;
    this.xthread = 0;
};

SoundEffect.prototype.setVolume = function(float_val) {
    float_val = Math.max(0, Math.min(1, float_val)); 
    this.audio.volume = float_val;
};

// support to crossfade the thread to a different audio clip.
SoundEffect.prototype.crossfadeTo = function(fx, incr, step, offset){
    if(typeof(offset)==='undefined') offset = 0; 

    this.xfade = fx;
    this.xratio = 0;
    this.xincr = incr;
    this.xstep = step;

    fx.stop();
    fx.setVolume(0);
    fx.start();
    
    // crossfade maybe janky to any loop that is too short a clip ... 
    // since the clip will be over 
    this.crossfading = true;
    var that = this;
    this.xthread = setTimeout(function() {
	that._xfade();	
    }, offset);
};

SoundEffect.prototype._xfade = function () {
    var that = this;
    this.xthread = setTimeout( function () {
        that.xratio += that.xincr;
        that.setVolume(1-that.xratio);
        that.xfade.setVolume(that.xratio); 	
	if (that.xratio < 1.0) that._xfade();
	else { // the crossfade is done
   	  that.stop();
	  that.setVolume(1);
          that.crossfading = false;
        }
    },  this.xstep);
};


// If a looping effect this will play indefinitely, otherwise it will naturally stop
// at the end time.
SoundEffect.prototype.start = function() {  
  // If clip is looping just let it keep playing.
  // Should this behaviour hold true for non-loop clips?
  if (this.playing) return;
  this.playing = true;
  this._start();
};

SoundEffect.prototype._start = function() {
  this.audio.currentTime = this.start_ms/1000;
  var that = this;
  this.event_thread = setTimeout( function(){
       that.loop? that._start(): that.stop();
  }, this.duration_ms);
  this.audio.play();
};

// Premptive stop of the audio clip.
SoundEffect.prototype.stop = function() {
  this.audio.pause();
  this.playing = false;
  this.crossfading = false;
  // NOTE: might have to stop the crossfade target too?
  clearTimeout(this.event_thread);
};

// TODO: move this to a math utility
// circular buffer that gives average of all existing values 
CircularBuffer = function(size) {
    this.buffer = new Array(size);
    this.size = size;
    this.start = this.end = 0;
    this.sum = 0;
    for (var x = 0; x < size; x++) {
	this.buffer[x] = 0;
    }
    this._incr = function(value) {
	return (value+1) % this.size;
    }
};

// Consider moving this to the ComputedVelocity class ... 
CircularBuffer.prototype.getSum = function() {
    return this.sum;
};

CircularBuffer.prototype.addValue = function(val) {
    // Keep track of the sum of values, consider moving this to computed_v?
    this.sum -= this.buffer[this.end];
    this.buffer[this.end] = val;
    this.sum += val;
    
    this.end = this._incr(this.end);
    
    if (this.start == this.end) {
	this.start = this._incr(this.start);
    }
};

CircularBuffer.prototype.getValue = function() {
  result = this.buffer[this.start];
  this.start = this._incr(this.start);
  return result;
};

CircularBuffer.prototype.length = function() {
  if (this.start < this.end) {
    return this.end - this.start;
  }
  else {
    return (this.size - this.start) + this.end;
  }
};

ComputedVelocity = function(size) {
  this.buffer = new CircularBuffer(size);
};

ComputedVelocity.prototype.addValue = function(val) {
  return this.buffer.addValue(val);  
};

ComputedVelocity.prototype.getValue = function() {
  return this.buffer.getSum()/this.buffer.length();
};

/**
 * @constructor
 */
SoundFX = function() {

  this.lastVelocity = 0;

  this.computed_v = new ComputedVelocity(8);

  // Keep track of the state of the user.
  this.IDLE = 0;
  this.STARTING = 1;
  this.FLYING = 2;
  this.STOPPING = 3;
  this.state = this.IDLE; 

  // Preload the sound clips.
  // TODO(arshan): Better to load these out of a config file?
  this.largestart = new SoundEffect(chrome.extension.getURL('sounds/largestart.wav'), 
				    0, 1217, false);
  this.largeidle = new SoundEffect(chrome.extension.getURL('sounds/largeidle.wav'), 
				   0,  27282, true);
 
  this.smallstart = new SoundEffect(chrome.extension.getURL('sounds/smallstart.wav'), 
				0,  2000, false); 
  this.smallidle = new SoundEffect(chrome.extension.getURL('sounds/smallidle.wav'), 
				0,  14003, true);
  this.cutoff = new SoundEffect(chrome.extension.getURL('sounds/cutoff.wav'), 
				0,  1994, false);

  // Global (always on) sounds
  this.smallidle.start();
};

SoundFX.prototype.handlePoseChange = function(twist) {

    // Check boundary conditions.
    // TODO(arshan): Where can we get the altitude for the ground collision?
    // TODO(arshan): Also need the raw altitude values for change in atmosphere.
    x = twist.linear.x;
    y = twist.linear.y;
    z = twist.linear.z;
    v_mag = Math.sqrt(z*z + Math.sqrt(x*x+y*y));
    
    this.computed_v.addValue(v_mag);
    
    // histerisis if needed ...
    // val = this.computed_v.getValue();
    val = v_mag;

    // dumpUpdateToScreen("avg: " + val + "; [" + v_mag + "] " + x + "x" + y  + "x" + z );

    // Now play the corresponding sound effects.
    // TODO: add timing of state change to debounce.
    switch ( this.state ) {
    case this.IDLE:
	if ( val > .3 ) {
	    this.largestart.start();
	    this.state = this.STARTING;
	}
      break;
    case this.STARTING:
	if ( val > .2 ) {
	    this.largestart.crossfadeTo(this.largeidle, .05, 100, 300);
	    this.state = this.FLYING;
	}
      break;
    case this.FLYING:
        if ( val < .1 ) {

	    this.state = this.STOPPING;
	}
      break;
    case this.STOPPING:
	if (val < .05) {
	    this.largeidle.crossfadeTo(this.cutoff, .05, 100, 0);	    
	    this.state = this.IDLE;
	}
      break;
    }

    if (this.state == this.FLYING && val < .2) {
	this.cutoff.start();
    }

    if (this.state == 1 && val < .01) {
        this.largestart.stop();
	this.largeidle.stop();
	this.state = 0;
    }
    
    // Record values for next round.
    this.lastVelocity = val;

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