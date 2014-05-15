/**
 * A tone generator with any number of oscillators.
 *
 * @example
 * var tone = new ToneGenerator("sine", "sawtooth");
 * tone.setGain(0.2);
 * tone.setFreq(250);
 * tone.start();
 * setTimeout(tone.stop, 1000);
 *
 * @param {...string} type - Oscillator type supported by Web Audio API.
 * @see {@link https://developer.mozilla.org/en-US/docs/Web/API/OscillatorNode}
 *
 * @author Matt Vollrath <matt@endpoint.com>
 */

function ToneGenerator() {
  var context;

  try {
    window.AudioContext = window.AudioContext || window.webkitAudioContext;
    context = new AudioContext();
  }
  catch(e) {
    console.error('Web Audio API is not supported in this browser.');
    throw e;
  }

  var numOscillators = arguments.length;
  var oscillators = [];

  var gainNode = context.createGain();
  gainNode.gain.value = 0.2;
  gainNode.connect(context.destination);

  var mixNode = context.createChannelMerger(numOscillators);
  mixNode.connect(gainNode);

  function isString(s) {
   return typeof s == 'string' || s instanceof String;
  }

  function isNumber(n) {
    return !isNaN(parseFloat(n)) && isFinite(n);
  }

  for (var i = 0; i < numOscillators; i++) {
    var oscNode = context.createOscillator();
    var oscType = arguments[i];
    if (!isString(oscType)) {
      throw "Oscillator type must be a string."
    }
    oscNode.type = oscType;
    oscNode.frequency.value = 440;
    oscNode.connect(mixNode);

    oscillators.push(oscNode);
  }

  /**
   * Sets the gain.
   *
   * @param {number} gain - The gain to set.
   */
  function setGain(gain) {
    if (!isNumber(gain)) {
      throw "Gain must be a number.";
    }
    gainNode.gain.value = gain;
  }

  /**
   * Sets the frequency for all oscillators.
   *
   * @param {number} freq - The frequency to set.
   */
  function setFreq(freq) {
    if (!isNumber(freq)) {
      throw "Frequency must be a number.";
    }
    for (var i = 0; i < numOscillators; i++) {
      oscillators[i].frequency.value = freq;
    }
  }

  /**
   * Starts generating audio.
   */
  function start() {
    for (var i = 0; i < numOscillators; i++) {
      oscillators[i].noteOn(0);
    }
  }

  /**
   * Stops generating audio.
   */
  function stop() {
    for (var i = 0; i < numOscillators; i++) {
      oscillators[i].noteOff(0);
    }
  }

  return {
    setGain: setGain,
    setFreq: setFreq,
    start: start,
    stop: stop
  }
}
