/**
 * A tone generator with any number of oscillators.
 *
 * @example
 * var tone = new ToneGenerator(new AudioContext());
 * tone.setGain(0.2);
 * tone.setFreq(250);
 * tone.start();
 * setTimeout(tone.stop, 1000);
 *
 * @constructor
 * @param {AudioContext} context
 * @see https://developer.mozilla.org/en-US/docs/Web/API/OscillatorNode
 *
 * @author Matt Vollrath <matt@endpoint.com>
 */

function ToneGenerator(context) {

  function isNumber(n) {
    return !isNaN(parseFloat(n)) && isFinite(n);
  }

  var gainNode = context.createGain();
  gainNode.gain.value = 0.2;
  gainNode.connect(context.destination);

  var panNode = context.createPanner();
  //panNode.setOrientation(0.0, 0.0, 1.0);
  panNode.panningModel = 'equalpower';
  panNode.connect(gainNode);

  var oscNode = context.createOscillator();
  oscNode.type = 'sine';
  oscNode.frequency.value = 440;
  oscNode.connect(panNode);

  /**
   * Sets the gain.
   *
   * @param {number} gain - The gain to set.
   */
  function setGain(gain) {
    if (!isNumber(gain)) {
      throw 'Gain must be a number.';
    }
    gainNode.gain.value = gain;
  }

  /**
   * Sets the stereo pan.
   *
   * @param {number} pan - The stereo position [-1, 1]
   */
  function setPan(pan) {
    if (!isNumber(pan)) {
      throw 'Pan must be a number.';
    }
    panNode.setPosition(pan, 0.0, 1.0 - Math.abs(pan));
  }

  /**
   * Sets the frequency for all oscillators.
   *
   * @param {number} freq - The frequency to set.
   */
  function setFreq(freq) {
    if (!isNumber(freq)) {
      throw 'Frequency must be a number.';
    }
    oscNode.frequency.value = freq;
  }

  /**
   * Starts generating audio.
   */
  function start() {
    oscNode.noteOn(0);
  }

  /**
   * Stops generating audio.
   */
  function stop() {
    oscNode.noteOff(0);
  }

  return {
    setGain: setGain,
    setPan: setPan,
    setFreq: setFreq,
    start: start,
    stop: stop
  };
}
