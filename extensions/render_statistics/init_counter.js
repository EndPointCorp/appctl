/**
 * Frame rate counter based on requestAnimationFrame.
 *
 * Uses StatsD helpers to report results back to the extension.
 *
 * @namespace
 */
var RenderStatistics = RenderStatistics || (function() {
  /**
   * The RenderStatistics instance.
   * @private
   * @static
   */
  var instance;

  function init() {
    /**
     * Minimum allowed frame rendering time in milliseconds.
     *
     * This prevents false 0-1ms frame counts from tactile.
     * @private
     * @instance
     * @memberof RenderStatistics
     */
    var MIN_FRAME_TIME = 8;

    /**
     * Keeps track of the running state of the frame rate counter.
     *
     * Intended to prevent redundant scheduling by making start() idempotent.
     * @private
     * @instance
     * @memberof RenderStatistics
     */
    var running = false;

    /**
     * The time of the last frame.
     * @private
     * @instance
     * @memberof RenderStatistics
     */
    var t = Date.now();

    /**
     * How many frames have been rendered since the last frameRateCounter().
     * @private
     * @instance
     * @memberof RenderStatistics
     */
    var frameCount = 0;

    /**
     * The running requestAnimationFrame() handle.
     * @private
     * @instance
     * @memberof RenderStatistics
     */
    var frameRequest = null;

    /**
     * The running frameRateCounter() timer handle.
     * @private
     * @instance
     * @memberof RenderStatistics
     */
    var frameRateTimer = null;

    /**
     * StatsD helper for frame count.
     * @private
     * @instance
     * @memberof RenderStatistics
     */
    var frameCountSink = new StatsD.Counter('render.frames');

    /**
     * StatsD helper for frame render time.
     * @private
     * @instance
     * @memberof RenderStatistics
     */
    var frameTimeSink = new StatsD.Timing('render.time');

    /**
     * Handles a freshly drawn frame.
     * @private
     * @memberof RenderStatistics
     */
    function frameHandler() {
      var elapsed = Date.now() - t;
      if (elapsed > MIN_FRAME_TIME) {
        frameCount++;
        frameTimeSink.update(elapsed);
      }
      scheduleFrameHandler();
      t = Date.now();
    }

    /**
     * Schedules a request for the next frame.
     * @private
     * @memberof RenderStatistics
     */
    function scheduleFrameHandler() {
      frameRequest = requestAnimationFrame(frameHandler);
    }

    /**
     * Counts frames since the last frameRateCounter(), sends them to the sink.
     * @private
     * @memberof RenderStatistics
     */
    function frameRateCounter() {
      console.debug(frameCount, 'fps');
      frameCountSink.update(frameCount);
      frameCount = 0;
    }

    /**
     * Start collecting statistics.
     * @public
     * @memberof RenderStatistics
     */
    function start() {
      if (running) return;

      console.debug('RenderStatistics.start()');
      running = true;
      frameCount = 0;
      scheduleFrameHandler();
      frameRateTimer = setInterval(frameRateCounter, 1000);
      t = Date.now();
    }

    /**
     * Stop collecting statistics.
     * @public
     * @memberof RenderStatistics
     */
    function stop() {
      console.debug('RenderStatistics.stop()');
      running = false;
      cancelAnimationFrame(frameRequest);
      clearInterval(frameRateTimer);
    }

    return {
      start: start,
      stop: stop
    };
  }

  return {
    getInstance: function() {
      if (!instance) {
        instance = init();
      }

      return instance;
    }
  };
})();
