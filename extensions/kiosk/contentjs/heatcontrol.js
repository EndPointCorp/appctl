/**
 * Heater control for heat simulation.
 * @param {Object} ros
 *       A shared ROS connection.
 * @author Matt Vollrath <matt@endpoint.com>
 */
var HeatControl = function(ros) {
  /** Maximum altitude (in meters) at which heat can occur. */
  this.CEILING = 150000;
  /** Minimum latitude for heat. */
  this.MIN_LAT = -23;
  /** Maximum latitude for heat. */
  this.MAX_LAT = 23;

  this.on_ = false;

  this.heaterTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/fan_pdu/outlet/2',
    type: 'std_msgs/Bool'
  });
  this.heaterTopic.advertise();
};

/**
 * Sets the heater outlet state.
 * @param {Boolean} state
 *       Desired outlet power state.
 */
HeatControl.prototype.setOutletState_ = function(state) {
  var outletMsg = new ROSLIB.Message({
    data: state ? true : false
  });
  this.heaterTopic.publish(outletMsg);
};

/**
 * Turns on the heater, if it is not on already.
 */
HeatControl.prototype.turnOn_ = function() {
  if (!this.on_) {
    this.on_ = true;
    this.setOutletState_(true);
  }
};

/**
 * Turns off the heater, if it is not already off.
 */
HeatControl.prototype.turnOff_ = function() {
  if (this.on_) {
    this.on_ = false;
    this.setOutletState_(false);
  }
};

/**
 * Respond to an incoming pose message, with heat if appropriate.
 * @param {Object} stampedPose The portal stamped pose msg object.
 */
HeatControl.prototype.handlePoseChange = function(stampedPose) {
  var pose = stampedPose.pose;

  var altitude = pose.position.z;
  var latitude = pose.position.y;

  if (altitude < this.CEILING &&
      latitude > this.MIN_LAT &&
      latitude < this.MAX_LAT) {

    this.turnOn_();
  } else {

    this.turnOff_();
  }
};
