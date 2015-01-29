/**
 * Fan speed controller for wind simulation.
 * @param {Object} ros
 *       A shared ROS connection.
 * @author Matt Vollrath <matt@endpoint.com>
 */
var FanControl = function(ros) {
  this.POWER_STATES = {
    OFF: 0,
    PULSE: 1,
    ON: 2
  };
  this.PULSE_RATE = 2; // Hz

  /** The last pose received. */
  this.lastPose = null;
  /** @type {Boolean} */
  this.enabled = true;

  this.leftFanSpeedTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/fancontrol/left/speed',
    type: 'std_msgs/Float32'
  });
  this.leftFanSpeedTopic.advertise();

  this.rightFanSpeedTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/fancontrol/right/speed',
    type: 'std_msgs/Float32'
  });
  this.rightFanSpeedTopic.advertise();

  this.leftFanPowerTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/weather_pdu/outlet/1',
    type: 'std_msgs/Bool',
    queue_size: 5
  });
  this.leftFanPowerTopic.advertise();
  this.leftFanPowerState = this.POWER_STATES.OFF;
  this.leftFanPulseTimer = null;
  this.leftFanPulseState = false;

  /*
  this.rightFanPowerTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/fan_pdu/outlet/2',
    type: 'std_msgs/Bool',
    queue_size: 5
  });
  this.rightFanPowerTopic.advertise();
  this.rightFanPowerState = this.POWER_STATES.OFF;
  this.rightFanPulseTimer = null;
  this.rightFanPulseState = false;
  */
}

FanControl.prototype.setFanPower_ = function(leftSpeed, rightSpeed) {
  var self = this;
  leftSpeed = (leftSpeed + rightSpeed) / 2;

  // TODO(mv): refactor this garbage
  /*
  if (this.leftFanPowerState != this.POWER_STATES.PULSE && leftSpeed > 0.2 && leftSpeed <= 0.5) {
    this.leftFanPowerState = this.POWER_STATES.PULSE;
    this.leftFanPulseState = true;
    this.leftFanPowerTopic.publish(new ROSLIB.Message({data: true}));
    this.leftFanPulseTimer = setInterval(function () {
      self.leftFanPulseState = !self.leftFanPulseState;
      self.leftFanPowerTopic.publish(new ROSLIB.Message({data: self.leftFanPulseState}));
    }, 1000 / this.PULSE_RATE);
  */
  if (this.leftFanPowerState != this.POWER_STATES.ON && leftSpeed > 0.2) {
    this.leftFanPowerState = this.POWER_STATES.ON;
    clearInterval(this.leftFanPulseTimer);
    this.leftFanPowerTopic.publish(new ROSLIB.Message({data: true}));

  } else if (this.leftFanPowerState != this.POWER_STATES.OFF && leftSpeed <= 0.2) {
    this.leftFanPowerState = this.POWER_STATES.OFF;
    clearInterval(this.leftFanPulseTimer);
    this.leftFanPowerTopic.publish(new ROSLIB.Message({data: false}));

  }

  // TODO(mv): refactor this garbage
  /*
  if (this.rightFanPowerState != this.POWER_STATES.PULSE && rightSpeed > 0.33 && rightSpeed <= 0.66) {
    this.rightFanPowerState = this.POWER_STATES.PULSE;
    this.rightFanPulseState = true;
    this.rightFanPowerTopic.publish(new ROSLIB.Message({data: true}));
    this.rightFanPulseTimer = setInterval(function () {
      self.rightFanPulseState = !self.rightFanPulseState;
      self.rightFanPowerTopic.publish(new ROSLIB.Message({data: self.rightFanPulseState}));
    }, 1000 / this.PULSE_RATE);

  } else if (this.rightFanPowerState != this.POWER_STATES.ON && rightSpeed > 0.66) {
    this.rightFanPowerState = this.POWER_STATES.ON;
    clearInterval(this.rightFanPulseTimer);
    this.rightFanPowerTopic.publish(new ROSLIB.Message({data: true}));

  } else if (this.rightFanPowerState != this.POWER_STATES.OFF && rightSpeed <= 0.33) {
    this.rightFanPowerState = this.POWER_STATES.OFF;
    clearInterval(this.rightFanPulseTimer);
    this.rightFanPowerTopic.publish(new ROSLIB.Message({data: false}));

  }
  */
};

/**
 * Sets the speed of each fan.
 * @param {Number} leftSpeed
 *       Speed for the left fan [0, 1]
 * @param {Number} rightSpeed
 *       Speed for the right fan [0, 1]
 */
FanControl.prototype.setFanSpeed_ = function(leftSpeed, rightSpeed) {
  var leftMsg = new ROSLIB.Message({ data: leftSpeed });
  var rightMsg = new ROSLIB.Message({ data: rightSpeed });
  this.leftFanSpeedTopic.publish(leftMsg);
  this.rightFanSpeedTopic.publish(rightMsg);

  /*
  if (leftSpeed > 0.5) {
    this.leftFanPowerTopic.publish(new ROSLIB.Message({
      data: true
    }));
  } else {
    this.leftFanPowerTopic.publish(new ROSLIB.Message({
      data: false
    }));
  }

  if (rightSpeed > 0.5) {
    this.rightFanPowerTopic.publish(new ROSLIB.Message({
      data: true
    }));
  } else {
    this.rightFanPowerTopic.publish(new ROSLIB.Message({
      data: false
    }));
  }
  */
};

/**
 * Respond to the incoming pose message, with fans if appropriate.
 * @param {Object} stampedPose The portal stamped pose msg object.
 */
FanControl.prototype.handlePoseChange = function(stampedPose) {
  this.lastPose = stampedPose.pose;
};

/**
 * Handles an incoming SpaceNav update.
 * @param {Object} twist
 */
FanControl.prototype.handleNavTwist = function(twist) {
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

  var atmosphere = SoundFX.prototype.getAtmosphereCoeff(
    altitude,
    FLYING_CEILING_MOD
  );
  var val = valPrime * atmosphere;
  var pan = ((x + 1) / 2) * (Math.PI / 2);
  //var forward = 1.0 - (Math.max(-y, 0) / 2);
  var forward = 1;
  var leftSpeed = val * Math.cos(pan) * forward;
  var rightSpeed = val * Math.sin(pan) * forward;
  /*
  console.log(
    'forward:', forward,
    'left fan:', leftSpeed,
    'right fan:', rightSpeed
  );
  */
  if (!hovering) {
    this.setFanSpeed_(leftSpeed, rightSpeed);
    this.setFanPower_(leftSpeed, rightSpeed);
  } else {
    this.setFanSpeed_(0, 0);
    this.setFanPower_(0, 0);
  }
};

/**
 * Enables fan speed updates from camera motion.
 */
FanControl.prototype.enable = function() {
  this.enabled = true;
};

/**
 * Disables fan speed updates from camera motion.
 */
FanControl.prototype.disable = function() {
  this.enabled = false;
  this.setFanSpeed_(0, 0);
};
