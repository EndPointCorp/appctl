/**
 * Weather machine.
 * @param {Object} ros
 *       A shared ROS connection.
 * @author Matt Vollrath <matt@endpoint.com>
 */
var WeatherMachine = function(ros) {
  /** Maximum altitude (in meters) at which weather can occur. */
  this.CEILING = 120000;

  this.HOT_TEMPERATURE = 21.5;

  this.lastWeatherRequest_ = Date.now();
  this.weatherRequestInterval_ = 12000; // ms
  this.heatOn = false;

  /*
  this.fanTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/fan_pdu/outlet/1',
    type: 'std_msgs/Bool'
  });
  this.fanTopic.advertise();
  */

  this.heaterTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/fan_pdu/outlet/2',
    type: 'std_msgs/Bool'
  });
  this.heaterTopic.advertise();
};

WeatherMachine.prototype.requestLocation = function(lat, lon) {
  var self = this;
  this.lastWeatherRequest_ = Date.now();
  var url = 'http://api.wunderground.com/api/{id}/geolookup/q/{lat},{lon}.json'
    .replace('{lat}', lat.toFixed(2))
    .replace('{lon}', lon.toFixed(2))
    .replace('{id}', 'fada70572c6a3472');
  //console.log(url);
  var req = new XMLHttpRequest();
  req.onreadystatechange = function() {
    self.handleLocationResponse(req);
  };
  req.open('GET', url, true);
  req.send(null);
};

WeatherMachine.prototype.handleLocationResponse = function(res) {
  if (res.status != 200) {
    console.error('Error in location request!');
    // turn everything off?
    return;
  }
  var data = JSON.parse(res.response);
  var locationUrl = data.location.l;
  this.requestWeather(locationUrl);
};

WeatherMachine.prototype.requestWeather = function(locationUrl) {
  var self = this;
  var url = 'http://api.wunderground.com/api/{id}/conditions/{url}.json'
    .replace('{url}', locationUrl)
    .replace('{id}', 'fada70572c6a3472');
  //console.log(url);
  var req = new XMLHttpRequest();
  req.onreadystatechange = function() {
    self.handleWeatherResponse(req);
  };
  req.open('GET', url, true);
  req.send(null);
};

WeatherMachine.prototype.handleWeatherResponse = function(res) {
  if (res.status != 200) {
    console.error('Error in weather request!');
    //this.turnHeatOff();
    return;
  }
  var data = JSON.parse(res.response);
  var weather = data.current_observation;
  var temp = weather.temp_c;
  //console.log('temperature:', temp, 'C');
  if (temp > this.HOT_TEMPERATURE) {
    this.turnHeatOn();
  } else {
    this.turnHeatOff();
  }
};

WeatherMachine.prototype.turnHeatOn = function() {
  if (!this.heatOn) {
    //console.log('heat ON');
    this.heatOn = true;
    this.heaterTopic.publish(new ROSLIB.Message({ data: true }));
  }
};

WeatherMachine.prototype.turnHeatOff = function() {
  if (this.heatOn) {
    //console.log('heat OFF');
    this.heatOn = false;
    this.heaterTopic.publish(new ROSLIB.Message({ data: false }));
  }
};

/**
 * Respond to an incoming pose message, with weather if appropriate.
 * @param {Object} stampedPose The portal stamped pose msg object.
 */
WeatherMachine.prototype.handlePoseChange = function(stampedPose) {
  var now = Date.now();
  var pose = stampedPose.pose;

  var altitude = pose.position.z;
  var latitude = pose.position.y;
  var longitude = pose.position.x;

  if (altitude < this.CEILING) {
    if (now > (this.lastWeatherRequest_ + this.weatherRequestInterval_)) {
      this.requestLocation(latitude, longitude);
    }
  } else {
    this.turnHeatOff();
  }
};

