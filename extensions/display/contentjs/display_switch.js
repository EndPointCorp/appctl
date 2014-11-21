$(document).ready(function(){
  
  console.log("Adding div element");

  var d = document.createElement('div');
  d.setAttribute('id', 'fade_overlay');
  d.style.cssText = 'position:fixed; top:0px; left:0px; width:100%; height:100%; z-index:10009876; background-color:black;';
  document.body.appendChild(d);

  console.log("x2");

  $('#fade_overlay').fadeOut(5000, function() { $('#fade_overlay').remove(); console.log("x3"); } );
});

var portalRosDisplay = new ROSLIB.Ros({
  url: 'wss://42-b:9090'
});

var displaySwitchTopic = new ROSLIB.Topic({
  ros: portalRosDisplay,
  name: '/display/switch',
  messageType: 'std_msgs/String',
});

displaySwitchTopic.subscribe(function(msg){
  var url = msg.data;

  // the below line causes the white flickering

  var d = document.createElement('div');
  d.setAttribute('id', 'fade_overlay');
  d.style.cssText = 'display: none; position:fixed; top:0px; left:0px; width:100%; height:100%; z-index:10009876; background-color:black;';
  document.body.appendChild(d);

  $('#fade_overlay').fadeIn(2000, function() { document.location = url } );
});
