console.log("Loading debug.js");

// Yes, this is hardcoded, and will be.
// TODO: Move it to some other place
var PASSWORD = "slinkydog";

// secret menu support
var lastMouseDownTime = null;

var overlayContentURL = chrome.extension.getURL('pages/debug.html');

var hideDebugOverlay = function() {document.getElementById('debug_overlay').style.visibility = 'hidden';};

var showDebugOverlay = function() {document.getElementById('debug_overlay').style.visibility = 'visible';};

var relaunchPortal = function() {console.log("Relaunching Portal.");};


// Update the debug overlay
setInterval(function() {

  var d = $("#debug_overlay");
  if ( d == false || d.is(':visible') == false ) return;

  // Current document url
  var span_url = $("#debug_overlay #current_url .value");
  if (span_url) span_url.text(document.location.href);

  // Current time
  var span_time = $("#debug_overlay #current_timestamp .value");
  if (span_time) span_time.text(new Date());

}, 500);

var initDebugOverlay = function () {
  $("body").append('<div id="debug_overlay">');
  $("#debug_overlay").load(overlayContentURL);
};


var initDebugPassword = function() {
  $('body').append('<div id="debug_overlay_password">');
  $("#debug_overlay_password").append('<input id="password" type="password">');
  $("#debug_overlay_password").keypress(function(e){
    var code = (e.keyCode ? e.keyCode : e.which);
    if(code == 13) {
      var p = $("#password").val();
      document.getElementById('debug_overlay_password').style.visibility = 'hidden';
      if (p == PASSWORD) {
        document.getElementById('debug_overlay').style.visibility = 'visible';
      }
    }
  });
};

var initRelaunchPortalButton = function() {
  var b = $('#debug_overlay #relaunch_portal');
  if (b) b.onclick = relaunchPortal;
  else setTimeout(this, 1000);
}();

var initCloseButton = function() {
	var b = $('#debug_overlay #buttons #close_debug_window');
	if (b) { b.click(hideDebugOverlay); }
	else { setTimeout(this, 1000); console.log("waiting... "); }
}();


var initJsLog = function() {
  var log = $("#jslogs");
  if (log == null) {
    console.log('Loading initJsLog in a moment');
    setTimeout(initJsLog, 1000);
    return;
  }

  var startY = null;

  log.bind('mousedown', function(){  });
  log.bind('mouseup', function(){ startY = null; });
  log.bind('mousemove', function(){});

}

initDebugOverlay();
initDebugPassword();
initJsLog();

////////////////////////////////////////////////////////////////////////////
// ROS

var debugROS = new ROSLIB.Ros({
  url: 'wss://42-b:9090'
});

var logTopic = new ROSLIB.Topic({
  ros:            debugROS,
  name:           '/logs',
  messageType:    'diagnostic_msgs/DiagnosticStatus'
});

var portalKioskCurrentPoseTopic = new ROSLIB.Topic({
  ros:            debugROS,
  name:           '/portal_kiosk/current_pose',
  messageType:    'portal_nav/PortalPose',
  throttle_rate:  30
});

var poseFeedback = function(e) {
  var pose = e.current_pose.position;
  var span = $("#debug_overlay #current_camera_position .value");
  if (span) {
    span.find("#lon").text( parseFloat(pose.x).toFixed(7) );
    span.find("#lat").text( parseFloat(pose.y).toFixed(7) );
    span.find("#alt").text( parseFloat(pose.z).toFixed(7) );
  }
};

var logFeedback = function(e) {
  /* The level values are:
   *  0 - OK
   *  1 - WARNING
   *  2 - ERROR
   *
   * For the rest we don't care,
   * these are just strings to show.
   */

  var level = e.level;
  var name  = e.name;
  var msg   = e.message;
  var host  = e.hardware_id;

  var levelStr = "";
  if (level == 0) levelStr = "DEBUG";
  if (level == 1) levelStr = "WARNING";
  if (level == 2) levelStr = "ERROR";

  var msgText = '<div>';
  msgText += '<span class="level">' + levelStr + '</span>';
  msgText += '<span class="host">'  + host     + '</span>';
  msgText += '<span class="name">'  + name     + '</span>';
  msgText += '<span class="msg">'   + msg      + '</span>';
  msgText += '</div>';

  $("#jslogs").append(msgText);

  var toScroll = $("#jslogs div").toArray().reduce(function(sum, el){ return sum + $(el).outerHeight(); }, 0);
  $("#jslogs").scrollTop(toScroll);

}

portalKioskCurrentPoseTopic.subscribe(poseFeedback);
logTopic.subscribe(logFeedback);
