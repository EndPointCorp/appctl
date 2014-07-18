/*
 * Onboard on-screen keyboard integration.
 * Shows onboard when tapped the search field.
 * Hides onboard when tapped anywhere else, or spacenav is moved.
 */

var onboardRos = new ROSLIB.Ros({
  url: 'ws://master:9090'
});

// This topic object is used for publishing the show and hide messages.
var onboardPublisher = new ROSLIB.Topic({
  ros: onboardRos,
  name: 'onboard/visibility',
  messageType: 'std_msgs/Bool'
});

// Messages to be sent by onboardPublisher, they show and hide onboard.
var onboardShowMsg = new ROSLIB.Message({data: true});
var onboardHideMsg = new ROSLIB.Message({data: false});

// We need to hide the keyboard when spacenavigator is touched
// for that we need to listen to the spacenav/twist
// and react when there is something else than zero anywhere.
var onboardSpacenavListener = new ROSLIB.Topic({
  ros: onboardRos,
  name: 'spacenav/twist',
  messageType: 'geometry_msgs/Twist',
  throttle_rate: 250
});

onboardSpacenavListener.subscribe(function(msg){

  if (   msg.linear.x == 0
      && msg.linear.y == 0
      && msg.linear.z == 0
      && msg.angular.x == 0
      && msg.angular.y == 0
      && msg.angular.z == 0) {

        return;
  }

  hideOnboard();
});


function showOnboard() {
  console.log('Showing Onboard keyboard');
  onboardPublisher.publish(onboardShowMsg);
}

function hideOnboard() {
  console.log('Hiding Onboard keyboard');
  onboardPublisher.publish(onboardHideMsg);
}

// Adds callbacks to the search field.
// onclick - shows keyboard
// onblur  - hides keyboard
function addCallbacks() {
  console.log('adding callbacks');

  /* This is needed because Chrome tries loading this plugin when
   * DOM is not ready yet, `even with run_at: document_end`
   */
  if (document.readyState !== 'complete') {
    console.log('Document is not ready yet, going to sleep for a while.');
    setTimeout(addCallbacks, 500);
    return;
  }

  var tx = document.querySelector('#searchboxinput');
  if (tx) {
    console.log('adding onboard event');
    tx.addEventListener('click', showOnboard);
    tx.addEventListener('blur', hideOnboard);
  }
}

console.log('Loading Onboard Extension');
addCallbacks();
