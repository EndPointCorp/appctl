console.log('Pacman extension loading');


var initiatePacmanSpacenavHandlers = function() {

  var GUTTER = 180;

  var keyBindings = {
    'LEFT': 37,
    'UP': 38,
    'RIGHT': 39,
    'DOWN': 40
  };

  var pacmanFrame = document.getElementById('i');

  if (pacmanFrame == null) {
    console.log('DOM elements are not ready yet... wating for a while...');
    setTimeout(initiatePacmanSpacenavHandlers, 2000);
    return;
  }

  var pacmanWindow = pacmanFrame.contentWindow;

  var slinkyRosKiosk = new ROSLIB.Ros({
    url: 'wss://42-b:9090'
  });

  var navigatorListener = new ROSLIB.Topic({
    ros: slinkyRosKiosk,
      name: '/spacenav/twist',
      messageType: 'geometry_msgs/Twist',
      throttle_rate: 30
  });

  function keydown(which) {
    var e = new Event('keydown');
    e.which = which;
    e.keyCode = which;
    return e;
  }

  var previousMessage = '';

  function move(direction) {
    if (previousMessage == direction) return;
    previousMessage = direction;

    var keyCode = keyBindings[direction];
    pacmanWindow.dispatchEvent(keydown(keyCode));
  };

  navigatorListener.subscribe(function(twist) {

    // use a combination of linear and angular values
    var spacenavX = -twist.linear.y + twist.angular.x;
    var spacenavY = twist.linear.x + twist.angular.y;

    if (spacenavY > GUTTER) move('UP');
    else if (spacenavY < -GUTTER) move('DOWN');
    else if (spacenavX > GUTTER) move('RIGHT');
    else if (spacenavX < -GUTTER) move('LEFT');
  });

  console.log('Pacman loaded');
};

initiatePacmanSpacenavHandlers();
