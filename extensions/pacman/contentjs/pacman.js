console.log('Pacman extension loading');


var initiatePacmanSpacenavHandlers = function() {

  GUTTER = 100;
  MESSAGE_INTERVAL = 200; // [ms]

  messageTs = 0;

  var pacmanFrame = document.getElementById('i');

  if (pacmanFrame == null) {
    console.log("DOM elements are not ready yet... wating for a while...");
    setTimeout(initiatePacmanSpacenavHandlers, 2000);
    return;
  }

  var pacmanWindow = pacmanFrame.contentWindow;

  var slinkyRosKiosk = new ROSLIB.Ros({
    url: 'wss://42-b:9090'
  });

  var navigatorListener = new ROSLIB.Topic({
    ros: slinkyRosKiosk,
      name: '/spacenav/offset',
      messageType: 'geometry_msgs/Vector3',
      throttle_rate: 100
  });

  function keydown(which) {
    var e = new Event('keydown');
    e.which = which;
    e.keyCode = which;
    return e;
  }

  var previousMessage = "";

  function move(direction) {
    // var t = new Date().getTime();

    if (previousMessage == direction) return;
    //if (t - messageTs < MESSAGE_INTERVAL) return;
    //messageTs = t;
    previousMessage = direction;

    if (direction == 'LEFT') left();
    else if (direction == 'RIGHT') right();
    else if (direction == 'DOWN') down();
    else if (direction == 'UP') up();
  };

  function left()  { console.log('LEFT');  pacmanWindow.dispatchEvent(keydown(37)); }
  function up()    { console.log('UP');    pacmanWindow.dispatchEvent(keydown(38)); }
  function right() { console.log('RIGHT'); pacmanWindow.dispatchEvent(keydown(39)); }
  function down()  { console.log('DOWN');  pacmanWindow.dispatchEvent(keydown(40)); }

  navigatorListener.subscribe(function(v){

    if (v.x > GUTTER) move('UP');
    else if (v.x < -GUTTER) move('DOWN');
    else if (v.y > GUTTER) move('LEFT');
    else if (v.y < -GUTTER) move('RIGHT');
  });

}

initiatePacmanSpacenavHandlers();
