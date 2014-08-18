var runPacman = function(e) {

    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#pacman_selected").show();

    // and send the ROS message
    sendSwitchROSMessage(e);
}

$('#pacman').click(function(){
  runPacman();
});

document.getElementById('pacman').addEventListener(
  'touchstart',
  runPacman,
  true
);