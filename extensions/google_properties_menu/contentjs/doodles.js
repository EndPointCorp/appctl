var runPacman = function() {

    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#pacman_selected").show();

    // and send the ROS message
    sendSwitchROSMessage($(this));
}

$('#pacman').click(function(){
  runPacman();
});

$('#pacman').addEventListener(
  'touchstart', 
  runPacman,
  true
);