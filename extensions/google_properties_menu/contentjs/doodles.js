$('#pacman').click(function(){
    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#pacman_selected").show();

    // and send the ROS message
    sendSwitchROSMessage();
});