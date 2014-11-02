var runPacman = function(e) {

    $('.game').removeClass("selected");
    $('.description').hide();
    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#pacman_selected").show();

    // and send the essage
    sendSwitchMessage(e);
}

var runOcean = function(e) {

    $('.game').removeClass("selected");
    $('.description').hide();
    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#ocean_selected").show();

    // and send the message
    sendSwitchMessage(e);
}

var runCubeslam = function(e) {

    $('.game').removeClass("selected");
    $('.description').hide();
    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#cubeslam_selected").show();

    // and send the message
    //sendSwitchessage(e); // cubeslam is a placeholder now
}

// PACMAN

$('#pacman').bind('click', function() {
  $('#pacman').trigger('touchstart');
});

$('#pacman').bind('touchstart', runPacman);

// OCEAN

$('#ocean').bind('mousedown', function(ev) {
  $(ev.target).trigger('touchstart');
});

$('#ocean').bind('touchstart', runOcean);

// CUBESLAM

$('#cubeslam').bind('click', function() {
  $('#cubeslam').trigger('touchstart');
});

$('#cubeslam').bind('touchstart', runCubeslam);
