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

document.getElementById('pacman').addEventListener(
  'touchstart',
  runPacman,
  true
);

// OCEAN

document.getElementById('ocean').addEventListener(
  'touchstart',
  runOcean,
  true
);

// CUBESLAM

document.getElementById('cubeslam').addEventListener(
  'touchstart',
  runCubeslam,
  true
);
