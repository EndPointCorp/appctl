var runPacman = function(e) {

    $('.game').removeClass("selected");
    $('.description').hide();
    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#pacman_selected").show();

    // and send the ROS message
    sendSwitchROSMessage(e);
}

var runOcean = function(e) {

    $('.game').removeClass("selected");
    $('.description').hide();
    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#ocean_selected").show();

    // and send the ROS message
    sendSwitchROSMessage(e);
}

var runCubeslam = function(e) {

    $('.game').removeClass("selected");
    $('.description').hide();
    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#cubeslam_selected").show();

    // and send the ROS message
    //sendSwitchROSMessage(e); // cubeslam is a placeholder now
}

// PACMAN

$('#pacman').click(runPacman);

document.getElementById('pacman').addEventListener(
  'touchstart',
  runPacman,
  true
);

// OCEAN
$('#ocean').click(runOcean);

document.getElementById('ocean').addEventListener(
  'touchstart',
  runOcean,
  true
);

// CUBESLAM
$('#cubeslam').click(runCubeslam);

document.getElementById('cubeslam').addEventListener(
  'touchstart',
  runCubeslam,
  true
);