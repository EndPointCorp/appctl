var runPacman = function(e) {

    $('.game').removeClass("selected");
    $('.description').hide();
    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#pacman_selected").show();

    propertiesMenu.handleDoodleSelection(e);
}

var runOcean = function(e) {

    $('.game').removeClass("selected");
    $('.description').hide();
    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#ocean_selected").show();

    propertiesMenu.handleDoodleSelection(e);
}

var runCubeslam = function(e) {

    $('.game').removeClass("selected");
    $('.description').hide();
    $(this).addClass("selected");
    $("#choose_game").hide();
    $("#cubeslam_selected").show();

    propertiesMenu.handleDoodleSelection(e);
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
