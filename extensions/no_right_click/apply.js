/**
 * Disables context menus, including those initiated by a right click.
 */
function disableRightClick() {
  window.addEventListener('oncontextmenu', function() {
    return false;
  }, true);
}

disableRightClick();
