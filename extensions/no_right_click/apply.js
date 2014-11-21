/**
 * Disables context menus, including those initiated by a right click.
 */
function disableRightClick() {
  window.addEventListener('contextmenu', function(e) {
    e.preventDefault();
    return false;
  }, true);
}

disableRightClick();
