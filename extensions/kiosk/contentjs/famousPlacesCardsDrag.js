var initialMousePosition = null;
var initialScrollLeft = null;

var initiateFamousPlacesHandlers = function() {

  var famousPlaces = document.querySelector('#acme-famous-places');

  if (famousPlaces == null) {
    setTimeout(initiateFamousPlacesHandlers, 2000);
    return;
  }

  famousPlaces.addEventListener('touchstart', function(e) {
    // This should be enabled only in chrome.
    // In chromium, I need to support the multiple touch events when using one
    // finger, so I just take the first one currently.
    /*
     *if (e.touches.length > 1) {
     *  console.log('No multiple touch allowed');
     *  return;
     *}
     */

    initialMousePosition = e.touches[0].pageX;
    initialScrollLeft = famousPlaces.scrollLeft;
    e.stopPropagation();
  },
  true);


  famousPlaces.addEventListener('touchend', function(e) {
    initialMousePosition = null;
    e.stopPropagation();
  },
  true);


  famousPlaces.addEventListener('touchmove', function(e) {
    if (initialMousePosition === null) return;

    // This should be enabled only in chrome.
    // In chromium, I need to support the multiple touch events when using one
    // finger, so I just take the first one currently.
    /*
     *if (e.touches.length > 1) {
     *  console.log('No multiple touch allowed');
     *  return;
     *}
     */


    // this scrolls like I feel it should...
    var dx = initialMousePosition - e.touches[0].pageX;
    famousPlaces.scrollLeft = initialScrollLeft + dx;
    e.stopPropagation();
  },
  true);

  };

initiateFamousPlacesHandlers();
