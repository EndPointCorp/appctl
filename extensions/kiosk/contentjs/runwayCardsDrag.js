console.log("Initializing the runway handlers.");
var runway = document.querySelector('.widget-runway-tray-wrapper');

var initialMousePosition = null;
var initialScrollLeft = null;

runway.addEventListener('touchstart', function(e){
  console.log('touchstart');

  // this should be enabled only in chrome
  // in chromium I need to support the multiple touch events when using one finger,
  // so I just take the first one currently
  // if (e.touches.length > 1) { console.log('No multiple touch allowed'); return; }

  initialMousePosition = e.touches[0].pageX;
  initialScrollLeft = runway.scrollLeft;
},
true);


runway.addEventListener('touchend', function(e){
  console.log('touchend');
  initialMousePosition = null;
},
true);


runway.addEventListener('touchmove', function(e){
  console.log('touchmove');
  if (initialMousePosition === null) return;

  // this should be enabled only in chrome
  // in chromium I need to support the multiple touch events when using one finger,
  // so I just take the first one currently
  //if (e.touches.length > 1) { console.log('No multiple touch allowed'); return; }


  // this scrolls like I feel it should...
  var dx = initialMousePosition - e.touches[0].pageX;
  runway.scrollLeft = initialScrollLeft + dx;
},
true);
