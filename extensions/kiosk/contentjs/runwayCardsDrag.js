
var runway = document.querySelector('.widget-runway-tray-wrapper');
var initialMousePosition = null;
var initialScrollLeft = null;

runway.addEventListener('mousedown', function(e){
	initialMousePosition = e.pageX;
	initialScrollLeft = runway.scrollLeft;
}, 
true);


runway.addEventListener('mouseup', function(e){
	initialMousePosition = null;
}, 
true);


runway.addEventListener('mousemove', function(e){
	if (initialMousePosition === null) return;

	var dx = e.pageX - initialMousePosition;
	runway.scrollLeft = initialScrollLeft + dx;
}, 
true);
