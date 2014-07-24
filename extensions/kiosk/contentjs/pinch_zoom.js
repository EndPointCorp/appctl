
// this disables the pinch zoom globally, a good place to start
// TODO: find out why a normal touch generates an event with information
// that there are two points on the screen, when I use two fingers, the
// event know about three points.
document.addEventListener("touchmove", function(e){                                               
	console.log('touchmove');
	console.log(e);                                                                                 
	console.log(e.touches);
	console.log(e.touches.length);
	if(e.touches.length >= 3) {  e.preventDefault();  e.stopPropagation();}                         
}, true);

