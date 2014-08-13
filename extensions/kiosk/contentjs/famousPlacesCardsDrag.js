console.log("Initializing the famousPlaces handlers.");

var initialMousePosition = null;
var initialScrollLeft = null;

var initiateFamousPlacesHandlers = function() {

	var famousPlaces = document.querySelector('#acme-famous-places');

	console.log(famousPlaces);

	if (famousPlaces == null) {
		console.log("FamousPlaces are not ready yet... wating for a while...");
		setTimeout(initiateFamousPlacesHandlers, 2);
		return;
	}

	famousPlaces.addEventListener('touchstart', function(e){
		console.log('Famous touchstart');

		// this should be enabled only in chrome
		// in chromium I need to support the multiple touch events when using one finger,
		// so I just take the first one currently
		// if (e.touches.length > 1) { console.log('No multiple touch allowed'); return; }

		initialMousePosition = e.touches[0].pageX;
		initialScrollLeft = famousPlaces.scrollLeft;
		e.stopPropagation();
	},
	true);


	famousPlaces.addEventListener('touchend', function(e){
		console.log('Famous touchend');
		initialMousePosition = null;
		e.stopPropagation();
	},
	true);


	famousPlaces.addEventListener('touchmove', function(e){
		console.log('Famous touchmove');
		if (initialMousePosition === null) return;

		// this should be enabled only in chrome
		// in chromium I need to support the multiple touch events when using one finger,
		// so I just take the first one currently
		//if (e.touches.length > 1) { console.log('No multiple touch allowed'); return; }


		// this scrolls like I feel it should...
		var dx = initialMousePosition - e.touches[0].pageX;
		famousPlaces.scrollLeft = initialScrollLeft + dx;
		e.stopPropagation();
	},
	true);

	};

initiateFamousPlacesHandlers();
