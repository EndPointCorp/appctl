/*
 * Feedback arrows UX extension.
 * Shows two independent webgl models that should immitate spacenav movement.
 * Object should fade in and fade out on spacenav use.
 */
/*
 * Feedback arrows UX extension.
 * Shows two independent webgl models that should immitate spacenav movement.
 * Object should fade in and fade out on spacenav use.
 */

console.log('Loading Feedback arrows Extension: creating WS socket');

//Let's connect directly to websocket<->ROS bridge
var spacenavROS = new ROSLIB.Ros({
	url : 'ws://master:9090'
});

console.log('Loading Feedback arrows Extension: initializing ROS');

// Define topic object
var feedbackArrowsSpacenavListener = new ROSLIB.Topic({
	ros : spacenavROS,
	name : 'spacenav/twist',
	messageType : 'geometry_msgs/Twist',
	throttle_rate : 50
});

// we'll need a map function
Number.prototype.map = function ( in_min , in_max , out_min , out_max ) {
	  return ( this - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;
	}

function setOpacity(threejs_obj, opacity) {
	threejs_obj.traverse(function(child) {
		if (child instanceof THREE.Mesh) {
			child.material.opacity = opacity;
		}
	});
}

var ringMultiplier = 0.1;
var arrowMultiplier = 0.1;
var spacenav_min = -350;
var spacenav_max = 350;

var arrows_min = -3;
var arrows_max = 3;

var arrowObjPosition = [ 0, 0, 0, 0, 0, 0 ];
var ringObjPosition = [ 0, 0, 0, 0, 0, 0 ];

// Subscribe to spacenav topic + ROS messages rate limiting
console.log('Loading Feedback arrows Extension: subscribing to spacenav topic');

//Let's normalize values with .map function ( in_min , in_max , out_min , out_max )

feedbackArrowsSpacenavListener.subscribe(function(msg) {
	this.msg = msg; 
	if (this.msg.linear.x == 0 && this.msg.linear.y == 0
			&& this.msg.linear.z == 0 && this.msg.angular.x == 0
			&& this.msg.angular.y == 0 && this.msg.angular.z == 0) 
		{ 
		/*** 
		 * - fade objects out 
		 * - return to point 0 
		 */
		for (var i = 0; i < arrowObjPosition.length; i++) {
			arrowObjPosition[i] = 0;
			ringObjPosition[i] = 0;
		}
		// set opacity to 0 
		if ((typeof(arrowObj) === "undefined") && (typeof(ringObj) === "undefined")) {
			console.log("Initializing objects");
		} else {
			setOpacity(ringObj, 0);
			setOpacity(arrowObj, 0);	
		}
		return;

	} else { 
		/***
		 * - map spacenav values to threejs object coordinates  
		 * - fade in
		 * - move objects 
		 *
		 *	 ObjPosition = 
		 *	 [
		 *	 0 : "go front right", 
		 *	 1 : "go up (z)",
		 *	 2 : "go front left",
		 *	 3 : "rotate over y axis",
		 *	 4 : "rotate over center",
		 *	 5 : "rotate over x axis"
		 *	 ]
		 *	
		 *	 rostopic echo /spacenav/twist:
		 *	
		 *	 [
		 *	 "push then pull" : "linear z -350 => +350",
		 *	 "rotate from left to right" : "angular z: +350 => -350",
		 *	 "move backward then forward" : "linear x -350=>+350",
		 *	 "move left then right" : "linear y +350 => -350",
		 *	 "lean left then lean right" : "angular x -350 => +350",
		 *	 "lean forward then lean backward" : "angular y +350 => -350"
		 *	 ]
		 *
		 ***/
		
		// needed for arrow
		this.msg.linear.x = this.msg.linear.x.map(spacenav_min, spacenav_max, -1, 1);
		this.msg.linear.y = this.msg.linear.y.map(spacenav_min, spacenav_max, -1, 1);
		
		// needed for ring
		this.msg.linear.z = this.msg.linear.z.map(spacenav_min, spacenav_max, arrows_min, arrows_max);
		this.msg.angular.x = this.msg.angular.x.map(spacenav_min, spacenav_max, arrows_min, arrows_max);
		this.msg.angular.y = this.msg.angular.y.map(spacenav_min, spacenav_max, arrows_min, arrows_max);
		this.msg.angular.z = this.msg.angular.z.map(spacenav_min, spacenav_max, arrows_min, arrows_max);
			
		// make object transparency proportional to the values
		setOpacity(arrowObj, 0.7);
		setOpacity(ringObj, 0.7);	
		
		console.log("Setting opacity to 0.9");
		
		// pull up , push down
		ringObjPosition[1] = this.msg.linear.z;
		// rotate (twist)
		ringObjPosition[4] = this.msg.angular.z * 0.5;
		console.log("this.msg.angular.z =>", this.msg.angular.z);
		// lean forward and backward
		ringObjPosition[5] = this.msg.angular.x * -0.1;
		ringObjPosition[3] = this.msg.angular.y * -0.1;
		

		// let's rotate and show the direction arrow with little tresholding
		if ((Math.abs(this.msg.linear.y) > 0.2) || (Math.abs(this.msg.linear.x) > 0.2)) {
			this.msg.linear.y = this.msg.linear.y * -1;
			direction = (Math.atan2(this.msg.linear.y, this.msg.linear.x)/ Math.PI * 180)/ -18;
			console.log("This is direction1:", direction,
					"computed out of (x,y)", this.msg.linear.y, "/",
					this.msg.linear.x);
			arrowObjPosition[4] = direction;
		}
	}

	
});

// Three.js part
var container, stats;
var camera, scene, renderer;
var mouseX = 0, mouseY = 0;

var windowHalfX = window.innerWidth / 2;
var windowHalfY = window.innerHeight / 2;

console.log("Arrows: before init");
init();
console.log("Arrows: after init");
console.log("Arrows: before animate");
animate();
console.log("Arrows: after animate");

function init() {
	container = document.createElement('div');
	container.id = 'feedback_arrows';
	document.body.appendChild(container);

	camera = new THREE.PerspectiveCamera(45, window.innerWidth
			/ window.innerHeight, 1, 2000);
	camera.position.x = 0;
	camera.position.y = 10;
	camera.position.z = 33;

	// scene

	scene = new THREE.Scene();

	var ambient = new THREE.AmbientLight(0x101030);
	scene.add(ambient);

	var directionalLight = new THREE.DirectionalLight(0xffeedd);
	directionalLight.position.set(0, 0, 1);
	scene.add(directionalLight);

	// texture

	var manager = new THREE.LoadingManager();
	manager.onProgress = function(item, loaded, total) {

		console.log(item, loaded, total);

	};

	var ring_texture = new THREE.Texture();
	var ring_imaginator = new Image();
	ring_imaginator.src = "data:image/jpeg;base64,"
			+ "R0lGODlhkAGQAfAAAP///wAAACH5BAAAAAAAIf4Kc2NyaThlLmNvbQAsAAAAAJABkAEAAv6Ej6nL7Q+jnLTai7PevPsPhuJIluaJpurKtu4Lx/JM1/aN5/rO9/4PDAqHxKLxiEwql8ym8wmNSqfUqvWKzWq33K73Cw6Lx+Sy+YxOq9fstvsNj8vn9Lr9js/r9/y+/w8YKDhIWGh4iJiouMjY6PgIGSk5SVlpeYmZqbnJ2en5CRoqOkpaanqKmqq6ytrq+gobKztLW2t7i5uru8vb6/sLHCw8TFxsfIycrLzM3Oz8DB0tPU1dbX2Nna29zd3t/Q0eLj5OXm5+jp6uvs7e7v4OHy8/T19vf4+fr7/P3+//DzCgwIEECxo8iDChwoUMGzp8CDGixIkUK1q8iDGjxv6NHDt6/AgypMiRJEuaPIkypcqVLFu6fAkzpsyZNGvavIkzp86dPHv6/Ak0qNChRIsaPYo0qdKlTJs6fQo1qtSpVKtavYo1q9atXLt6/Qo2rNixZMuaPYs2rdq1bNu6fQs3rty5dOvavYs3r969fPv6/Qs4sODBhAsbPow4seLFjBs7fgw5suTJlCtbvow5s+bNnDt7/gw6tOjRpEubPo06terVrFu7fg07tuzZtGvbvo07t+7dvHv7/g08uPDhxIsbP448ufLlzJs7fw49uvTp1Ktbv449u/bt3Lt7/w4+vPjx5MubP48+vfr17Nu7fw8/vvz59Ovbv48/v/79/E/7+/8PYIACDkhggQYeiGCCCi7IYIMOPghhhBJOSGGFFl6IYYYabshhhx5+CGKIIo5IYokmnohiiiquyGKLLr4IY4wyzkhjjTbeiGOObRQAADs="
	ring_texture.image = ring_imaginator;
	ring_texture.needsUpdate = true;
	
	
	var arrow_texture = new THREE.Texture();
	var arrow_imaginator = new Image();
	arrow_imaginator.src = "data:image/jpeg;base64,"
			+ "R0lGODlhkAGQAfAAAP///wAAACH5BAAAAAAAIf4Kc2NyaThlLmNvbQAsAAAAAJABkAEAAv6Ej6nL7Q+jnLTai7PevPsPhuJIluaJpurKtu4Lx/JM1/aN5/rO9/4PDAqHxKLxiEwql8ym8wmNSqfUqvWKzWq33K73Cw6Lx+Sy+YxOq9fstvsNj8vn9Lr9js/r9/y+/w8YKDhIWGh4iJiouMjY6PgIGSk5SVlpeYmZqbnJ2en5CRoqOkpaanqKmqq6ytrq+gobKztLW2t7i5uru8vb6/sLHCw8TFxsfIycrLzM3Oz8DB0tPU1dbX2Nna29zd3t/Q0eLj5OXm5+jp6uvs7e7v4OHy8/T19vf4+fr7/P3+//DzCgwIEECxo8iDChwoUMGzp8CDGixIkUK1q8iDGjxv6NHDt6/AgypMiRJEuaPIkypcqVLFu6fAkzpsyZNGvavIkzp86dPHv6/Ak0qNChRIsaPYo0qdKlTJs6fQo1qtSpVKtavYo1q9atXLt6/Qo2rNixZMuaPYs2rdq1bNu6fQs3rty5dOvavYs3r969fPv6/Qs4sODBhAsbPow4seLFjBs7fgw5suTJlCtbvow5s+bNnDt7/gw6tOjRpEubPo06terVrFu7fg07tuzZtGvbvo07t+7dvHv7/g08uPDhxIsbP448ufLlzJs7fw49uvTp1Ktbv449u/bt3Lt7/w4+vPjx5MubP48+vfr17Nu7fw8/vvz59Ovbv48/v/79/E/7+/8PYIACDkhggQYeiGCCCi7IYIMOPghhhBJOSGGFFl6IYYYabshhhx5+CGKIIo5IYokmnohiiiquyGKLLr4IY4wyzkhjjTbeiGOObRQAADs="
	arrow_texture.image = arrow_imaginator;
	arrow_texture.needsUpdate = true;
	// EOFD

	var arrows_loader = new THREE.OBJLoader(manager);
	arrows_loader.load(chrome.extension.getURL('models/arrow.obj'), function(
			arrow_object) {
		this.arrowObj = arrow_object;
		arrow_object.traverse(function(child) {
			if (child instanceof THREE.Mesh) {
				child.material.map = arrow_texture;
				child.material.transparent = true;
				child.material.opacity = 0.7;
			}
		});
		scene.add(arrow_object);
	});

	var ring_loader = new THREE.OBJLoader(manager);
	ring_loader.load(chrome.extension.getURL('models/ring.obj'), function(
			ring_object) {
		this.ringObj = ring_object;
		ring_object.traverse(function(child) {
			if (child instanceof THREE.Mesh) {
				child.material.map = ring_texture;
				child.material.transparent = true;
				child.material.opacity = 0.7;
				
				this.ring_opacity = child.material.opacity;
				
				//ring_material = child.material;
				//ring_material.transparent = true;
				//ring_material.opacity = 1;
			}
		});
		scene.add(ring_object);
	});

	//

	// let's make transparent background
	renderer = new THREE.WebGLRenderer({
		alpha : true
	});
	renderer.setSize(window.innerWidth, window.innerHeight);
	container.appendChild(renderer.domElement);
	renderer.setClearColor(0x000000, 0);
	window.addEventListener('resize', onWindowResize, false);

}


// we need this for debug purposes
function onWindowResize() {
	windowHalfX = window.innerWidth / 2;
	windowHalfY = window.innerHeight / 2;
	camera.aspect = window.innerWidth / window.innerHeight;
	camera.updateProjectionMatrix();
	renderer.setSize(window.innerWidth, window.innerHeight);
}

//

function animate() {
	requestAnimationFrame(animate);
	render();
}

function render() {
	if (typeof(arrowObj) === "undefined") {
		console.log("Still initializing arrowObj");
		}
		else {
		arrowObj.position.x = arrowObjPosition[0];
		arrowObj.position.y = arrowObjPosition[1];
		arrowObj.position.z = arrowObjPosition[2];
		arrowObj.rotation.x = arrowObjPosition[3];
		arrowObj.rotation.y = arrowObjPosition[4];
		arrowObj.rotation.z = arrowObjPosition[5];
		}
	
	if ( typeof(ringObj) === "undefined") {
		console.log("Still initializing ringObj");
	} else {
		ringObj.position.x = ringObjPosition[0];
		ringObj.position.y = ringObjPosition[1];
		ringObj.position.z = ringObjPosition[2];
		ringObj.rotation.x = ringObjPosition[3];
		ringObj.rotation.y = ringObjPosition[4];
		ringObj.rotation.z = ringObjPosition[5];
	}
	camera.lookAt(scene.position);
	renderer.render(scene, camera);
}
