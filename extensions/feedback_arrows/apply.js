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
  url: 'ws://master:9090'
});

console.log('Loading Feedback arrows Extension: initializing ROS');

// Define topic object
var feedbackArrowsSpacenavListener = new ROSLIB.Topic({
  ros: spacenavROS,
  name: 'spacenav/twist',
  messageType: 'geometry_msgs/Twist'
});


// Subscribe to spacenav topic + ROS messages rate limiting
console.log('Loading Feedback arrows Extension: subscribing to spacenav topic');
var counter = 0;
var sendEachNoMessage = 10;
feedbackArrowsSpacenavListener.subscribe(function(msg){

  if (
		  // rotate our models here
	  msg.linear.x == 0
      && msg.linear.y == 0
      && msg.linear.z == 0
      && msg.angular.x == 0
      && msg.angular.y == 0
      && msg.angular.z == 0
      ) {

        counter = 0;
        return;
   }

  if (counter % sendEachNoMessage == 0) {
     counter = 1;
     console.log("Feedback arrows Extension: catched message");
  } else {
    counter += 1;
  }
  console.log("Counter " + counter);
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

	controls = new THREE.TrackballControls( camera );
	  controls.rotateSpeed = 5.0;
	  controls.zoomSpeed = 5;
	  controls.panSpeed = 2;
	  controls.noZoom = false;
	  controls.noPan = false;
	  controls.staticMoving = true;
	  controls.dynamicDampingFactor = 0.3;
	  
	container = document.createElement( 'div' );
	container.id = 'feedback_arrows';
	document.body.appendChild( container );

	camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 1, 2000 );
	camera.position.z = 100;

	// scene

	scene = new THREE.Scene();

	var ambient = new THREE.AmbientLight( 0x101030 );
	scene.add( ambient );

	var directionalLight = new THREE.DirectionalLight( 0xffeedd );
	directionalLight.position.set( 0, 0, 1 );
	scene.add( directionalLight );

	// texture

	var manager = new THREE.LoadingManager();
	manager.onProgress = function ( item, loaded, total ) {

		console.log( item, loaded, total );

	};

	var texture = new THREE.Texture();
	var imaginator = new Image();
	imaginator.src = "data:image/jpeg;base64," + "R0lGODlhkAGQAfAAAP///wAAACH5BAAAAAAAIf4Kc2NyaThlLmNvbQAsAAAAAJABkAEAAv6Ej6nL7Q+jnLTai7PevPsPhuJIluaJpurKtu4Lx/JM1/aN5/rO9/4PDAqHxKLxiEwql8ym8wmNSqfUqvWKzWq33K73Cw6Lx+Sy+YxOq9fstvsNj8vn9Lr9js/r9/y+/w8YKDhIWGh4iJiouMjY6PgIGSk5SVlpeYmZqbnJ2en5CRoqOkpaanqKmqq6ytrq+gobKztLW2t7i5uru8vb6/sLHCw8TFxsfIycrLzM3Oz8DB0tPU1dbX2Nna29zd3t/Q0eLj5OXm5+jp6uvs7e7v4OHy8/T19vf4+fr7/P3+//DzCgwIEECxo8iDChwoUMGzp8CDGixIkUK1q8iDGjxv6NHDt6/AgypMiRJEuaPIkypcqVLFu6fAkzpsyZNGvavIkzp86dPHv6/Ak0qNChRIsaPYo0qdKlTJs6fQo1qtSpVKtavYo1q9atXLt6/Qo2rNixZMuaPYs2rdq1bNu6fQs3rty5dOvavYs3r969fPv6/Qs4sODBhAsbPow4seLFjBs7fgw5suTJlCtbvow5s+bNnDt7/gw6tOjRpEubPo06terVrFu7fg07tuzZtGvbvo07t+7dvHv7/g08uPDhxIsbP448ufLlzJs7fw49uvTp1Ktbv449u/bt3Lt7/w4+vPjx5MubP48+vfr17Nu7fw8/vvz59Ovbv48/v/79/E/7+/8PYIACDkhggQYeiGCCCi7IYIMOPghhhBJOSGGFFl6IYYYabshhhx5+CGKIIo5IYokmnohiiiquyGKLLr4IY4wyzkhjjTbeiGOObRQAADs="
    texture.image = imaginator;
	texture.needsUpdate = true;
	// EOFD

	var loader_obj = new THREE.OBJLoader( manager );
	loader_obj.load( chrome.extension.getURL('models/arrows.obj'), function ( object ) {

		object.traverse( function ( child ) {

			if ( child instanceof THREE.Mesh ) {

				child.material.map = texture;

			}

		} );

		//object.position.y = - 80;
		scene.add( object );

	} );

	//

	renderer = new THREE.WebGLRenderer({ alpha: true });
	renderer.setSize( window.innerWidth, window.innerHeight );
	container.appendChild( renderer.domElement );
	renderer.setClearColor( 0xffffff, 1);
	document.addEventListener( 'mousemove', onDocumentMouseMove, false );

	//

	window.addEventListener( 'resize', onWindowResize, false );

}

function onWindowResize() {

	windowHalfX = window.innerWidth / 2;
	windowHalfY = window.innerHeight / 2;

	camera.aspect = window.innerWidth / window.innerHeight;
	camera.updateProjectionMatrix();

	renderer.setSize( window.innerWidth, window.innerHeight );

}

function onDocumentMouseMove( event ) {

	mouseX = ( event.clientX - windowHalfX ) / 2;
	mouseY = ( event.clientY - windowHalfY ) / 2;

}

//

function animate() {

	requestAnimationFrame( animate );
	render();

}

function render() {

	camera.position.x += ( mouseX - camera.position.x ) * .05;
	camera.position.y += ( - mouseY - camera.position.y ) * .05;

	camera.lookAt( scene.position );

	renderer.render( scene, camera );

}













/***




var container;
var camera, scene, renderer;
var mouseX = 0, mouseY = 0;

var windowHalfX = window.innerWidth / 2;
var windowHalfY = window.innerHeight / 2;



function init() {
  // This <div> will host the canvas for our scene.
  container = document.createElement( 'div' );
  container.id = 'feedback_arrows';
  
  document.body.appendChild(container);
  
  // You can adjust the cameras distance and set the FOV to something
  // different than 45°. The last two values set the clippling plane.
  camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 1, 2000 );
  camera.position.x = 33;
  camera.position.y = 33;
  camera.position.z = 33;
  
  // This is the scene we will add all objects to.
  scene = new THREE.Scene();

  // You can set the color of the ambient light to any value.
  // I have chose a completely white light because I want to paint
  // all the shading into my texture. You propably want something darker.
  var ambient = new THREE.AmbientLight( 0xffffff );
  scene.add( ambient );

  // Uncomment these lines to create a simple directional light.
  var directionalLight = new THREE.DirectionalLight( 0xffeedd );
  directionalLight.position.set( 0, 0, 1 ).normalize();
  scene.add( directionalLight );

 
  var manager = new THREE.LoadingManager();
  manager.onProgress = function ( item, loaded, total ) {
    console.log( "Feedback arrows threejs manager => ", item, loaded, total );
  };
  
  //texturing
  var texture = new THREE.Texture();
  var image_loader = new THREE.ImageLoader( manager );
  image_loader.load( 'arrows_texture.gif', function ( image ) {
    texture.image = image;
    texture.needsUpdate = true;
    console.log('Loading Feedback arrows Extension: texture');
  } );

  
  //obj loading
  var obj_loader = new THREE.OBJLoader( manager );

  // As soon as the OBJ has been loaded this function looks for a mesh
  // inside the data and applies the texture to it.
  
  obj_loader.load( 'arrows.obj', function ( event ) {
    var object = event;
    object.traverse( function ( child ) {
      if ( child instanceof THREE.Mesh ) {
    	 console.log('Loading Feedback arrows Extension: preloading object and adding texture');
    	 var material = new THREE.MeshBasicMaterial({color: 0xff00ff});
         child.material.map = material;
      }
    } 
    );
    // upward scaling
    //object.scale = new THREE.Vector3( 100, 100, 100 );

    // You can change the position of the object, so that it is not
    // centered in the view and leaves some space for overlay text.
    object.position.y -= 2.5;
    console.log('Loading Feedback arrows Extension: adding object to scene');
    scene.add(object);

    //var geometry = new THREE.SphereGeometry(10);
    //var material = new THREE.MeshBasicMaterial({color: 0xff00ff});
    //var sphere = new THREE.Mesh(geometry, material);
    //scene.add(sphere);

    //debugging
    //console.log("Sphere at " + sphere.position.x + " " + sphere.position.y + " " + sphere.position.z);
    
  });

  // We set the renderer to the size of the window and
  // append a canvas to our HTML page.
  //renderer = new THREE.WebGLRenderer({ alpha: false });
  renderer = new THREE.WebGLRenderer();
  //renderer.setClearColor( 0x000000, 0 );
  renderer.setSize( window.innerWidth, window.innerHeight );
  container.appendChild( renderer.domElement );
  console.log('Loading Feedback arrows Extension: FINISHED');
}

function animate() {
  // This function calls itself on every frame. You can for example change
  // the objects rotation on every call to create a turntable animation.
  requestAnimationFrame( animate );

  // On every frame we need to calculate the new camera position
  // and have it look exactly at the center of our scene.
  camera.lookAt(scene.position);
  renderer.render(scene, camera);
}

console.log('Loading Feedback arrows Extension: Initializing three.js');
init();
console.log('Loading Feedback arrows Extension: Starting animation');
animate();
***/
