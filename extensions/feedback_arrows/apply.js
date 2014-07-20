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
var container;
var camera, scene, renderer;
var mouseX = 0, mouseY = 0;

var windowHalfX = window.innerWidth / 2;
var windowHalfY = window.innerHeight / 2;

console.log('Loading Feedback arrows Extension: Initializing three.js');
init();
console.log('Loading Feedback arrows Extension: Starting animation');
animate();

/*** Initialize ***/
function init() {
  // This <div> will host the canvas for our scene.
  container = document.createElement( 'div' );
  container.id = 'feedback_arrows';
  
  document.body.appendChild(container);
  
  // You can adjust the cameras distance and set the FOV to something
  // different than 45°. The last two values set the clippling plane.
  camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 1, 2000 );
  camera.position.z = 100;

  // This is the scene we will add all objects to.
  scene = new THREE.Scene();

  // You can set the color of the ambient light to any value.
  // I have chose a completely white light because I want to paint
  // all the shading into my texture. You propably want something darker.
  var ambient = new THREE.AmbientLight( 0xffffff );
  scene.add( ambient );

  // Uncomment these lines to create a simple directional light.
  // var directionalLight = new THREE.DirectionalLight( 0xffeedd );
  // directionalLight.position.set( 0, 0, 1 ).normalize();
  // scene.add( directionalLight );

  /*** Texture Loading ***/
  var manager = new THREE.LoadingManager();
  manager.onProgress = function ( item, loaded, total ) {
    console.log( item, loaded, total );
  };
  var texture = new THREE.Texture();
  var loader = new THREE.ImageLoader( manager );

  //texturing
  loader.load( 'arrows_texture.gif', function ( image ) {
    texture.image = image;
    texture.needsUpdate = true;
    console.log('Loading Feedback arrows Extension: texture');
  } );

  //obj loading
  var loader = new THREE.OBJLoader( manager );

  // As soon as the OBJ has been loaded this function looks for a mesh
  // inside the data and applies the texture to it.
  loader.load( 'arrows.obj', function ( event ) {
    var object = event;
    object.traverse( function ( child ) {
      if ( child instanceof THREE.Mesh ) {
    	 console.log('Loading Feedback arrows Extension: preloading object and adding texture');
    	 child.material.color.setRGB(220, 220, 220);
         //child.material.map = texture;
      }
    } 
    );

    // upward scaling
    //object.scale = new THREE.Vector3( 25, 25, 25 );

    // You can change the position of the object, so that it is not
    // centered in the view and leaves some space for overlay text.
    object.position.y -= 2.5;
    console.log('Loading Feedback arrows Extension: adding object to scene');
    scene.add( object );
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

/*** The Loop ***/
function animate() {
  // This function calls itself on every frame. You can for example change
  // the objects rotation on every call to create a turntable animation.
  requestAnimationFrame( animate );

  // On every frame we need to calculate the new camera position
  // and have it look exactly at the center of our scene.
  camera.lookAt(scene.position);
  renderer.render(scene, camera);
}