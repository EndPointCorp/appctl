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
  messageType: 'geometry_msgs/Twist',
  throttle_rate: 10  
});

var arrowObjPosition = [0,0,0,0,0,0]; 

// Subscribe to spacenav topic + ROS messages rate limiting
console.log('Loading Feedback arrows Extension: subscribing to spacenav topic');

feedbackArrowsSpacenavListener.subscribe(function(msg){
  this.msg = msg;
  if (
	  this.msg.linear.x == 0
      && this.msg.linear.y == 0
      && this.msg.linear.z == 0
      && this.msg.angular.x == 0
      && this.msg.angular.y == 0
      && this.msg.angular.z == 0
      ) 
  {

        return;
   }

   arrowObjPosition[4] += 0.1;                                                                
   arrowObjPosition[5] += 0.1; 
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
	  
	container = document.createElement( 'div' );
	container.id = 'feedback_arrows';
	document.body.appendChild( container );

	camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 1, 2000 );
	camera.position.x = 33;
	camera.position.y = 33;
	camera.position.z = 33;

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
		this.arrowObj = object;
		
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
	renderer.setClearColor( 0x000000, 0);
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

    if (arrowObj) {
        arrowObj.position.x = arrowObjPosition[0];
        arrowObj.position.y = arrowObjPosition[1];
        arrowObj.position.z = arrowObjPosition[2];
        arrowObj.rotation.x = arrowObjPosition[3];
        arrowObj.rotation.y = arrowObjPosition[4];
        arrowObj.rotation.z = arrowObjPosition[5];
        } 

	camera.position.x += ( mouseX - camera.position.x ) * .05;
	camera.position.y += ( - mouseY - camera.position.y ) * .05;
	camera.lookAt( scene.position );
	renderer.render( scene, camera );

}
