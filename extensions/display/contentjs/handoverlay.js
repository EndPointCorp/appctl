/**
 *
 * @fileoverview HandOverlay provides a 3D overlay on which representations
 * of the users hands are rendered. Currently we limit the visualization to a
 * single hand, mostly to avoid issues with reflections causing ghost hands on
 * the leap.
 *
 */

var dumpUpdateToScreen = function(message) {
  var stringifiedMessage = JSON.stringify(message);
  var debugArea = document.getElementById('slinkydebug');
  if (!debugArea) {
    debugArea = document.createElement('div');
    debugArea.id = 'slinkydebug';
    debugArea.style.position = 'fixed';
    debugArea.style.bottom = '0px';
    debugArea.style.left = '0px';
    debugArea.style.height = '10%';
    debugArea.style.width = '100%';
    debugArea.style.zIndex = '99999';
    debugArea.style.backgroundColor = 'rgba(242, 242, 242, 1.0)';
    document.body.appendChild(debugArea);
  }
  debugArea.textContent = stringifiedMessage;
};

/**
 * @param {THREE.Scene} handOverlay
 * @param {object} leapInteractionBox
 * @param {number} handId
 * @constructor
 */
var Hand = function(handOverlay, leapInteractionBox, handId) {
  /** @type {THREE.Scene} */
  this.handOverlay_ = handOverlay;
  this.projector = new THREE.Projector();

    /** @type {THREE.ParticleSystem} */
  this.particleSystem;

  this.fingers = {};

  /** @type {number} */
  this.lastEventTimeMs = 0;

  this.visible = false;

  this.attributes;

  this.interactionMinY;
  this.interactionMaxY;

  this.leapOrigin = new THREE.Object3D();
  this.handOrigin = new THREE.Object3D();
  this.ring0;
  this.ring1;
  this.ring2;
  this.centerCircleFlat;
  this.centerDot;
  this.topCallout;
  this.topCalloutPanel = new THREE.Object3D();
  this.topCalloutPanel.position.x = 0.8;
  this.topCalloutPanel.position.y = 0.7;

  this.centerCircleGeomUniforms;
  this.geomUniforms;
  this.ring0GeomUniforms;
  this.dotUniforms;

  this.fadeInAnimation;

  this.setupGeometry_(leapInteractionBox);

  this.hudDiv = document.createElement('div');
  this.hudDiv.id = 'slinkyhud' + handId;
  this.hudDiv.style.position = 'absolute';
  this.hudDiv.style.height = '10%';
  this.hudDiv.style.width = '10%';
  this.hudDiv.style.zIndex = '99999';
  this.hudDiv.style.color = '#e3efff';
  this.hudDiv.style.backgroundColor = 'rgba(0, 0, 0, 0.0)';
};

/**
 * Given a world coordinate return a screen coordinate.
 */
Hand.prototype.toScreenCoords = function(worldPos) {
    var vector = this.projector.projectVector(worldPos.clone(),
        this.handOverlay_.camera);
    vector.x = (vector.x + 1) / 2 * window.innerWidth;
    vector.y = -(vector.y - 1) / 2 * window.innerHeight;
    return vector;
};

Hand.prototype.updateHudPosition = function(worldPos) {
  var screenPos = this.toScreenCoords(worldPos);
  this.hudDiv.style.left = screenPos.x + 'px';
  this.hudDiv.style.top = screenPos.y + 'px';
};

Hand.prototype.updateHudMessage = function(pose) {
  var handPose = pose.detail;

  var northing = handPose.lat >= 0 ? 'N' : 'S';
  var easting = handPose.lon >= 0 ? 'E' : 'W';

  var self = this;

  window.dispatchEvent(new CustomEvent('acmePopulationQuery', {
    detail: {
      latitude: handPose.lat,
      longitude: handPose.lon,
      callback: function(result) {
        self.hudDiv.innerHTML = 'Alt: ' + handPose.alt.toFixed(3) +
          'm<p>Lat: ' + Math.abs(handPose.lat).toFixed(3) + '&deg; ' + northing +
          '<p>Lng: ' + Math.abs(handPose.lon).toFixed(3) + '&deg; ' + easting +
          '<p>Pop: ' + Number(result.value - result.value % 100);
      }
    }
  }));
};

Hand.prototype.createRings_ = function(handOrigin) {
    // attributes
  var handAttributes = {
  };

  // TODO reuse these textures for every hand.
  // Load textures from base64 images to avoid cross domain issue. See
  // http://tp69.wordpress.com/2013/06/17/cors-bypass/
  var xRayColorTexture = new THREE.Texture(light_blue);
  xRayColorTexture.wrapS = THREE.ClampToEdgeWrapping;
  xRayColorTexture.wrapT = THREE.ClampToEdgeWrapping;
  xRayColorTexture.needsUpdate = true;

  // uniforms
  this.centerCircleGeomUniforms = {
     xRayDirection: {
       type: 'v3', value: new THREE.Vector3(0, 0, 1).normalize() },
     colorTexture: { type: 't', value: xRayColorTexture },
     alpha: { type: 'f', value: 0.1 },
     fade: { type: 'f', value: 0.0 }
  };

  // particle system material
  var centerCircleShader = new THREE.ShaderMaterial({

      uniforms: this.centerCircleGeomUniforms,
      attributes: handAttributes,
      vertexShader: document.getElementById('handvertexshader').textContent,
      fragmentShader: document.getElementById('handfragmentshader').textContent,
      transparent: true
  });


  // uniforms
  this.ring0GeomUniforms = {
     xRayDirection: {
       type: 'v3', value: new THREE.Vector3(0, 0, 1).normalize() },
     colorTexture: { type: 't', value: xRayColorTexture },
     alpha: { type: 'f', value: 0.35 },
     fade: { type: 'f', value: 0.0 }
  };


  // particle system material
  var ring0Shader = new THREE.ShaderMaterial({

      uniforms: this.ring0GeomUniforms,
      attributes: handAttributes,
      vertexShader: document.getElementById('handvertexshader').textContent,
      fragmentShader: document.getElementById('handfragmentshader').textContent,
      transparent: true
  });

    // uniforms
  this.dotUniforms = {
     xRayDirection: {
       type: 'v3', value: new THREE.Vector3(0, 0, 1).normalize() },
     colorTexture: { type: 't', value: xRayColorTexture },
     alpha: { type: 'f', value: 0.3 },
     fade: { type: 'f', value: 0.0 }
  };

  // particle system material
  var dotShader = new THREE.ShaderMaterial({

      uniforms: this.dotUniforms,
      attributes: handAttributes,
      vertexShader: document.getElementById('handvertexshader').textContent,
      fragmentShader: document.getElementById('handfragmentshader').textContent,
      transparent: true
  });


  // uniforms
  this.geomUniforms = {
     xRayDirection: {
       type: 'v3', value: new THREE.Vector3(0, 0, 1).normalize() },
     colorTexture: { type: 't', value: xRayColorTexture },
     alpha: { type: 'f', value: 0.4 },
     fade: { type: 'f', value: 0.0 }
  };

  // particle system material
  var handShader = new THREE.ShaderMaterial({

      uniforms: this.geomUniforms,
      attributes: handAttributes,
      vertexShader: document.getElementById('handvertexshader').textContent,
      fragmentShader: document.getElementById('handfragmentshader').textContent,
      transparent: true
  });

  this.ring0 = new THREE.Mesh(this.handOverlay_.ring0Geom, ring0Shader);
  this.ring0.name = 'ring0';
  this.handOrigin.add(this.ring0);
  this.ring1 = new THREE.Mesh(this.handOverlay_.ring1Geom, handShader);
  this.ring1.name = 'ring1';
  this.handOrigin.add(this.ring1);
  this.ring2 = new THREE.Mesh(this.handOverlay_.ring2Geom, handShader);
  this.ring2.name = 'ring2';
  this.handOrigin.add(this.ring2);
  this.centerCircleFlat = new THREE.Mesh(
      this.handOverlay_.centerCircleFlatGeom, centerCircleShader);
  this.centerCircleFlat.name = 'centerCircleFlat';
  this.handOrigin.add(this.centerCircleFlat);
  this.centerDot = new THREE.Mesh(
      this.handOverlay_.centerDotGeom, dotShader);
  this.centerDot.name = 'centerCircleDot';
  this.handOrigin.add(this.centerDot);

  this.topCallout = new THREE.Mesh(
      this.handOverlay_.topCalloutGeom, handShader);
  this.topCallout.name = 'topCallout';
  this.topCallout.add(this.topCalloutPanel);
  this.handOrigin.add(this.topCallout);

  this.handOpacity = 0.0;

  this.fadeInAnimation;
  this.fadeOutAnimation;
};

Hand.prototype.setupGeometry_ = function(leapInteractionBox) {
  this.createRings_(this.handOrigin);

  var WIDTH = window.innerWidth,
      HEIGHT = window.innerHeight;

  var camera = this.handOverlay_.camera;

  // TODO(paulby) plane intersections should we assume we always get
  // the same (and valid) interaction box from the leap.

  var vector = new THREE.Vector3(-1, -1, 1);
  this.projector.unprojectVector(vector, camera);
  var ray = new THREE.Raycaster(camera.position,
      vector.sub(camera.position).normalize());
  var intersects = ray.intersectObject(this.handOverlay_.handPlane);

  vector.set(1, 1, 1);
  this.projector.unprojectVector(vector, camera);
  ray = new THREE.Raycaster(camera.position,
      vector.sub(camera.position).normalize());
  var intersects2 = ray.intersectObject(this.handOverlay_.handPlane);

  if (intersects.length > 0 && intersects2.length > 0) {
    var pos = intersects[0].point;
    var pos2 = intersects2[0].point;

    // Screen size in the scene and the handPlane.
    var sceneWidth = pos2.x - pos.x;
    var sceneHeight = pos2.y - pos.y;

    var scaleX = sceneWidth / leapInteractionBox.width;
    var scaleY = sceneHeight / leapInteractionBox.height;

    this.interactionMinY =
        leapInteractionBox.center.y - leapInteractionBox.height / 2;
    this.interactionMaxY =
        leapInteractionBox.center.y + leapInteractionBox.height / 2;

    var scale = Math.min(scaleX, scaleY);

    // Map the leap physical coordinate system to gl
    var handCoordSystem = new THREE.Matrix4();
    handCoordSystem.setPosition(new THREE.Vector3(
        0, -sceneHeight / 2 - (this.interactionMinY * scaleY) , 0));
    handCoordSystem.scale(new THREE.Vector3(scaleX, scaleY, scale));
  }

  this.leapOrigin.applyMatrix(handCoordSystem);
  this.leapOrigin.updateMatrixWorld(false);
  this.leapOrigin.add(this.handOrigin);
};

/**
 * Called by Tween fade[In|Out]Animation on completion.
 */
Hand.prototype.fadeInOutAnimationComplete = function() {
  if (!this.visible) {
    this.handOverlay_.scene.remove(this.leapOrigin);
    document.body.removeChild(this.hudDiv);
  }
};

Hand.prototype.fadeInOutAnimationOnStart = function() {
  if (this.visible) {
    this.handOverlay_.scene.add(this.leapOrigin);
    document.body.appendChild(this.hudDiv);
  }
};

Hand.prototype.fadeInOutAnimationOnUpdate = function() {
  this.centerCircleGeomUniforms.fade.needsUpdate = true;
  this.geomUniforms.fade.needsUpdate = true;
  this.ring0GeomUniforms.fade.needsUpdate = true;
  this.dotUniforms.fade.needsUpdate = true;
};

Hand.prototype.setVisible = function(visible) {
  if (this.visible === visible) {
    return;
  }

  var self = this;
  this.visible = visible;

  // Fading in/out with a single animation does not seem to work, either
  // with reverse, or with modifying the start and value, so we have two
  // animations.
  if (!this.fadeInAnimation) {
    this.fadeInAnimation = TweenMax.to(
        [this.centerCircleGeomUniforms.fade,
        this.geomUniforms.fade,
        this.ring0GeomUniforms.fade,
        this.dotUniforms.fade,
        this.hudDiv.style.opacity],
        1,
        {
          startAt: {value: 0.0},
          value: 1.0,
          onStart: self.fadeInOutAnimationOnStart.bind(self),
          onComplete: self.fadeInOutAnimationComplete.bind(self),
          onUpdate: self.fadeInOutAnimationOnUpdate.bind(self),
          paused: true
        });
  }

  if (!this.fadeOutAnimation) {
    this.fadeOutAnimation = TweenMax.to(
        [this.centerCircleGeomUniforms.fade,
        this.geomUniforms.fade,
        this.ring0GeomUniforms.fade,
        this.dotUniforms.fade,
        this.hudDiv.style.opacity],
        1,
        {
          startAt: {value: this.handOpacity},
          value: 0.0,
          onStart: self.fadeInOutAnimationOnStart.bind(self),
          onComplete: self.fadeInOutAnimationComplete.bind(self),
          onUpdate: self.fadeInOutAnimationOnUpdate.bind(self),
          paused: true
        });
  }

  if (visible) {
    this.fadeInAnimation.restart();
  } else {
    this.fadeOutAnimation.restart();
  }
};


Hand.prototype.animate_ = function() {
  var currentTimeMs = Date.now();
  if (currentTimeMs - this.lastEventTimeMs > 300) {
    this.setVisible(false);
  }

  this.topCalloutPanel.updateMatrixWorld(false);
  var hudPos = new THREE.Vector3();
  hudPos.setFromMatrixPosition(this.topCalloutPanel.matrixWorld);
  var screenPos = this.toScreenCoords(hudPos);

  // Requests new geo data for location. Returned async to
  // processHandGeoLocationEvent
  getPointInfo(screenPos.x, screenPos.y);
  this.updateHudPosition(hudPos);
};

function toRadians_(deg) {
  return deg * Math.PI / 180;
}

Hand.prototype.setPositionFromLeap = function(leapData, currentTimeMs,
    currentCameraPose) {
  this.lastEventTimeMs = currentTimeMs;

  var palmpos = leapData.stabilized_palm_position;
  var palmPitch = leapData.direction.pitch + Math.PI;
  var palmRoll = -leapData.palm_normal.roll;
  var palmYaw = leapData.direction.yaw;

  var START_FADE = 150;  // Distance from plane where the fade starts
  var FADE_DISTANCE = 10;  // Distance taken to go from opaque to transparent.

  var ray = new THREE.Raycaster(
      new THREE.Vector3(palmpos.x, palmpos.y, palmpos.z),
      new THREE.Vector3(0, -1, 0));
  var intersects = ray.intersectObject(this.handOverlay_.handFadePlane);
  if (intersects[0] && intersects[0].distance < START_FADE) {

    var distance = intersects[0].distance;
    var startFadeOut = START_FADE;
    var endFadeOut = START_FADE - FADE_DISTANCE;
    if (distance < endFadeOut) {
      this.handOpacity = 0.0;
    } else {
      this.handOpacity =
          Math.abs((distance - endFadeOut) / (startFadeOut - endFadeOut));
    }
    this.centerCircleGeomUniforms.fade.value = this.handOpacity;
    this.geomUniforms.fade.value = this.handOpacity;
    this.ring0GeomUniforms.fade.value = this.handOpacity;
    this.dotUniforms.fade.value = this.handOpacity;
    this.hudDiv.style.opacity = this.handOpacity;
    this.fadeInOutAnimationOnUpdate();  // Inform shaders of data change.
  }


  this.handOrigin.position.set(palmpos.x, palmpos.y, palmpos.z);
  if (currentCameraPose) {
    this.handOrigin.rotation.set(
        toRadians_(90 - currentCameraPose.tilt), 0, 0);
  }
  this.handOrigin.scale.set(20, 20, 20);
  this.handOrigin.updateMatrixWorld(false);

  this.ring2.rotation.set(0, -palmYaw, 0);
};

/**
 * @constructor
 */
var HandOverlay = function() {
  /** @type {THREE.Scene} */
  this.scene;

  /** @type {THREE.WebGLRenderer} */
  this.renderer;

  /** @type {THREE.Camera} */
  this.camera;

  /** @type {Array.<Hand|null>} */
  this.hands_ = new Array();

  /** @type {THREE.Mesh} */
  this.handPlane;

  /** Plane at which hand will start to fade out. */
  this.handFadePlane;

  /** Geometry used for every finger */
  this.fingerGeom;

  this.palmGeom;

  this.initialized_ = false;

  this.ring0Geom;
  this.ring1Geom;
  this.ring2Geom;
  this.centerCircleFlatGeom;
  this.centerDotGeom;
  this.topCalloutGeom;

  this.currentHand = -1;
};


HandOverlay.prototype.setCurrentCameraPose = function(pose) {
  this.currentCameraPose = pose;
};


/**
 * Inject the GL shaders into the page.
 */
HandOverlay.prototype.injectShaders = function() {

  // Particle Shaders.
  var script0 = document.createElement('script');
  script0.type = 'x-shader/x-vertex';
  script0.id = 'particlevertexshader';
  script0.textContent =
    'attribute float alpha;' +
    'attribute vec4 color4;' +
    'varying vec4 vColor;' +

    'void main() {' +
    '    vColor = color4;' +
    '    vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );' +
    '    gl_PointSize = 1.0;' +
    '    gl_Position = projectionMatrix * mvPosition;' +
    '}';
  document.body.appendChild(script0);

  var script1 = document.createElement('script');
  script1.type = 'x-shader/x-fragment';
  script1.id = 'particlefragmentshader';
  script1.textContent =
    'varying vec4 vColor;' +

    'void main() {' +
    '    gl_FragColor = vec4( vColor );' +
    '}';
  document.body.appendChild(script1);

  // Hand Geometry Shaders.
  var handVertexScript = document.createElement('script');
  handVertexScript.type = 'x-shader/x-vertex';
  handVertexScript.id = 'handvertexshader';
  handVertexScript.textContent =
    'uniform vec3 xRayDirection;' +
    'uniform float alpha;' +
    'uniform float fade;' +
    'uniform sampler2D colorTexture;' +
    'varying vec4 vColor;' +

    'void main() {' +
    '    vec3 mvNormal = normalize(normalMatrix * normal);' +
    '    float dotP = (dot(xRayDirection, mvNormal) + 1.0) / 2.0;' +
    '    vec2 uv = vec2(0.5, dotP);' +
    '    vColor.rgb = texture2D(colorTexture, uv).rgb;' +
    '    vColor.a = alpha * fade;' +
    '    vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );' +
    '    gl_Position = projectionMatrix * mvPosition;' +
    '}';
  document.body.appendChild(handVertexScript);

  var handFragmentScript = document.createElement('script');
  handFragmentScript.type = 'x-shader/x-fragment';
  handFragmentScript.id = 'handfragmentshader';
  handFragmentScript.textContent =
    'varying vec4 vColor;' +

    'void main() {' +
    '    gl_FragColor = vec4( vColor );' +
    '}';
  document.body.appendChild(handFragmentScript);
};


/**
 * Initialize the 3D canvas and renderer.
 */
HandOverlay.prototype.init3js = function() {

  this.scene = new THREE.Scene();
  var WIDTH = window.innerWidth,
      HEIGHT = window.innerHeight;

  var handCanvas = document.getElementById('handCanvas');

  if (!handCanvas) {
    handCanvas = document.createElement('canvas');
    handCanvas.id = 'handCanvas';
    handCanvas.style.position = 'fixed';
    handCanvas.style.bottom = '0px';
    handCanvas.style.left = '0px';
    handCanvas.style.height = '100%';
    handCanvas.style.width = '100%';
    handCanvas.style.zIndex = '99999';
    handCanvas.style.backgroundColor = 'rgba(255, 255, 255, 0.0)';
    handCanvas.style.pointerEvents = 'none';
    document.body.appendChild(handCanvas);
  }

  var gl = handCanvas.getContext('webgl', {premultipliedAlpha: false});

  this.renderer = new THREE.WebGLRenderer({canvas: handCanvas, alpha: true});
  this.renderer.setSize(WIDTH, HEIGHT);

  this.renderer.setClearColor(new THREE.Color(0xffffff), 0);

  this.camera = new THREE.PerspectiveCamera(45, WIDTH / HEIGHT, 0.1, 200);
  this.camera.position.set(0, 0, 6);
  this.scene.add(this.camera);

  var self = this;
  window.addEventListener('resize', function() {
    var WIDTH = window.innerWidth,
        HEIGHT = window.innerHeight;
    self.renderer.setSize(WIDTH, HEIGHT);
    self.camera.aspect = WIDTH / HEIGHT;

    self.camera.updateProjectionMatrix();
  });

  this.handPlane = new THREE.Mesh(
      new THREE.PlaneGeometry(2000, 2000),
      new THREE.MeshBasicMaterial({
        color: 0x000000,
        wireframe: true }
  ));
  this.handPlane.lookAt(this.camera.position);
  this.handPlane.visible = false;
  this.handPlane.position = new THREE.Vector3(0, 0, 0);
  this.scene.add(this.handPlane);

  this.handFadePlane = new THREE.Mesh(
      new THREE.PlaneGeometry(2000, 2000),
      new THREE.MeshBasicMaterial({
        color: 0xff0000,
        wireframe: false }
  ));
  this.handFadePlane.lookAt(this.camera.position);
  this.handFadePlane.visible = false;
  this.handFadePlane.position = new THREE.Vector3(0, 1, 0);
  this.handFadePlane.rotation.x = toRadians_(-75);
  this.scene.add(this.handFadePlane);

  // Load the geometry for various parts of the hand.
  var loader0 = new THREE.JSONLoader();
  loader0.load(chrome.extension.getURL('models/ring_0.json'),
    function(geometry) {
       self.ring0Geom = geometry;
    });

  var loader1 = new THREE.JSONLoader();
  loader1.load(chrome.extension.getURL('models/ring_1.json'),
    function(geometry) {
       self.ring1Geom = geometry;
    });

  var loader2 = new THREE.JSONLoader();
  loader2.load(chrome.extension.getURL('models/ring_2.json'),
    function(geometry) {
       self.ring2Geom = geometry;
    });

  var loader3 = new THREE.JSONLoader();
  loader3.load(chrome.extension.getURL('models/center_circle_flat.json'),
    function(geometry) {
       self.centerCircleFlatGeom = geometry;
    });

  var loader4 = new THREE.JSONLoader();
  loader4.load(chrome.extension.getURL('models/center_dot.json'),
    function(geometry) {
       self.centerDotGeom = geometry;
    });

  var loader5 = new THREE.JSONLoader();
  loader5.load(chrome.extension.getURL('models/top_callout.json'),
    function(geometry) {
       self.topCalloutGeom = geometry;
    });

  this.injectShaders();
  initialized_ = true;
  self.animate_();
};

HandOverlay.prototype.processLeapMessage = function(leapMessage) {
  if (!initialized_) {
    return;
  }
  var leapInteractionBox = leapMessage.interaction_box;

  for (var i = 0; i < leapMessage.hands.length; i++) {
    var handData = leapMessage.hands[i];
    handOverlay.processHandMoved(handData, leapInteractionBox);
  }
};

/**
 * Handle geoLocationEvents, this is the geo location of the center of the
 * hand.
 */
HandOverlay.prototype.processHandGeoLocationEvent = function(pose) {
  if (this.currentHand == -1) {
    return;
  }

  var hand = this.hands_[this.currentHand];

  if (!hand) {
    return;
  }

  hand.updateHudMessage(pose);
};

/**
  * Called when the leap motion has new hand data.
  *
  * @typedef {{
  *   roll: number,
  *   pitch:number,
  *   yaw:number,
  *   x: number,
  *   y: number,
  *   z: number
  * }}
  * goog.LeapVector;
  *
  * @param {{
  *   palm_position:Object,
  *   palm_normal:goog.LeapVector,
  *   direction:goog.LeapVector,
  *   pointables:Array.<{
  *     tip_position: goog.LeapVector,
  *     stabilized_tip_position: goog.LeapVector,
  *     id: number,
  *     direction: goog.LeapVector
  *   }>
  * }} leapData
 */
HandOverlay.prototype.processHandMoved = function(
    leapData, leapInteractionBox) {
  if (this.currentHand == -1) {
    this.currentHand = leapData.id;
  }

  // We only support a single hand at the moment. currentHand will be reset when
  // the hand changes in animate_();
  if (this.currentHand != leapData.id) {
    return;
  }

  var hand = this.hands_[leapData.id];
  if (!hand) {
    hand = new Hand(this, leapInteractionBox, leapData.id);
    this.hands_[leapData.id] = hand;
  }

  if (!hand.visible) {
    hand.setVisible(true);
  }

  var timeMs = Date.now();
  hand.lastEventTimeMs = timeMs;

  hand.setPositionFromLeap(leapData, timeMs, this.currentCameraPose);
};

/**
 * Animate the handOverlay. Called by the browser renderer.
 * @private
 */
HandOverlay.prototype.animate_ = function() {
  var self = this;
  function f() {
    self.animate_();
  };
  requestAnimationFrame(f);
  if (this.scene == null || this.camera == null) {
    return;
  }

  var timeMs = Date.now();

  for (var key in this.hands_) {
    if (this.hands_.hasOwnProperty(key)) {
      var hand = this.hands_[key];
      if (hand.visible) {
        // Animate. If the hand is no longer receiving events then this call
        // will set it to invisible.
        hand.animate_();
      } else if (timeMs - hand.lastEventTimeMs < 750) {
        // Delete hands which are not visible and have received no events
        // for 10 seconds.
        this.currentHand = -1;
        delete this.hands_[key];
        delete hand;
      }
    }
  }
  this.renderer.render(this.scene, this.camera);
};

