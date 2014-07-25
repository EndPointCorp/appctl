/**
 *
 * @fileoverview HandOverlay provides a 3D overlay on which representations
 * of the users hands are rendered. Currently we limit the visualization to a
 * single hand, mostly to avoid issues with reflections causing ghost hands on
 * the leap.
 *
 */

var EARTH_RADIUS = 6371; // kilometers

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

  this.maxOverallOpacity = 0.35;

  this.attributes;

  this.interactionMinY;
  this.interactionMaxY;

  this.leapOrigin = new THREE.Object3D();
  this.handOrigin = new THREE.Object3D();
  this.calloutOrigin = new THREE.Object3D();
  this.ring0;
  this.ring1;
  this.ring2;
  this.centerCircleFlat;
  this.centerDot;
  this.topCallout;
  this.popCallout;
  this.compassRose;
  this.topCalloutPanel = new THREE.Object3D();
  this.popCalloutPanel = new THREE.Object3D();
  this.topCalloutPos = new THREE.Vector3();
  this.popCalloutPos = new THREE.Vector3();

  this.centerCircleGeomUniforms;
  this.geomUniforms;
  this.ring0GeomUniforms;
  this.dotUniforms;

  this.fadeInAnimation;

  this.setupGeometry_(leapInteractionBox);

  this.hudSpanAltId = 'hudAlt' + handId;
  this.hudSpanLatId = 'hudLat' + handId;
  this.hudSpanLngId = 'hudLng' + handId;
  this.hudSpanNorthingId = 'hudNorthing' + handId;
  this.hudSpanEastingId = 'hudEasting' + handId;

  this.hudDiv = document.createElement('div');
  this.hudDiv.id = 'slinkyhud' + handId;
  this.hudDiv.style.position = 'absolute';
  this.hudDiv.style.height = 'auto';
  this.hudDiv.style.width = 'auto';
  this.hudDiv.style.whiteSpace = 'nowrap';
  this.hudDiv.style.zIndex = '99999';
  this.hudDiv.style.fontFamily = 'Arimo, sans-serif';
  this.hudDiv.style.fontSize = '166%';
  this.hudDiv.style.color = '#e3efff';
  this.hudDiv.style.backgroundColor = 'rgba(0, 0, 0, 0.0)';
  this.hudDiv.innerHTML = '<p>Alt: <span id="' + this.hudSpanAltId + '"></span>m</p>' +
    '<p>Lat: <span id="' + this.hudSpanLatId + '"></span>&deg; <span id="' + this.hudSpanNorthingId + '"></span></p>' +
    '<p>Lng: <span id="' + this.hudSpanLngId + '"></span>&deg; <span id="' + this.hudSpanEastingId + '"></span></p>';


  this.popDiv = document.createElement('div');
  this.popDiv.id = 'slinkypop' + handId;
  this.popDiv.style.position = 'absolute';
  this.popDiv.style.height = 'auto';
  this.popDiv.style.width = '20%';
  this.popDiv.style.whiteSpace = 'nowrap';
  this.popDiv.style.zIndex = '99999';
  this.popDiv.style.fontFamily = 'Arimo, sans-serif';
  this.popDiv.style.fontSize = '166%';
  this.popDiv.style.textAlign = 'right';
  this.popDiv.style.color = '#e3efff';
  this.popDiv.style.backgroundColor = 'rgba(0, 0, 0, 0.0)';

  this.nextElevationRequest = 0;
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

Hand.prototype.updateHudPosition = function(div, worldPos) {
  var screenPos = this.toScreenCoords(worldPos);
  div.style.right = window.innerWidth - screenPos.x + 'px';
  div.style.top = screenPos.y + 'px';
};

Hand.prototype.updateHudMessage = function(pose) {
  var handPose = pose.detail;

  var northing = handPose.lat >= 0 ? 'N' : 'S';
  var easting = handPose.lon >= 0 ? 'E' : 'W';

  // TODO(mv): cache these lookups
  var hudSpanAlt = document.getElementById(this.hudSpanAltId);
  var hudSpanLat = document.getElementById(this.hudSpanLatId);
  var hudSpanLng = document.getElementById(this.hudSpanLngId);
  var hudSpanNorthing = document.getElementById(this.hudSpanNorthingId);
  var hudSpanEasting = document.getElementById(this.hudSpanEastingId);

  hudSpanLat.innerHTML = Math.abs(handPose.lat).toFixed(3);
  hudSpanLng.innerHTML = Math.abs(handPose.lon).toFixed(3);
  hudSpanNorthing.innerHTML = northing;
  hudSpanEasting.innerHTML = easting;

  var self = this;

  // async population data request from geodata ros service
  window.dispatchEvent(new CustomEvent('acmePopulationQuery', {
    detail: {
      latitude: handPose.lat,
      longitude: handPose.lon,
      radius: this.dataRadius,
      callback: function(result) {
        self.popDiv.innerHTML = '<p><img src="' + chrome.extension.getURL('models/population_icon.png') + '" />: ' + Number(result.value - result.value % 100);
      }
    }
  }));

  // async elevation request from Google elevation api

  // limit elevation request rate
  var ELEVATION_REQ_RATE = 5; // Hz
  var now = Date.now();
  if (now < this.nextElevationRequest) {
    return;
  }
  this.nextElevationRequest = now + 1000 / ELEVATION_REQ_RATE;

  // TODO(mv): API key
  var url = 'https://maps.googleapis.com/maps/api/elevation/json?locations={loc}';
  //var url = 'http://maps.googleapis.com/maps/api/elevation/json?loations={loc}&key={key}';
  url = url.replace('{loc}', [handPose.lat, handPose.lon].join(','));
  //url = url.replace('{key}', API_KEY);

  var elevationRequest = new XMLHttpRequest();
  elevationRequest.overrideMimeType('application/json');
  elevationRequest.open('GET', url, true);
  elevationRequest.onload = function() {
    if (elevationRequest.status != 200) {
      console.error('elevation request returned', elevationRequest.status);
      return;
    }
    var response = JSON.parse(elevationRequest.responseText);
    if (response.status == 'OK') {
      var elevation = response.results[0].elevation;
      hudSpanAlt.innerHTML = elevation.toFixed(3);
    } else {
      console.error('elevation request status:', response.status);
    }
  };
  elevationRequest.send();
};

Hand.prototype.updateHudCompass = function(pose) {
  var handPose = pose.detail;

  // rotate compass rose by rotating its parent
  // TODO(mv): inaccurate when going across poles, prime meridian from camera
  var currentCameraPose = this.handOverlay_.currentCameraPose;
  var cameraHdg = currentCameraPose.heading;
  var cameraLon = currentCameraPose.lon;
  var handLon = handPose.lon;
  var handLat = handPose.lat;
  this.centerDot.rotation.set(
    0,
    toRadians_(cameraHdg - (cameraLon - handLon) * (handLat / 90)),
    0
  );
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
     alpha: { type: 'f', value: this.maxOverallOpacity * 0.33 },
     fade: { type: 'f', value: 0.0 }
  };

  // particle system material
  var centerCircleShader = new THREE.ShaderMaterial({

      uniforms: this.centerCircleGeomUniforms,
      attributes: handAttributes,
      vertexShader: document.getElementById('xrayvertexshader').textContent,
      fragmentShader: document.getElementById('xrayfragmentshader').textContent,
      transparent: true
  });


  // uniforms
  this.ring0GeomUniforms = {
     xRayDirection: {
       type: 'v3', value: new THREE.Vector3(0, 0, 1).normalize() },
     colorTexture: { type: 't', value: xRayColorTexture },
     alpha: { type: 'f', value: this.maxOverallOpacity * 0.66 },
     fade: { type: 'f', value: 0.0 }
  };


  // particle system material
  var ring0Shader = new THREE.ShaderMaterial({

      uniforms: this.ring0GeomUniforms,
      attributes: handAttributes,
      vertexShader: document.getElementById('xrayvertexshader').textContent,
      fragmentShader: document.getElementById('xrayfragmentshader').textContent,
      transparent: true
  });

    // uniforms
  this.dotUniforms = {
     xRayDirection: {
       type: 'v3', value: new THREE.Vector3(0, 0, 1).normalize() },
     colorTexture: { type: 't', value: xRayColorTexture },
     alpha: { type: 'f', value: this.maxOverallOpacity * 0.66 },
     fade: { type: 'f', value: 0.0 }
  };

  // particle system material
  var dotShader = new THREE.ShaderMaterial({

      uniforms: this.dotUniforms,
      attributes: handAttributes,
      vertexShader: document.getElementById('xrayvertexshader').textContent,
      fragmentShader: document.getElementById('xrayfragmentshader').textContent,
      transparent: true
  });


  // uniforms
  this.geomUniforms = {
     xRayDirection: {
       type: 'v3', value: new THREE.Vector3(0, 0, 1).normalize() },
     colorTexture: { type: 't', value: xRayColorTexture },
     alpha: { type: 'f', value: this.maxOverallOpacity },
     fade: { type: 'f', value: 0.0 }
  };

  // particle system material
  var handShader = new THREE.ShaderMaterial({

      uniforms: this.geomUniforms,
      attributes: handAttributes,
      vertexShader: document.getElementById('xrayvertexshader').textContent,
      fragmentShader: document.getElementById('xrayfragmentshader').textContent,
      transparent: true,
      depthTest: false
  });

  // compass rose vertex-colored uniforms
  this.compassRoseUniforms = {
     alpha: { type: 'f', value: this.maxOverallOpacity },
     fade: { type: 'f', value: 0.0 }
  };

  // compass rose vertex-colored material
  var compassRoseShader = new THREE.ShaderMaterial({

      vertexColors: THREE.VertexColors,
      uniforms: this.compassRoseUniforms,
      attributes: handAttributes,
      vertexShader: document.getElementById('vcolorvertexshader').textContent,
      fragmentShader: document.getElementById('vcolorfragmentshader').textContent,
      transparent: true,
      depthTest: false
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

  this.topCalloutPanel = new THREE.Mesh(
      new THREE.SphereGeometry(8, 8),
      new THREE.MeshBasicMaterial({
        color: 0x00FF00,
        wireframe: true }
      )
  );
  this.topCalloutPanel.scale.set(0.01, 0.01, 0.01);
  this.topCalloutPanel.position.x = 1.6;
  this.topCalloutPanel.position.y = 0.7;
  this.topCalloutPanel.visible = false;

  this.topCallout = new THREE.Mesh(
      this.handOverlay_.topCalloutGeom, handShader);
  this.topCallout.name = 'topCallout';
  this.topCallout.add(this.topCalloutPanel);
  this.topCallout.scale.set(1.1, 1.1, 1.1);
  this.topCallout.visible = true;
  this.calloutOrigin.add(this.topCallout);

  this.popCalloutPanel = new THREE.Mesh(
      new THREE.SphereGeometry(8, 8), 
      new THREE.MeshBasicMaterial({
        color: 0x00FF00,
        wireframe: true }
      )
  );
  this.popCalloutPanel.scale.set(0.01, 0.01, 0.01);
  this.popCalloutPanel.position.x = 2.0;
  this.popCalloutPanel.position.y = -0.05;
  this.popCalloutPanel.visible = false;

  this.popCallout = new THREE.Mesh(
      this.handOverlay_.popCalloutGeom, 
      handShader
  );
  this.popCallout.name = 'popCallout';
  this.popCallout.add(this.popCalloutPanel);
  this.popCallout.visible = true;
  this.calloutOrigin.add(this.popCallout);

  this.compassRose = new THREE.Mesh(
    this.handOverlay_.compassRoseGeom,
    compassRoseShader
  );
  this.compassRose.name = 'compassRose';
  this.compassRose.position.set(0, 0, -1.25);
  this.compassRose.rotation.set(Math.PI / 2, 0, 0);
  this.centerDot.add(this.compassRose);

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

  /*
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
  */
};

/**
 * Called by Tween fade[In|Out]Animation on completion.
 */
Hand.prototype.fadeInOutAnimationComplete = function() {
  if (!this.visible) {
    this.handOverlay_.scene.remove(this.handOrigin);
    this.handOverlay_.scene.remove(this.calloutOrigin);
    document.body.removeChild(this.hudDiv);
    document.body.removeChild(this.popDiv);
  }
};

Hand.prototype.fadeInOutAnimationOnStart = function() {
  if (this.visible) {
    this.handOverlay_.scene.add(this.handOrigin);
    this.handOverlay_.scene.add(this.calloutOrigin);
    document.body.appendChild(this.hudDiv);
    document.body.appendChild(this.popDiv);
  }
};

Hand.prototype.fadeInOutAnimationOnUpdate = function() {
  this.centerCircleGeomUniforms.fade.needsUpdate = true;
  this.geomUniforms.fade.needsUpdate = true;
  this.ring0GeomUniforms.fade.needsUpdate = true;
  this.dotUniforms.fade.needsUpdate = true;
  this.compassRoseUniforms.fade.needsUpdate = true;
};

Hand.prototype.setVisible = function(visible) {
  if (this.visible === visible) {
    return;
  }

  this.visible = visible;

  if (visible) {
    this.fadeInOutAnimationOnStart();
  } else {
    this.fadeInOutAnimationComplete();
  }
};

Hand.prototype.setOpacity = function(opacity) {
  this.centerCircleGeomUniforms.fade.value = this.handOpacity;
  this.geomUniforms.fade.value = this.handOpacity;
  this.ring0GeomUniforms.fade.value = this.handOpacity;
  this.dotUniforms.fade.value = this.handOpacity;
  this.compassRoseUniforms.fade.value = this.handOpacity;
  this.hudDiv.style.opacity = this.handOpacity * this.maxOverallOpacity * 2;
  this.popDiv.style.opacity = this.handOpacity * this.maxOverallOpacity * 2;
  this.fadeInOutAnimationOnUpdate();  // Inform shaders of data change.
};

Hand.prototype.animate_ = function() {
  var currentTimeMs = Date.now();
  if (currentTimeMs - this.lastEventTimeMs > 300) {
    this.setVisible(false);
  }

  if (this.hudPos) {
    var screenPos = this.toScreenCoords(this.hudPos);

    // Requests new geo data for location. Returned async to
    // processHandGeoLocationEvent
    getPointInfo(screenPos.x, screenPos.y);

    // move the HUD data to the proper spot
    this.updateHudPosition(this.hudDiv, this.topCalloutPos);
    this.updateHudPosition(this.popDiv, this.popCalloutPos);
  }
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

  var palmHeight = palmpos.y;
  var palmSlide = palmpos.x;
  // use more responsive un-stabilized palm position for z
  var palmDepth = leapData.palm_position.z;


  var camera = this.handOverlay_.camera;

  /* window mode
  var leapVector = new THREE.Vector3(
    (palmSlide / 100),
    (palmHeight / 100 - 2.0),
    1.0
  );
  */

  var leapVector = new THREE.Vector3(
    (palmSlide / 100),
    (-palmDepth / 100),
    1.0
  );

  // limits of normalized height
  var HEIGHT_MIN = 0;
  var HEIGHT_MAX = 300;

  // normalized height where fade starts
  var FADE_LOW = HEIGHT_MIN + 50;
  var FADE_HIGH = HEIGHT_MAX - 50;

  // normal angle where fade starts on edges
  var FADE_EDGE = 0.5;

  // absolute height limits
  var MIN_ABS_HEIGHT = 120;
  var MAX_ABS_HEIGHT = MIN_ABS_HEIGHT + HEIGHT_MAX;

  // TODO(mv): better logic for bypassing ray intersect when out of bounds
  var intersects = [];
  if (palmHeight > MIN_ABS_HEIGHT && palmHeight <= MAX_ABS_HEIGHT) {
    var ray = this.projector.pickingRay(leapVector, camera);
    intersects = ray.intersectObject(this.handOverlay_.globeSphere);
  }

  if (intersects.length == 0) {
    this.handOpacity = 0.0;
    this.hudPos = null;
  } else {
    // normalize and limit height
    palmHeight = Math.min(Math.max(palmHeight - MIN_ABS_HEIGHT, HEIGHT_MIN), HEIGHT_MAX);

    // height limits for data radius
    var palmDataHeight = Math.min(Math.max(palmHeight, FADE_LOW), FADE_HIGH);

    this.hudPos = intersects[0].point;
    var distance = intersects[0].distance;
    var interNormal = intersects[0].face.normal;

    // push fade to the edge
    var normalAngle = Math.max(0,
      Math.abs(interNormal.x) +
      Math.abs(interNormal.y +
      toRadians_(currentCameraPose.tilt / 2)) -
      FADE_EDGE
    );
    var normalFade = Math.sqrt(Math.max(0, 1.0 - (normalAngle) * (1 / FADE_EDGE)));

    // height fade coefficient
    var lowFade = Math.min(Math.max((palmHeight - FADE_LOW) * (1 / FADE_LOW), 0), 1);
    var highFade = Math.min(Math.max((FADE_HIGH - palmHeight) * (1 / (HEIGHT_MAX - FADE_HIGH)), 0), 1);
    var heightFade = Math.min(lowFade, highFade);

    this.handOpacity = Math.min(Math.max(normalFade * heightFade, 0), 1);

    this.handOrigin.position.set(this.hudPos.x, this.hudPos.y, this.hudPos.z);
    this.calloutOrigin.position.set(this.hudPos.x, this.hudPos.y, this.hudPos.z);

    if (currentCameraPose) {
      this.handOrigin.rotation.set(
          toRadians_(90 - currentCameraPose.tilt) - interNormal.y,
          0,
          -interNormal.x
      );
    }

    var distanceMod = distance / 12;
    var ringScale = distanceMod + (palmDataHeight / (FADE_HIGH - FADE_LOW)) * distanceMod;
    var calloutScale = distanceMod * 1.1;

    this.handOrigin.scale.set(ringScale, ringScale, ringScale);
    this.calloutOrigin.scale.set(calloutScale, calloutScale, calloutScale);
    this.calloutOrigin.updateMatrixWorld(false);

    this.ring2.rotation.set(0, -palmYaw, 0);

    this.ring0.geometry.computeBoundingSphere();
    this.dataRadius = this.ring1.geometry.boundingSphere.radius * ringScale;

    this.topCalloutPos.setFromMatrixPosition(this.topCalloutPanel.matrixWorld);

    this.popCalloutPos.setFromMatrixPosition(this.popCalloutPanel.matrixWorld);
  }

  this.setOpacity(this.handOpacity);
  this.setVisible(this.handOpacity > 0.0);
};

/**
 * @constructor
 */
var HandOverlay = function(glEnvironment) {
  this.glEnvironment = glEnvironment;

  /** @type {THREE.Scene} */
  this.scene = this.glEnvironment.scene;

  /** @type {THREE.WebGLRenderer} */
  this.renderer = this.glEnvironment.renderer;

  /** @type {THREE.Camera} */
  this.camera = this.glEnvironment.camera;

  /** @type {Array.<Hand|null>} */
  this.hands_ = new Array();

  /** @type {THREE.Mesh} */
  this.handPlane;

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
  this.popCalloutGeom;
  this.compassRoseGeom;

  this.currentHand = -1;
};


HandOverlay.prototype.setCurrentCameraPose = function(pose) {
  this.currentCameraPose = pose;

  // move the virtual globe on top of the Tactile globe
  // alitude is converted to km
  var distanceToEarthCenter = -(pose.alt / 1000 + EARTH_RADIUS);
  var tilt = pose.tilt;
  var y = Math.sin(toRadians_(tilt)) * distanceToEarthCenter;
  var z = Math.cos(toRadians_(tilt)) * distanceToEarthCenter;
  this.globeSphere.position.set(0, y, z);
  this.globeSphere.lookAt(this.camera.position);

  // fix camera fov for zoom level to match Tactile
  // 60 at super high altitude, transitioning down to 35 from 17500km to 9500km
  // TODO(mv): make the transition match Tactile's curve more closely
  var fov = 60;
  if (pose.alt < 9500000) {
    fov = 35;
  } else if (pose.alt < 17500000) {
    fov = 35 + 25 * ((pose.alt - 9500000) / (17500000 - 9500000));
  }
  if (fov != this.camera.fov) {
    this.camera.fov = fov;
    this.camera.updateProjectionMatrix();
  }
};

/**
 * Initialize the 3D canvas and renderer.
 */
HandOverlay.prototype.init3js = function() {
  var self = this;

  /*
  this.scene = new THREE.Scene();
  */
  var WIDTH = window.innerWidth,
      HEIGHT = window.innerHeight;

  /*
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
  */

  /*
  this.renderer = new THREE.WebGLRenderer({canvas: handCanvas, alpha: true});
  this.renderer.setSize(WIDTH, HEIGHT);

  this.renderer.setClearColor(new THREE.Color(0xffffff), 0);
  */

  /*
  this.camera = new THREE.PerspectiveCamera(45, WIDTH / HEIGHT, 0.1, EARTH_RADIUS * 10);
  this.camera.position.set(0, 0, 6);
  this.scene.add(this.camera);
  */

  /*
  var self = this;
  window.addEventListener('resize', function() {
    var WIDTH = window.innerWidth,
        HEIGHT = window.innerHeight;
    self.renderer.setSize(WIDTH, HEIGHT);
    self.camera.aspect = WIDTH / HEIGHT;

    self.camera.updateProjectionMatrix();
  });
  */

  /*
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
  */

  this.globeSphere = new THREE.Mesh(
    new THREE.SphereGeometry(EARTH_RADIUS, 128, 128, 0, Math.PI),
    new THREE.MeshBasicMaterial({
      color: 0x000000,
      wireframe: true
    })
  );
  this.globeSphere.lookAt(this.camera.position);
  this.globeSphere.visible = false;
  this.globeSphere.position = new THREE.Vector3(0, 0, EARTH_RADIUS * 5);
  this.scene.add(this.globeSphere);

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

  var loader6 = new THREE.JSONLoader();
  loader6.load(chrome.extension.getURL('models/pop_callout.json'),
    function(geometry) {
       // shift origin
       geometry.applyMatrix(new THREE.Matrix4().makeTranslation(1, 0, 0));
       self.popCalloutGeom = geometry;
    });

  this.compassRoseColor = new THREE.Color(0xFF0000);
  this.compassRoseGeom = this.createGeometry_(3, 0.1, this.compassRoseColor);

  initialized_ = true;

  function _animate() {
    self.animate_();
  }
  this.glEnvironment.addAnimation(_animate);
};

/**
 * Creates geometry with n sides.
 * To use vertex colors, material must have vertexColors: THREE.VertexColors
 * @see http://jsfiddle.net/Elephanter/mUah5/
 */
HandOverlay.prototype.createGeometry_ = function(n, circumradius, color) {

  var geometry = new THREE.Geometry(),
    vertices = [],
    faces = [],
    x;

  // Generate the vertices of the n-gon.
  for (x = 1; x <= n; x++) {
    geometry.vertices.push(new THREE.Vector3(
      circumradius * Math.sin((Math.PI / n) + (x * ((2 * Math.PI)/ n))),
      circumradius * Math.cos((Math.PI / n) + (x * ((2 * Math.PI)/ n))),
      0
    ));
    geometry.colors.push(color);
  }

  // Generate the faces of the n-gon.
  for (x = 0; x < n-2; x++) {
    var face = new THREE.Face3(0, x + 1, x + 2);
    face.vertexColors = [color, color, color];
    geometry.faces.push(face);
  }

  geometry.computeBoundingSphere();

  return geometry;
}

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
  hand.updateHudCompass(pose);
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

  var timeMs = Date.now();
  hand.lastEventTimeMs = timeMs;

  hand.setPositionFromLeap(leapData, timeMs, this.currentCameraPose);
};

/**
 * Animate the handOverlay. Called by the browser renderer.
 * @private
 */
HandOverlay.prototype.animate_ = function() {
  /*
  var self = this;
  function f() {
    self.animate_();
  };
  requestAnimationFrame(f);
  */
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
  /*
  this.renderer.render(this.scene, this.camera);
  */
};

