/**
 * Feedback arrows UX extension.
 * Shows two independent webgl models that should immitate spacenav movement.
 * Object should fade in and fade out on spacenav use.
 * @constructor
 * @param {SlinkyGLEnvironment} glEnvironment shared WebGL context
 */
var SpacenavFeedback = function(glEnvironment) {
  this.glEnvironment = glEnvironment;
  this.canvas = glEnvironment.canvas;
  this.renderer = glEnvironment.renderer;
  this.camera = glEnvironment.camera;
  this.scene = glEnvironment.scene;
  this.absOrigin = new THREE.Object3D();
  this.absOrigin.position.set(0, -3, -95);
  this.absOrigin.rotation.set(0.5, 0, 0);
  this.absOrigin.scale.set(1.2, 1.2, 1.2);

  this.innerOrigin = new THREE.Object3D();

  this.flapLeftOrigin = new THREE.Object3D();
  this.flapRightOrigin = new THREE.Object3D();

  this.spacenav_min = -350;
  this.spacenav_max = 350;

  this.arrows_min = -3;
  this.arrows_max = 3;

  this.arrowObj;
  this.ringObj;
  this.flapLeftObj;
  this.flapRightObj;

  this.arrowUniforms;
  this.ringUniforms;
  this.flapUniforms;

  this.arrowObjPosition = [0, 0, 0, 0, 0, 0];
  this.ringObjPosition = [0, 0, 0, 0, 0, 0];
  this.flapRotation = 0.0;

  this.arrowOpacity = 0.0;
  this.ringXOpacity = 0.0;
  this.ringZOpacity = 0.0;
  this.flapOpacity = 0.0;

  this.maxAbsoluteOpacity = 0.35;

  this.enabled = true;
};

/**
 * Sets the opacity of shader uniforms.
 * @param {object} uniforms
 * @param {float} opacity [0, 1]
 */
SpacenavFeedback.prototype.setOpacity = function(uniforms, opacity) {
  if (uniforms.fade.value != opacity) {
    uniforms.fade.value = opacity;
    uniforms.fade.needsUpdate = true;
  }
};

/**
 * Clamps a SpaceNav axis to the given range.
 * @param {float} num inbound axis value
 * @param {float} low
 * @param {float} high
 * @return {float} clamped value
 */
SpacenavFeedback.prototype.clampAxis = function(num, low, high) {
  return (num - this.spacenav_min) * (high - low) /
         (this.spacenav_max - this.spacenav_min) + low;
};

/**
 * Checks to see if a pose is in the zero gutter.
 * @static
 * @param {object} pose
 * @return {boolean} true if the pose is in the gutter
 */
SpacenavFeedback.isInGutter = function(pose) {
  return (
    pose.linear.x == 0 && pose.linear.y == 0 &&
    pose.linear.z == 0 && pose.angular.x == 0 &&
    pose.angular.y == 0 && pose.angular.z == 0
  );
};

/**
 * Zeroes out the visuals position and opacity.
 */
SpacenavFeedback.prototype.clearVisuals = function() {
  /*
   * - fade objects out - return to point 0
   */
  for (var i = 0; i < this.arrowObjPosition.length; i++) {
    this.arrowObjPosition[i] = 0;
    this.ringObjPosition[i] = 0;
  }
  // set opacity to 0
  if ((typeof (this.arrowObj) === 'undefined') &&
      (typeof (this.ringObj) === 'undefined')) {
    console.log('Initializing objects');
  } else {
    this.ringXOpacity = 0;
    this.ringZOpacity = 0;
    this.arrowOpacity = 0;
  }
};

/**
 * Updates desired position and opacity of visuals from a pose.
 * @param {object} pose
 */
SpacenavFeedback.prototype.updateVisuals = function(pose) {
  /***********************************************************************
   * - map spacenav values to threejs object coordinates - fade in - move
   * objects
   *
   * ObjPosition = [
   * 0 : "go front right",
   * 1 : "go up (z)",
   * 2 : "go front left",
   * 3 : "rotate over y axis",
   * 4 : "rotate over center",
   * 5 : "rotate over x axis"
   *
   *
   * rostopic echo /spacenav/twist: [
   * "push then pull" : "linear z -350 => +350",
   * "rotate from left to right" : "angular z: +350 => -350",
   * "move backward then forward" : "linear x -350=>+350",
   * "move left then right" : "linear y +350 => -350",
   * "lean left then lean right" : "angular x -350 => +350",
   * "lean forward then lean backward" : "angular y +350 => -350"
   * ]
   *
   **********************************************************************/

  // needed for arrow
  var linearX = this.clampAxis(pose.linear.x, -1, 1);
  var linearY = this.clampAxis(pose.linear.y, -1, 1);

  // needed for ring
  var amin = this.arrows_min;
  var amax = this.arrows_max;
  var linearZ = this.clampAxis(pose.linear.z, amin, amax);
  var angularX = this.clampAxis(pose.angular.x, amin, amax);
  var angularY = this.clampAxis(pose.angular.y, amin, amax);
  var angularZ = this.clampAxis(pose.angular.z, amin, amax);

  // make object transparency proportional to the values

  // add pretty curve possibly y=3x/(x+2)
  var ring_opacity = Math.max(
    Math.abs(linearZ), Math.abs(angularZ),
    Math.abs(angularX), Math.abs(angularY)
  ) / amax;

  this.flapRotation = this.clampAxis(pose.linear.z, -1, 1);

  if (Math.abs(ring_opacity) > this.arrows_max / 20) { // yes that's evil
    this.ringXOpacity = this.clampAxis(pose.angular.z, -1, 1);
    this.ringZOpacity = -this.clampAxis(pose.angular.y, -1, 1);
    // rotate (twist)
    this.ringObjPosition[4] = angularZ * 0.5;
    // lean forward and backward
    this.ringObjPosition[5] = 0;
    this.ringObjPosition[3] = angularY * -0.1;
  } else {
    this.ringXOpacity = 0.0;
    this.ringZOpacity = 0.0;
  }

  // let's rotate and show the direction arrow with little tresholding
  if ((Math.abs(linearY) > 0.2) || (Math.abs(linearX) > 0.2)) {

    var direction = Math.atan2(-linearY, -linearX);

    /*
    console.log("This is direction1:", direction,
        "computed out of (x,y)", this.pose.linear.y, "/",
        this.pose.linear.x);
    */

    this.arrowOpacity = Math.max(Math.abs(linearY), Math.abs(linearX));
    this.arrowObjPosition[4] = direction;
  } else {
    this.arrowOpacity = 0.0;
  }
};

/**
 * Processes an incoming SpaceNav message.
 * @param {object} msg incoming message from Ros
 */
SpacenavFeedback.prototype.processSpacenavMessage = function(msg) {
  /*
   * Two possible states: spacenav not being used (all zeros), else spacenav
   * being touched - we rotate our pretty objects
   */
  if (!this.enabled || SpacenavFeedback.isInGutter(msg)) {
    this.clearVisuals();
  } else {
    this.updateVisuals(msg);
  }
};

/**
 * Applies a color to all vertices and faces of the given geometry.
 * @static
 * @param {THREE.Geometry} geometry
 * @param {THREE.Color} color
 */
SpacenavFeedback.paintGeometry = function(geometry, color) {
  var numVertices = geometry.vertices.length;
  for (var i = 0; i < numVertices; i++) {
    geometry.colors[i] = color;
  }

  var faceIndices = ['a', 'b', 'c', 'd'];

  var numFaces = geometry.faces.length;
  for (var i = 0; i < numFaces; i++) {
    var face = geometry.faces[i];

    var numSides = (face instanceof THREE.Face3) ? 3 : 4;
    for (var j = 0; j < numSides; j++) {
      var vertexIndex = face[faceIndices[j]];
      face.vertexColors[j] = geometry.colors[vertexIndex];
    }
  }
};

/**
 * Initializes the feedback module.
 */
SpacenavFeedback.prototype.init = function() {
  this.scene.add(this.absOrigin);
  this.absOrigin.add(this.innerOrigin);

  this.arrowUniforms = {
    alpha: { type: 'f', value: this.maxAbsoluteOpacity },
    fade: { type: 'f', value: 0.0 },
    fadeRadius: { type: 'f', value: 0.0 }
  };

  this.ringUniforms = {
    alpha: { type: 'f', value: this.maxAbsoluteOpacity },
    xFade: { type: 'f', value: 0.0 },
    bothXFade: { type: 'f', value: 0.0 },
    zFade: { type: 'f', value: 0.0 },
    fadeRadius: { type: 'f', value: 0.0 }
  };

  this.flapLeftUniforms = {
    alpha: { type: 'f', value: this.maxAbsoluteOpacity },
    fade: { type: 'f', value: 0.0 }
  };

  this.flapRightUniforms = {
    alpha: { type: 'f', value: this.maxAbsoluteOpacity },
    fade: { type: 'f', value: 0.0 }
  };

  var zGradientVertexShader =
    document.getElementById('zgradientvertexshader').textContent;
  var zGradientFragmentShader =
    document.getElementById('zgradientfragmentshader').textContent;

  var xzGradientVertexShader =
    document.getElementById('xzgradientvertexshader').textContent;
  var xzGradientFragmentShader =
    document.getElementById('xzgradientfragmentshader').textContent;

  var vColorVertexShader =
    document.getElementById('vcolorvertexshader').textContent;
  var vColorFragmentShader =
    document.getElementById('vcolorfragmentshader').textContent;

  var arrowShader = new THREE.ShaderMaterial({
    vertexColors: THREE.VertexColors,
    uniforms: this.arrowUniforms,
    vertexShader: zGradientVertexShader,
    fragmentShader: zGradientFragmentShader,
    transparent: true,
    depthTest: false
  });

  var ringShader = new THREE.ShaderMaterial({
    vertexColors: THREE.VertexColors,
    uniforms: this.ringUniforms,
    vertexShader: xzGradientVertexShader,
    fragmentShader: xzGradientFragmentShader,
    transparent: true,
    depthTest: false
  });

  var flapLeftShader = new THREE.ShaderMaterial({
    vertexColors: THREE.VertexColors,
    uniforms: this.flapLeftUniforms,
    vertexShader: vColorVertexShader,
    fragmentShader: vColorFragmentShader,
    transparent: true,
    depthTest: false
  });

  var flapRightShader = new THREE.ShaderMaterial({
    vertexColors: THREE.VertexColors,
    uniforms: this.flapRightUniforms,
    vertexShader: vColorVertexShader,
    fragmentShader: vColorFragmentShader,
    transparent: true,
    depthTest: false
  });

  var arrowUrl = chrome.extension.getURL('models/arrow.json');
  var arrowLoader = new THREE.JSONLoader();
  arrowLoader.load(arrowUrl, function(geometry) {
    SpacenavFeedback.paintGeometry(geometry, new THREE.Color(0xFFFFFF));
    this.arrowObj = new THREE.Mesh(
      geometry,
      arrowShader
    );
    geometry.computeBoundingSphere();
    this.arrowUniforms.fadeRadius.value = geometry.boundingSphere.radius;
    this.arrowUniforms.fadeRadius.needsUpdate = true;
    this.innerOrigin.add(this.arrowObj);
  }.bind(this));

  var ringUrl = chrome.extension.getURL('models/ring.json');
  var ringLoader = new THREE.JSONLoader();
  ringLoader.load(ringUrl, function(geometry) {
    SpacenavFeedback.paintGeometry(geometry, new THREE.Color(0xFFFFFF));
    this.ringObj = new THREE.Mesh(
      geometry,
      ringShader
    );
    geometry.computeBoundingSphere();

    // attach flaps to the edges of the ring
    this.flapLeftOrigin.position.set(
      -geometry.boundingSphere.radius + 0.1,
      0,
      0
    );
    this.flapRightOrigin.position.set(
      geometry.boundingSphere.radius - 0.1,
      0,
      0
    );

    this.ringUniforms.fadeRadius.value = geometry.boundingSphere.radius;
    this.ringUniforms.fadeRadius.needsUpdate = true;
    this.innerOrigin.add(this.ringObj);
    this.ringObj.add(this.flapLeftOrigin);
    this.ringObj.add(this.flapRightOrigin);
  }.bind(this));

  var flapUrl = chrome.extension.getURL('models/flap.json');
  var flapLoader = new THREE.JSONLoader();
  flapLoader.load(flapUrl, function(geometry) {
    SpacenavFeedback.paintGeometry(geometry, new THREE.Color(0xFFFFFF));
    this.flapLeftObj = new THREE.Mesh(
      geometry,
      flapLeftShader
    );
    this.flapRightObj = new THREE.Mesh(
      geometry,
      flapRightShader
    );

    this.flapLeftObj.rotation.set(0, -Math.PI, 0);

    this.flapLeftOrigin.add(this.flapLeftObj);
    this.flapRightOrigin.add(this.flapRightObj);
    geometry.computeBoundingSphere();
  }.bind(this));

  var self = this;
  function _animate() {
    self.animate();
  }
  this.glEnvironment.addAnimation(_animate);
};

/**
 * Updates SpaceNav visuals in the scene.
 */
SpacenavFeedback.prototype.animate = function() {
  if (typeof (this.arrowObj) === 'undefined') {
    console.log('Still initializing arrowObj');
  } else {
    this.arrowObj.position.x = this.arrowObjPosition[0];
    this.arrowObj.position.y = this.arrowObjPosition[1];
    this.arrowObj.position.z = this.arrowObjPosition[2];
    this.arrowObj.rotation.x = this.arrowObjPosition[3];
    this.arrowObj.rotation.y = this.arrowObjPosition[4];
    this.arrowObj.rotation.z = this.arrowObjPosition[5];
    this.setOpacity(this.arrowUniforms, this.arrowOpacity);
  }

  if (typeof (this.ringObj) === 'undefined') {
    console.log('Still initializing ringObj');
  } else {
    this.innerOrigin.position.x = this.ringObjPosition[0];
    this.innerOrigin.position.y = this.ringObjPosition[1];
    this.innerOrigin.position.z = this.ringObjPosition[2];
    // tilt rotation
    this.innerOrigin.rotation.x = this.ringObjPosition[3];
    // twist rotation
    this.ringObj.rotation.y = this.ringObjPosition[4] *
      Math.pow(1.66, Math.abs(this.ringObjPosition[4]));
    // roll rotation
    this.innerOrigin.rotation.z = this.ringObjPosition[5];

    this.ringUniforms.xFade.value = this.ringXOpacity * 0.5;

    this.ringUniforms.bothXFade.value = Math.abs(this.flapRotation * 0.5) +
      Math.abs(this.ringXOpacity * 0.5);

    this.ringUniforms.zFade.value = this.ringZOpacity *
      (1 - this.ringUniforms.bothXFade.value);

    this.ringUniforms.xFade.needsUpdate = true;
    this.ringUniforms.zFade.needsUpdate = true;
    this.ringUniforms.bothXFade.needsUpdate = true;
  }

  if (typeof this.flapLeftObj === 'undefined' ||
      typeof this.flapRightObj === 'undefined') {

    console.log('Still initializing flap objects');
  } else {
    this.flapRightObj.rotation.z = this.flapRotation;
    this.flapLeftObj.rotation.z = this.flapRotation;

    // TODO(mv): untangle this
    var leftVal = Math.min(1, Math.max(0, Math.abs(this.flapRotation) - this.ringXOpacity) + Math.abs(this.ringXOpacity) * 0.2);
    var rightVal = Math.min(1, Math.max(0, Math.abs(this.flapRotation) + this.ringXOpacity) + Math.abs(this.ringXOpacity) * 0.2);

    this.setOpacity(
      this.flapLeftUniforms,
      leftVal
    );
    this.setOpacity(
      this.flapRightUniforms,
      rightVal
    );

    var overallScale = 1.0 + this.flapRotation * 0.125;
    this.innerOrigin.scale.set(overallScale, overallScale, overallScale);
  }
};
