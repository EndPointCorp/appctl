/*
 * Feedback arrows UX extension.
 * Shows two independent webgl models that should immitate spacenav movement.
 * Object should fade in and fade out on spacenav use.
 */

var SpacenavFeedback = function(glEnvironment) {
  this.glEnvironment = glEnvironment;
  this.canvas = glEnvironment.canvas;
  this.renderer = glEnvironment.renderer;
  this.camera = glEnvironment.camera;
  this.scene = glEnvironment.scene;
  this.absOrigin = new THREE.Object3D();
  this.absOrigin.position.set(0, -7, -95);
  this.absOrigin.rotation.set(0.5, 0, 0);
  this.absOrigin.scale.set(1.25, 1.25, 1.25);

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

  this.arrowObjPosition = [ 0, 0, 0, 0, 0, 0 ];
  this.ringObjPosition = [ 0, 0, 0, 0, 0, 0 ];
  this.flapRotation = 0.0;

  this.arrowOpacity = 0.0;
  this.ringXOpacity = 0.0;
  this.ringZOpacity = 0.0;
  this.flapOpacity = 0.0;
};

SpacenavFeedback.prototype.setOpacity = function(uniforms, opacity) {
  if (uniforms.fade.value != opacity) {
    uniforms.fade.value = opacity;
    uniforms.fade.needsUpdate = true;
  }
};

SpacenavFeedback.prototype.clampAxis = function(num, low, high) {
  return (num - this.spacenav_min) * (high - low) / (this.spacenav_max - this.spacenav_min) + low;
};

SpacenavFeedback.prototype.processSpacenavMessage = function(msg) {
  /*
   * Two possible states: spacenav not being used (all zeros), else spacenav
   * being touched - we rotate our pretty objects
   */
  if (msg.linear.x == 0 && msg.linear.y == 0
      && msg.linear.z == 0 && msg.angular.x == 0
      && msg.angular.y == 0 && msg.angular.z == 0) {
    /*
     * - fade objects out - return to point 0
     */
    for (var i = 0; i < this.arrowObjPosition.length; i++) {
      this.arrowObjPosition[i] = 0;
      this.ringObjPosition[i] = 0;
    }
    // set opacity to 0
    if ((typeof (this.arrowObj) === "undefined")
        && (typeof (this.ringObj) === "undefined")) {
      console.log("Initializing objects");
    } else {
      this.ringXOpacity = 0;
      this.ringZOpacity = 0;
      this.arrowOpacity = 0;
    }
    return;

  } else {
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
     * ]
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
    var linearX = this.clampAxis(msg.linear.x, -1, 1);
    var linearY = this.clampAxis(msg.linear.y, -1, 1);

    // needed for ring
    var linearZ = this.clampAxis(msg.linear.z, this.arrows_min, this.arrows_max);
    var angularX = this.clampAxis(msg.angular.x, this.arrows_min, this.arrows_max);
    var angularY = this.clampAxis(msg.angular.y, this.arrows_min, this.arrows_max);
    var angularZ = this.clampAxis(msg.angular.z, this.arrows_min, this.arrows_max);

    // make object transparency proportional to the values

    // add pretty curve possibly y=3x/(x+2)
    var ring_opacity = Math.max(Math.abs(linearZ), Math
        .abs(angularZ), Math.abs(angularX), Math
        .abs(angularY))
        / this.arrows_max;

    this.flapRotation = this.clampAxis(msg.linear.z, -1, 1);

    if (Math.abs(ring_opacity) > this.arrows_max / 20) { // yes that's evil
      this.ringXOpacity = this.clampAxis(msg.angular.z, -1, 1);
      this.ringZOpacity = -this.clampAxis(msg.angular.y, -1, 1);
      //this.ringZOpacity = ring_opacity;
      // pull up , push down
      //this.ringObjPosition[1] = linearZ;
      // rotate (twist)
      this.ringObjPosition[4] = angularZ * 0.5;
      // lean forward and backward
      //this.ringObjPosition[5] = angularX * -0.1;
      this.ringObjPosition[5] = 0;
      this.ringObjPosition[3] = angularY * -0.1;
    } else {
      this.ringXOpacity = 0.0;
      this.ringZOpacity = 0.0;
    }

    // let's rotate and show the direction arrow with little tresholding
    if ((Math.abs(linearY) > 0.2)
        || (Math.abs(linearX) > 0.2)) {

      var direction = Math.atan2(-linearY, -linearX);

      /*
      console.log("This is direction1:", direction,
          "computed out of (x,y)", this.msg.linear.y, "/",
          this.msg.linear.x);
      */

      this.arrowOpacity = Math.max(Math.abs(linearY), Math.abs(linearX));
      this.arrowObjPosition[4] = direction;
    } else {
      this.arrowOpacity = 0.0;
    }
  }
};

SpacenavFeedback.prototype.paintGeometry = function(geometry, color) {
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

SpacenavFeedback.prototype.init = function() {
  this.scene.add(this.absOrigin);
  this.absOrigin.add(this.innerOrigin);

  this.arrowUniforms = {
    alpha: { type: 'f', value: 0.23 },
    fade: { type: 'f', value: 0.0 },
    fadeRadius: { type: 'f', value: 0.0 }
  };

  this.ringUniforms = {
    alpha: { type: 'f', value: 0.23 },
    xFade: { type: 'f', value: 0.0 },
    zFade: { type: 'f', value: 0.0 },
    fadeRadius: { type: 'f', value: 0.0 }
  };

  this.flapLeftUniforms = {
    alpha: { type: 'f', value: 0.23 },
    fade: { type: 'f', value: 0.0 }
  };

  this.flapRightUniforms = {
    alpha: { type: 'f', value: 0.23 },
    fade: { type: 'f', value: 0.0 }
  };

  var arrowShader = new THREE.ShaderMaterial({
    vertexColors: THREE.VertexColors,
    uniforms: this.arrowUniforms,
    vertexShader: document.getElementById('zgradientvertexshader').textContent,
    fragmentShader: document.getElementById('zgradientfragmentshader').textContent,
    transparent: true,
    depthTest: false
  });

  var ringShader = new THREE.ShaderMaterial({
    vertexColors: THREE.VertexColors,
    uniforms: this.ringUniforms,
    vertexShader: document.getElementById('xzgradientvertexshader').textContent,
    fragmentShader: document.getElementById('xzgradientfragmentshader').textContent,
    transparent: true,
    depthTest: false
  });

  var flapLeftShader = new THREE.ShaderMaterial({
    vertexColors: THREE.VertexColors,
    uniforms: this.flapLeftUniforms,
    vertexShader: document.getElementById('vcolorvertexshader').textContent,
    fragmentShader: document.getElementById('vcolorfragmentshader').textContent,
    transparent: true,
    depthTest: false
  });

  var flapRightShader = new THREE.ShaderMaterial({
    vertexColors: THREE.VertexColors,
    uniforms: this.flapRightUniforms,
    vertexShader: document.getElementById('vcolorvertexshader').textContent,
    fragmentShader: document.getElementById('vcolorfragmentshader').textContent,
    transparent: true,
    depthTest: false
  });

  var arrowLoader = new THREE.JSONLoader();
  arrowLoader.load(chrome.extension.getURL('models/arrow.json'), function(geometry) {
    this.paintGeometry(geometry, new THREE.Color(0xFFFFFF));
    this.arrowObj = new THREE.Mesh(
      geometry,
      arrowShader
    );
    geometry.computeBoundingSphere();
    this.arrowUniforms.fadeRadius.value = geometry.boundingSphere.radius;
    this.arrowUniforms.fadeRadius.needsUpdate = true;
    this.innerOrigin.add(this.arrowObj);
  }.bind(this));

  var ringLoader = new THREE.JSONLoader();
  ringLoader.load(chrome.extension.getURL('models/ring.json'), function(geometry) {
    this.paintGeometry(geometry, new THREE.Color(0xFFFFFF));
    this.ringObj = new THREE.Mesh(
      geometry,
      ringShader
    );
    geometry.computeBoundingSphere();
    this.flapLeftOrigin.position.set(-geometry.boundingSphere.radius + 0.1, 0, 0);
    this.flapRightOrigin.position.set(geometry.boundingSphere.radius - 0.1, 0, 0);
    this.ringUniforms.fadeRadius.value = geometry.boundingSphere.radius;
    this.ringUniforms.fadeRadius.needsUpdate = true;
    this.innerOrigin.add(this.ringObj);
    this.ringObj.add(this.flapLeftOrigin);
    this.ringObj.add(this.flapRightOrigin);
  }.bind(this));

  var flapLoader = new THREE.JSONLoader();
  flapLoader.load(chrome.extension.getURL('models/flap.json'), function(geometry) {
    this.paintGeometry(geometry, new THREE.Color(0xFFFFFF));
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

SpacenavFeedback.prototype.animate = function() {
  if (typeof (this.arrowObj) === "undefined") {
    console.log("Still initializing arrowObj");
  } else {
    this.arrowObj.position.x = this.arrowObjPosition[0];
    this.arrowObj.position.y = this.arrowObjPosition[1];
    this.arrowObj.position.z = this.arrowObjPosition[2];
    this.arrowObj.rotation.x = this.arrowObjPosition[3];
    this.arrowObj.rotation.y = this.arrowObjPosition[4];
    this.arrowObj.rotation.z = this.arrowObjPosition[5];
    this.setOpacity(this.arrowUniforms, this.arrowOpacity);
  }

  if (typeof (this.ringObj) === "undefined") {
    console.log("Still initializing ringObj");
  } else {
    this.innerOrigin.position.x = this.ringObjPosition[0];
    this.innerOrigin.position.y = this.ringObjPosition[1];
    this.innerOrigin.position.z = this.ringObjPosition[2];
    this.innerOrigin.rotation.x = this.ringObjPosition[3];
    this.ringObj.rotation.y = this.ringObjPosition[4];
    this.innerOrigin.rotation.z = this.ringObjPosition[5];

    this.ringUniforms.xFade.value = this.ringXOpacity;
    this.ringUniforms.zFade.value = this.ringZOpacity;
    this.ringUniforms.xFade.needsUpdate = true;
    this.ringUniforms.zFade.needsUpdate = true;
  }

  if (typeof this.flapLeftObj === "undefined" ||
      typeof this.flapRightObj === "undefined") {

    console.log("Still initializing flap objects");
  } else {
    this.flapRightObj.rotation.z = this.flapRotation;
    this.flapLeftObj.rotation.z = this.flapRotation;

    this.setOpacity(
      this.flapLeftUniforms,
      Math.min(1, Math.max(0, Math.abs(this.flapRotation) - this.ringXOpacity))
    );
    this.setOpacity(
      this.flapRightUniforms,
      Math.min(1, Math.max(0, Math.abs(this.flapRotation) + this.ringXOpacity))
    );
  }
};
