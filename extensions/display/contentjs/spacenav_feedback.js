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
  this.absOrigin.position.set(0, -14, -70);
  this.absOrigin.rotation.set(0.5, 0, 0);
  this.innerOrigin = new THREE.Object3D();

  this.spacenav_min = -350;
  this.spacenav_max = 350;

  this.arrows_min = -3;
  this.arrows_max = 3;

  this.arrowObj;
  this.ringObj;

  this.arrowObjPosition = [ 0, 0, 0, 0, 0, 0 ];
  this.ringObjPosition = [ 0, 0, 0, 0, 0, 0 ];

  this.arrowOpacity = 0;
  this.ringOpacity = 0;
};

SpacenavFeedback.prototype.setOpacity = function(threejs_obj, opacity) {
  threejs_obj.traverse(function(child) {
    if (child instanceof THREE.Mesh) {
      child.material.opacity = opacity;
    }
  });
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
      this.ringOpacity = 0;
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
    if (ring_opacity > this.arrows_max / 20) { // yes that's evil
      this.ringOpacity = ring_opacity;
      // pull up , push down
      this.ringObjPosition[1] = linearZ;
      // rotate (twist)
      this.ringObjPosition[4] = angularZ * 0.5;
      // lean forward and backward
      //this.ringObjPosition[5] = angularX * -0.1;
      this.ringObjPosition[5] = 0;
      this.ringObjPosition[3] = angularY * -0.1;
    } else {
      this.ringOpacity = 0;
    }

    // let's rotate and show the direction arrow with little tresholding
    if ((Math.abs(linearY) > 0.2)
        || (Math.abs(linearX) > 0.2)) {
      linearY = linearY*-1;
      linearX = linearX*-1;

      var direction = (Math.atan2(linearY, linearX)
          / Math.PI * 180)
          / -18;

      /*
      console.log("This is direction1:", direction,
          "computed out of (x,y)", this.msg.linear.y, "/",
          this.msg.linear.x);
      */

      this.arrowOpacity = Math.max(Math.abs(linearY), Math.abs(linearX));
      this.arrowObjPosition[4] = direction;
    }
  }
};

SpacenavFeedback.prototype.init = function() {
  this.scene.add(this.absOrigin);
  this.absOrigin.add(this.innerOrigin);

  var arrows_loader = new THREE.OBJLoader(manager);
  arrows_loader.load(chrome.extension.getURL('models/arrow.obj'), function(
      arrow_object) {
    this.arrowObj = arrow_object;
    arrow_object.traverse(function(child) {
      if (child instanceof THREE.Mesh) {
        child.material.map = arrow_texture;
        child.material.transparent = true;
        child.material.opacity = 0.8;
      }
    }.bind(this));
    this.innerOrigin.add(arrow_object);
  }.bind(this));

  var ring_loader = new THREE.OBJLoader(manager);
  ring_loader.load(chrome.extension.getURL('models/ring.obj'), function(
      ring_object) {
    this.ringObj = ring_object;
    ring_object.traverse(function(child) {
      if (child instanceof THREE.Mesh) {
        child.material.map = ring_texture;
        child.material.transparent = true;
        child.material.opacity = 0.8;
      }
    });
    this.innerOrigin.add(ring_object);
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
    this.setOpacity(this.arrowObj, this.arrowOpacity);
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
    this.setOpacity(this.ringObj, this.ringOpacity);
  }
};
