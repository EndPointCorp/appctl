var CANVAS_SCALE_FACTOR = 1;
var RENDER_SCALE_FACTOR = 1/8;

var PortalGLEnvironment = function() {
  var self = this;

  this.lastDraw = Date.now();

  this.scene = new THREE.Scene();

  this.renderer = new THREE.WebGLRenderer({
    alpha: true,
    canvas: this.canvas
  });
  this.renderer.setClearColor(new THREE.Color(0x000000), 0);

  this.canvas = this.renderer.domElement;
  this.canvas.style.position = 'fixed';
  this.canvas.style.bottom = '0px';
  this.canvas.style.left = '0px';
  this.canvas.style.zIndex = '99999';
  this.canvas.style.backgroundColor = 'rgba(255, 255, 255, 0.0)';
  this.canvas.style.pointerEvents = 'none';
  document.body.appendChild(this.canvas);

  this.camera = new THREE.PerspectiveCamera(
    45,
    window.innerWidth / window.innerHeight,
    0.1,
    65535
  );
  this.camera.position.set(0, 0, 0);
  this.scene.add(this.camera);

  function handleResize() {
    self.canvas.style.width =
        (window.innerWidth * CANVAS_SCALE_FACTOR) + 'px';
    self.canvas.style.height =
        (window.innerHeight * CANVAS_SCALE_FACTOR) + 'px';
    self.renderer.setSize(
      window.innerWidth * RENDER_SCALE_FACTOR,
      window.innerHeight * RENDER_SCALE_FACTOR
    );
    self.camera.aspect = window.innerWidth / window.innerHeight;

    self.camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', handleResize);
  handleResize();

  this.animations = [];

  this.animate();
};

/**
 * Runs all registered animation methds and renders the scene.
 */
PortalGLEnvironment.prototype.animate = function() {
  var now = Date.now();

  var self = this;
  function _animate() {
    self.animate();
  }

  requestAnimationFrame(_animate);

  // skip duplicate frames
  if (now - this.lastDraw < 14) {
    return;
  }

  var numAnimations = this.animations.length;
  for (var i = 0; i < numAnimations; i++) {
    this.animations[i]();
  }

  this.renderer.render(this.scene, this.camera);
  this.lastDraw = now;
};

/**
 * Registers an animation method.
 * @param {function} callback
 */
PortalGLEnvironment.prototype.addAnimation = function(callback) {
  this.animations.push(callback);
};
