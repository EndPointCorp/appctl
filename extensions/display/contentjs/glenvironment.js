var SlinkyGLEnvironment = function() {
  var self = this;

  this.scene = new THREE.Scene();

  this.canvas = document.getElementById('glCanvas');

  if (!this.canvas) {
    this.canvas = document.createElement('canvas');
    this.canvas.id = 'glCanvas';
    this.canvas.style.position = 'fixed';
    this.canvas.style.bottom = '0px';
    this.canvas.style.left = '0px';
    this.canvas.style.height = '100%';
    this.canvas.style.width = '100%';
    this.canvas.style.zIndex = '99999';
    this.canvas.style.backgroundColor = 'rgba(255, 255, 255, 0.0)';
    this.canvas.style.pointerEvents = 'none';
    document.body.appendChild(this.canvas);
  }

  this.renderer = new THREE.WebGLRenderer({
    canvas: this.canvas,
    alpha: true,
    antialiasing: true
  });
  this.renderer.setSize(window.innerWidth, window.innerHeight);
  this.renderer.setClearColor(new THREE.Color(0x000000), 0);

  this.camera = new THREE.PerspectiveCamera(
    45,
    window.innerWidth / window.innerHeight,
    0.1,
    65535
  );
  this.camera.position.set(0, 0, 0);
  this.scene.add(this.camera);

  window.addEventListener('resize', function() {
    self.renderer.setSize(window.innerWidth, window.innerHeight);
    self.camera.aspect = window.innerWidth / window.innerHeight;

    self.camera.updateProjectionMatrix();
  });

  this.animations = [];

  this.animate();
};

/**
 * Runs all registered animation methds and renders the scene.
 */
SlinkyGLEnvironment.prototype.animate = function() {
  var self = this;
  function _animate() {
    self.animate();
  }

  requestAnimationFrame(_animate);

  var numAnimations = this.animations.length;
  for (var i = 0; i < numAnimations; i++) {
    this.animations[i]();
  }

  this.renderer.render(this.scene, this.camera);
};

/**
 * Registers an animation method.
 * @param {function} callback
 */
SlinkyGLEnvironment.prototype.addAnimation = function(callback) {
  this.animations.push(callback);
};
