/**
 * Inject the GL shaders into the page.
 */
var SlinkyShaders = (function() {
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
  var xRayVertexScript = document.createElement('script');
  xRayVertexScript.type = 'x-shader/x-vertex';
  xRayVertexScript.id = 'xrayvertexshader';
  xRayVertexScript.textContent =
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
  document.body.appendChild(xRayVertexScript);

  var xRayFragmentScript = document.createElement('script');
  xRayFragmentScript.type = 'x-shader/x-fragment';
  xRayFragmentScript.id = 'xrayfragmentshader';
  xRayFragmentScript.textContent =
    'varying vec4 vColor;' +

    'void main() {' +
    '    gl_FragColor = vec4( vColor );' +
    '}';
  document.body.appendChild(xRayFragmentScript);

  // Compass Rose Geometry Shaders.
  var vcolorVertexScript = document.createElement('script');
  vcolorVertexScript.type = 'x-shader/x-vertex';
  vcolorVertexScript.id = 'vcolorvertexshader';
  vcolorVertexScript.textContent =
    'uniform float alpha;' +
    'uniform float fade;' +
    'varying vec4 vColor;' +

    'void main() {' +
    '    vColor.rgb = color.rgb;' +
    '    vColor.a = alpha * fade;' +
    '    vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );' +
    '    gl_Position = projectionMatrix * mvPosition;' +
    '}';
  document.body.appendChild(vcolorVertexScript);

  var vcolorFragmentScript = document.createElement('script');
  vcolorFragmentScript.type = 'x-shader/x-fragment';
  vcolorFragmentScript.id = 'vcolorfragmentshader';
  vcolorFragmentScript.textContent =
    'varying vec4 vColor;' +

    'void main() {' +
    '    gl_FragColor = vColor;' +
    '}';
  document.body.appendChild(vcolorFragmentScript);

  // Gradient fade from X and Z edges.
  var xzGradientVertexScript = document.createElement('script');
  xzGradientVertexScript.type = 'x-shader/x-vertex';
  xzGradientVertexScript.id = 'xzgradientvertexshader';
  xzGradientVertexScript.textContent =
    'uniform float alpha;' +
    'uniform float fadeRadius;' +
    'uniform float zFade;' +
    'uniform float xFade;' +
    'uniform float bothXFade;' +
    'varying vec4 vColor;' +

    'void main() {' +
    '    vColor.rgb = color.rgb;' +
    '    float doubleRad = fadeRadius * 2.0;' +
    '    float xPosFade = (position.x / doubleRad * xFade) +' +
    '                     abs(position.x / fadeRadius * bothXFade);' +
    '    float zPosFade = (position.z / doubleRad * zFade);' +
    '    vColor.a = alpha * max(0.0, min(1.0, (xPosFade + zPosFade)));' +
    '    vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );' +
    '    gl_Position = projectionMatrix * mvPosition;' +
    '}';
  document.body.appendChild(xzGradientVertexScript);

  var xzGradientFragmentScript = document.createElement('script');
  xzGradientFragmentScript.type = 'x-shader/x-fragment';
  xzGradientFragmentScript.id = 'xzgradientfragmentshader';
  xzGradientFragmentScript.textContent =
    'varying vec4 vColor;' +

    'void main() {' +
    '    gl_FragColor = vColor;' +
    '}';
  document.body.appendChild(xzGradientFragmentScript);

  // Gradient fade from z = N.
  var zGradientVertexScript = document.createElement('script');
  zGradientVertexScript.type = 'x-shader/x-vertex';
  zGradientVertexScript.id = 'zgradientvertexshader';
  zGradientVertexScript.textContent =
    'uniform float alpha;' +
    'uniform float fade;' +
    'uniform float fadeRadius;' +
    'varying vec4 vColor;' +

    'void main() {' +
    '    vColor.rgb = color.rgb;' +
    '    vColor.a = alpha * fade * (position.z / fadeRadius);' +
    '    vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );' +
    '    gl_Position = projectionMatrix * mvPosition;' +
    '}';
  document.body.appendChild(zGradientVertexScript);

  var zGradientFragmentScript = document.createElement('script');
  zGradientFragmentScript.type = 'x-shader/x-fragment';
  zGradientFragmentScript.id = 'zgradientfragmentshader';
  zGradientFragmentScript.textContent =
    'varying vec4 vColor;' +

    'void main() {' +
    '    gl_FragColor = vColor;' +
    '}';
  document.body.appendChild(zGradientFragmentScript);
})();

