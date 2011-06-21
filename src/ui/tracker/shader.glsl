vertex {
  attribute vec4 position;
  image {
    gl_Position = vec4(position.xy, 0, 1);
    varying vec2 texCoord;
    texCoord = position.zw;
  }
  transform {
    uniform mat4 transform;
    gl_Position = transform * position;
  }
  distort {
    // TODO(MatthiasF): not sure if this is correct
    // low order distortion should be enough for preview
    // u = d + (d − center)(K1*r2)
    vec2 u = gl_Position.xy;
    uniform vec2 center;
    uniform float K1;
    float r2 = length(u-center);
    gl_Position.xy = (K1*r2*center + u) / (K1*r2+1);
  }
  bundle {
    gl_PointSize = 5;
  }
}

fragment {
  image {
    varying vec2 texCoord;
    uniform sampler2D image;
    gl_FragColor = texture2D(image, texCoord);
  }
  marker {
    gl_FragColor = vec4(0, 0.25, 0, 1);
  }
  bundle {
    gl_FragColor = vec4(0, 0.25, 0, 1)
        * (1-step(1, length(2*gl_PointCoord.xy-vec2(1, 1))));
  }
  camera {
    gl_FragColor = vec4(0.25, 0, 0, 1);
  }
  object {
    gl_FragColor = vec4(0, 0, 0.25, 1);
  }
}
