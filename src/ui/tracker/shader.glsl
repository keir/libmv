vertex {
  attribute vec4 position;
  image {
    gl_Position = vec4(position.xy,0,1);
    varying vec2 texCoord;
    texCoord = position.zw;
  }
  transform {
    uniform mat4 transform;
    gl_Position = transform * position;
  }
  bundle {
    uniform float pointSize;
    gl_PointSize = pointSize / length(gl_Position);
  }
}

fragment {
  image {
    varying vec2 texCoord;
    uniform sampler2D image;
    gl_FragColor = texture2D(image,texCoord);
  }
  marker {
    gl_FragColor = vec4(0,0.25,0,1);
  }
  bundle {
    gl_FragColor = vec4(0,0.25,0,1) * (1-length(2*gl_PointCoord.xy-vec2(1,1)));
  }
  camera {
    gl_FragColor = vec4(0.25,0,0,1);
  }
  object {
    gl_FragColor = vec4(0,0,0.25,1);
  }
}
