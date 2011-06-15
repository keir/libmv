vertex {
  varying in vec4 position;
  image {
    gl_Position = vec4(position.xy,0,1);
    varying out vec2 texCoord;
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
  varying out vec4 color;
  image {
    varying in vec2 texCoord;
    uniform sampler2D image;
    color = texture2D(image,texCoord);
  }
  marker {
    color = vec4(0,0.25,0,1);
  }
  bundle {
    color = vec4(0,0.25,0,1) * (1-length(2*gl_PointCoord.xy-vec2(1,1)));
  }
  camera {
    color = vec4(0.25,0,0,1);
  }
  object {
    color = vec4(0,0,0.25,1);
  }
}
