vertex {
  in vec4 position;
  image {
    gl_Position = vec4(position.xy,0,1);
    out vec2 texCoord;
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
  out vec4 color;
  image {
    in vec2 texCoord;
    uniform sampler2D image;
    color = texture(image,texCoord);
    color = vec4(texture(image,texCoord).rgb,1);
    //color = vec4(texCoord,0,1);
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
