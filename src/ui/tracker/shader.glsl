vertex {
 in vec4 position;
 uniform mat4 viewProjectionMatrix;
 gl_Position = viewProjectionMatrix * position;
 bundle {
  uniform float pointSize;
  gl_PointSize = pointSize / length(gl_Position);
 }
}

fragment {
 out vec4 color;
 bundle {
  color = vec4(0,0.25,0,1) * (1-length(2*gl_PointCoord.xy-vec2(1,1)));
 }
 camera {
  color = vec4(0,0,0.25,1);
 }
}
