vertex {
 in vec4 position;
 uniform mat4 viewProjectionMatrix;
 gl_Position = viewProjectionMatrix * position;
 bundle {
  gl_PointSize = 1024 / length(gl_Position);
 }
}

fragment {
 out vec4 color;
 bundle {
  color = vec4( 1-length(2*gl_PointCoord.xy-vec2(1,1)) );
 }
 camera {
  color = vec4( 0,0,1,1 );
 }
}
