vertex {
 in vec4 position;
 uniform mat4 viewProjectionMatrix;
 gl_Position = viewProjectionMatrix * position;
 gl_PointSize = 64 / length(gl_Position);
}

fragment {
 out vec4 color;
 color = vec4(1,1,1,1);
}
