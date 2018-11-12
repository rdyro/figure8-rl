in vec2 position;
in vec3 color;

out vec3 pass_color;

uniform mat4 S;

void main() {
  gl_Position = S * vec4(position, 0.0, 1.0);
  //gl_Position = vec4(position, 0.0, 1.0);

  pass_color = color;
}
