in vec2 position;
in vec3 color;
in float usetex;
in vec2 texcoord;

out vec3 pass_color;
out float pass_usetex;
out vec2 pass_texcoord;

uniform mat4 Wmat;
uniform mat4 Tmat;

void main() {
  gl_Position = Wmat * Tmat * vec4(position, 0.0, 1.0);

  pass_color = color;
  pass_usetex = usetex;
  pass_texcoord = texcoord;
}
