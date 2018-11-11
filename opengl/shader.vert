in vec2 position;
in vec3 color;
in vec2 texcoord;
in float use_texture;

out vec3 pass_color;
out vec2 pass_texcoord;
out float pass_use_texture;

uniform mat4 S;

void main() {
  gl_Position = S * vec4(position, 0.0, 1.0);
  //gl_Position = vec4(position, 0.0, 1.0);

  pass_color = color;
  pass_texcoord = texcoord;
  pass_use_texture = use_texture;
}
