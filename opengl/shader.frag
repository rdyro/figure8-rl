in vec3 pass_color;
in vec2 pass_texcoord;
in float pass_use_texture;

out vec4 out_color;

uniform sampler2D tex;

void main() {
  out_color = vec4(pass_color, 1.0) * (1.0 - pass_use_texture) + 
              texture(tex, pass_texcoord) * pass_use_texture;
}
