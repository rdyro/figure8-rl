in vec3 pass_color;
in float pass_usetex;
in vec2 pass_texcoord;

out vec4 out_color;

uniform sampler2D tex;

void main() {
  out_color = (vec4(pass_color, 1.0) * (1.0 - pass_usetex) +
              texture(tex, pass_texcoord) * pass_usetex);
}
