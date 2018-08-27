#version 330 core
in vec2 ex_texCoord;

out vec4 color;

uniform sampler2D tex;

void main()
{
        color = texture(tex, ex_texCoord);
}
