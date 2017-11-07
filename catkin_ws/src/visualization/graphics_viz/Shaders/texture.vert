#version 330 core
layout(location=0) in vec3 position;
layout(location=2) in vec2 texCoord;

out vec2 ex_texCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main(){
    gl_Position = projection * view * model * vec4(position, 1.0);
    // Just pass the color through directly.
    ex_texCoord = vec2(texCoord.x, texCoord.y);
}
