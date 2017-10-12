#include "graphics_viz/quad.h"

Quad::Quad(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, glm::vec3 v4){
    vertexArray.push_back(Vertex(v1, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v2 - v1, v3 - v1))));
    vertexArray.push_back(Vertex(v2, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v3 - v2, v1 - v2))));
    vertexArray.push_back(Vertex(v3, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v4 - v3, v2 - v3))));
    vertexArray.push_back(Vertex(v4, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v1 - v4, v3 - v4))));
    GLuint indexArray[6] = {0, 1, 2, 0, 2, 3};
    index.insert(index.begin(), indexArray, indexArray + sizeof(indexArray) / sizeof(GLuint));
}

Quad::~Quad(){
}

void Quad::setUVTexture(glm::vec2 uv1, glm::vec2 uv2, glm::vec2 uv3, glm::vec2 uv4){
    vertexArray[0].m_tex = uv1;
    vertexArray[1].m_tex = uv2;
    vertexArray[2].m_tex = uv3;
    vertexArray[3].m_tex = uv4;
    super::update();
}

void Quad::render(){
    shader_ptr->turnOn();
    if(texture_ptr)
        texture_ptr->bind(GL_TEXTURE0);
    super::render();
    shader_ptr->turnOff();
}
