#include "graphics_viz/triangle.h"

Triangle::Triangle(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3){
    vertexArray.push_back(Vertex(v1, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v2 - v1, v3 - v1))));
    vertexArray.push_back(Vertex(v2, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v3 - v2, v1 - v2))));
    vertexArray.push_back(Vertex(v3, glm::vec4(), glm::vec2(), glm::normalize(glm::cross(v1 - v3, v2 - v3))));
    GLuint indexArray[3] = {0, 1, 2};
    index.insert(index.begin(), indexArray, indexArray + sizeof(indexArray) / sizeof(GLuint));
}

Triangle::~Triangle(){
}
