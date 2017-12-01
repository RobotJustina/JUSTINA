#include "graphics_viz/grid.h"

Grid::Grid(int gridSize, float cellSize)
{
    for (int i = -gridSize; i <= gridSize; i++) {
        vertexArray.push_back(Vertex(glm::vec3((float)i * cellSize, -(float)gridSize * cellSize, 0.0f), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, 1.0)));
        vertexArray.push_back(Vertex(glm::vec3((float)i * cellSize, (float)gridSize * cellSize, 0.0f), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, 1.0)));
        vertexArray.push_back(Vertex(glm::vec3(-(float)gridSize * cellSize, (float)i * cellSize, 0.0f), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, 1.0)));
        vertexArray.push_back(Vertex(glm::vec3((float)gridSize * cellSize, (float)i * cellSize, 0.0f), glm::vec4(), glm::vec2(), glm::vec3(0.0, 0.0, 1.0)));
    }
}

Grid::~Grid(){
}

void Grid::render(glm::mat4 parentTrans){
    shader_ptr->turnOn();
    glBindVertexArray(VAO);
    GLint modelLoc = shader_ptr->getUniformLocation("model");
    GLint viewLoc = shader_ptr->getUniformLocation("view");
    GLint projectionLoc = shader_ptr->getUniformLocation("projection");
    glm::mat4 modelMatrix = glm::scale(this->scale);
    modelMatrix = glm::translate(modelMatrix, this->position);
    glm::quat oX = glm::angleAxis<float>(glm::radians(orientation.x), glm::vec3(1.0, 0.0, 0.0));
    glm::quat oY = glm::angleAxis<float>(glm::radians(orientation.y), glm::vec3(0.0, 1.0, 0.0));
    glm::quat oZ = glm::angleAxis<float>(glm::radians(orientation.z), glm::vec3(0.0, 0.0, 1.0));
    glm::quat ori = oZ * oY * oX;
    modelMatrix = parentTrans * modelMatrix * glm::toMat4(ori);
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelMatrix));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(viewMatrix));
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projectionMatrix));
    glDrawArrays(GL_LINES, 0, vertexArray.size());
    glBindVertexArray(0);
    shader_ptr->turnOff();
}

bool Grid::rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection){

}
