#include "graphics_viz/lines.h"

Lines::Lines(){
}

Lines::~Lines(){
}

void Lines::init(glm::vec3 v1, glm::vec3 v2){
    vertexArray.push_back(VertexColor(v1, glm::vec4()));
    vertexArray.push_back(VertexColor(v2, glm::vec4()));

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertexArray.size() * sizeof(vertexArray[0]), vertexArray.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertexArray[0]), (GLvoid*) 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertexArray[0]), (GLvoid*) sizeof(vertexArray[0].m_pos));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Lines::init(std::vector<glm::vec3> linesStrip){
    for(std::vector<glm::vec3>::iterator it = linesStrip.begin(); it != linesStrip.end(); it++)
        vertexArray.push_back(VertexColor(*it, glm::vec4()));

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertexArray.size() * sizeof(vertexArray[0]), vertexArray.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertexArray[0]), (GLvoid*) 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertexArray[0]), (GLvoid*) sizeof(vertexArray[0].m_pos));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Lines::render(LINES_MODE linesMode){
    shader_ptr->turnOn();
    glBindVertexArray(VAO);
    GLint modelLoc = shader_ptr->getUniformLocation("model");
    GLint viewLoc = shader_ptr->getUniformLocation("view");
    GLint projectionLoc = shader_ptr->getUniformLocation("projection");
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelMatrix));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(viewMatrix));
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projectionMatrix));
    if(linesMode == LINES && vertexArray.size() > 0)
        glDrawArrays(GL_LINES, 0, vertexArray.size());
    else if(linesMode == LINE_STRIP && vertexArray.size() > 0)
        glDrawArrays(GL_LINE_STRIP, 0, vertexArray.size());
    glBindVertexArray(0);
    shader_ptr->turnOff();
}

void Lines::update(){

}
