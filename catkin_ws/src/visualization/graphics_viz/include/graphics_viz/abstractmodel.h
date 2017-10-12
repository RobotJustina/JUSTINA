#ifndef ABSTRACTMODEL_H
#define ABSTRACTMODEL_H
#include <iostream>
#include <vector>
#include <memory>
#include <GL/glew.h>
#include "Shader.h"
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform2.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

class AbstractModel
{
public:

    class Vertex{
    public:
        Vertex(){
        }

        Vertex(glm::vec3 m_pos, glm::vec4 m_color, glm::vec2 m_tex, glm::vec3 m_normal): m_pos(m_pos), m_color(m_color), m_tex(m_tex), m_normal(m_normal){
        }

    public:
        glm::vec3 m_pos;
        glm::vec4 m_color;
        glm::vec2 m_tex;
        glm::vec3 m_normal;
    };

    class VertexColor{
    public:
        VertexColor(){
        }

        VertexColor(glm::vec3 m_pos, glm::vec4 m_color): m_pos(m_pos), m_color(m_color){
        }
    public:
        glm::vec3 m_pos;
        glm::vec4 m_color;
    };

    void init(){
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);

        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertexArray.size() * sizeof(vertexArray[0]), vertexArray.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, index.size() * sizeof(index[0]), index.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertexArray[0]), (GLvoid*) 0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(vertexArray[0]), (GLvoid*) sizeof(vertexArray[0].m_pos));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(vertexArray[0]), (GLvoid*) (sizeof(vertexArray[0].m_pos) + sizeof(vertexArray[0].m_color)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(vertexArray[0]), (GLvoid*) (sizeof(vertexArray[0].m_pos) + sizeof(vertexArray[0].m_color) + sizeof(vertexArray[0].m_tex)));
        glEnableVertexAttribArray(3);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    void update(){
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        GLvoid* p = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
        memcpy(p, vertexArray.data(), vertexArray.size() * sizeof(vertexArray[0]));
        glUnmapBuffer(GL_ARRAY_BUFFER);
    }

    void render(){
        shader_ptr->turnOn();
        glBindVertexArray(VAO);
        GLint modelLoc = shader_ptr->getUniformLocation("model");
        GLint viewLoc = shader_ptr->getUniformLocation("view");
        GLint projectionLoc = shader_ptr->getUniformLocation("projection");
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelMatrix));
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(viewMatrix));
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projectionMatrix));
        glDrawElements(GL_TRIANGLES, index.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        shader_ptr->turnOff();
        this->enableFillMode();
    }

    void destroy(){
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glDeleteBuffers(1, &VBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glDeleteBuffers(1, &EBO);
        glBindVertexArray(0);
        glDeleteVertexArrays(1, &VAO);
    }

    void setShader(Shader * shader_ptr){
        this->shader_ptr = shader_ptr;
    }

    glm::mat4 getProjectionMatrix(){
        return this->projectionMatrix;
    }

    void setProjectionMatrix(glm::mat4 projectionMatrix){
        this->projectionMatrix = projectionMatrix;
    }

    glm::mat4 getModelMatrix(){
        return this->modelMatrix;
    }

    void setModelMatrix(glm::mat4 modelMatrix){
        this->modelMatrix = modelMatrix;
    }

    glm::mat4 getViewMatrix(){
        return this->viewMatrix;
    }

    void setViewMatrix(glm::mat4 viewMatrix){
        this->viewMatrix = viewMatrix;
    }

    void enableWireMode(){
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }

    void enableFillMode(){
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    void setColor(glm::vec4 color){
        shader_ptr->turnOn();
        GLint locColor = shader_ptr->getUniformLocation("color");
        //std::cout << "locColor:" << locColor << std::endl;
        //glUniform4fv(locColor, 1, glm::value_ptr(color));
        //glUniform4f(locColor, 0.0, 1.0, 1.0, 1.0);
        if(locColor >= 0)
            glUniform4fv(locColor, 1, glm::value_ptr(color));
        else{
            for(std::vector<Vertex>::iterator it = vertexArray.begin(); it != vertexArray.end(); it++)
                (*it).m_color = color;
            update();
        }
        shader_ptr->turnOff();
    }

protected:
    Shader * shader_ptr;
    glm::mat4 projectionMatrix;
    glm::mat4 viewMatrix;
    glm::mat4 modelMatrix;
    GLuint VAO, VBO, EBO;
    std::vector<Vertex> vertexArray;
    std::vector<GLuint> index;
};

#endif // ABSTRACTMODEL_H
