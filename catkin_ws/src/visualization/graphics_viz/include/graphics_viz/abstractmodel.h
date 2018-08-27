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
    enum TypeModel{
        BOX, SPHERE, CYLINDER, QUAD, TRIANGLE, NUM_MODELS = TRIANGLE + 1
    };

    class SBB{
    public:
        SBB(){
        }

        SBB(glm::vec3 c, float ratio){
            this->c = c;
            this->ratio = ratio;
        }
        glm::vec3 c;
        float ratio;
    };

    class AABB{
    public:
        AABB(){
        }

        AABB(glm::vec3 mins, glm::vec3 maxs){
            this->mins = mins;
            this->maxs = maxs;
        }
        AABB(glm::vec3 c, float width, float height, float length){
            mins.x = c.x - width / 2.0;
            mins.y = c.y - height / 2.0;
            mins.z = c.z - length / 2.0;
            maxs.x = c.x + width / 2.0;
            maxs.y = c.y + height / 2.0;
            maxs.z = c.z + length / 2.0;
        }
        AABB(float minx, float miny, float minz, float maxx, float maxy, float maxz){
            mins.x = minx;
            mins.y = miny;
            mins.z = minz;
            maxs.x = maxx;
            maxs.y = maxy;
            maxs.z = maxz;
        }
        glm::vec3 mins;
        glm::vec3 maxs;
    };

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

    void render(glm::mat4 parentTrans = glm::mat4()){
        shader_ptr->turnOn();
        glBindVertexArray(VAO);
        GLint modelLoc = shader_ptr->getUniformLocation("model");
        GLint viewLoc = shader_ptr->getUniformLocation("view");
        GLint projectionLoc = shader_ptr->getUniformLocation("projection");
        glm::mat4 scale = glm::scale(this->scale);
        glm::mat4 translate = glm::translate(this->position);
        glm::quat oX = glm::angleAxis<float>(glm::radians(orientation.x), glm::vec3(1.0, 0.0, 0.0));
        glm::quat oY = glm::angleAxis<float>(glm::radians(orientation.y), glm::vec3(0.0, 1.0, 0.0));
        glm::quat oZ = glm::angleAxis<float>(glm::radians(orientation.z), glm::vec3(0.0, 0.0, 1.0));
        glm::quat ori = oZ * oY * oX;
        glm::mat4 modelMatrix = parentTrans * translate * glm::toMat4(ori) * scale;
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

    Shader * getShader(){
        return this->shader_ptr;
    }

    glm::mat4 getProjectionMatrix(){
        return this->projectionMatrix;
    }

    void setProjectionMatrix(glm::mat4 projectionMatrix){
        this->projectionMatrix = projectionMatrix;
    }

    glm::vec3 getPosition(){
        return this->position;
    }

    glm::vec3 getScale(){
        return this->scale;
    }

    glm::vec3 getOrientation(){
        return this->orientation;
    }

    void setPosition(glm::vec3 position){
        this->position = position;
    }

    void setScale(glm::vec3 scale){
        this->scale = scale;
    }

    void setOrientation(glm::vec3 orientation){
        this->orientation = orientation;
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
        this->color = color;
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

    glm::vec4 getColor(){
        return this->color;
    }

    TypeModel getTypeModel(){
        return typeModel;
    }

    void setTypeModel(TypeModel typeModel){
        this->typeModel = typeModel;
    }

    virtual bool rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection) = 0;

protected:
    Shader * shader_ptr;
    glm::mat4 projectionMatrix;
    glm::mat4 viewMatrix;
    glm::vec3 position = glm::vec3(0.0, 0.0, 0.0);
    glm::vec3 scale = glm::vec3(1.0, 1.0, 1.0);
    glm::vec4 color;
    glm::vec3 orientation;
    GLuint VAO, VBO, EBO;
    std::vector<Vertex> vertexArray;
    std::vector<GLuint> index;
    TypeModel typeModel;
};

#endif // ABSTRACTMODEL_H
