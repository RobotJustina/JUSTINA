#ifndef CAMERA_H
#define CAMERA_H
#include <iostream>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform2.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

class Camera
{
public:
    virtual void mouseMoveCamera(float xoffset, float yoffset, int dt) = 0;
    virtual void scrollMoveCamera(float soffset, int dt) = 0;
    virtual void updateCamera() = 0;
    glm::mat4 getViewMatrix(){
        return glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
    }

    void setCameraPos(glm::vec3 cameraPos){
        this->cameraPos = cameraPos;
    }

    glm::vec3 getCameraPos(){
        return this->cameraPos;
    }

    glm::vec3 getCameraUp(){
        return this->cameraUp;
    }

    glm::vec3 getCameraFront(){
        return this->cameraFront;
    }

    float getSensitivity(){
        return sensitivity;
    }

    void setSensitivity(float sensitivity){
        this->sensitivity = sensitivity;
    }

protected:
    glm::vec3 cameraPos;
    glm::vec3 cameraFront;
    glm::vec3 cameraUp;

    float yaw;
    float pitch;
    float sensitivity;
};

#endif // CAMERA_H
