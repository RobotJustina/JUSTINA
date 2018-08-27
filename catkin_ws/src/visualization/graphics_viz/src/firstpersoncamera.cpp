#include "graphics_viz/firstpersoncamera.h"

FirstPersonCamera::FirstPersonCamera(){
    yaw = -90.0f;
    cameraPos = glm::vec3(0.0f, 3.0f, 3.0f);
    cameraFront = glm::vec3(0.0f, -1.0f, 0.0f);
    cameraUp = glm::vec3(0.0f, 0.0f, 1.0f);
    sensitivity = 0.02;
}

void FirstPersonCamera::mouseMoveCamera(float xoffset, float yoffset, int dt){
    xoffset *= dt * sensitivity;
    yoffset *= -1 * dt * sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    updateCamera();
}

void FirstPersonCamera::scrollMoveCamera(float soffset, int dt){
}

void FirstPersonCamera::updateCamera(){
    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    glm::vec3 front;

    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.z = sin(glm::radians(pitch));
    front.y = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraFront = glm::normalize(front);
}

void FirstPersonCamera::moveFrontCamera(bool dir, int dt){
    if(dir)
        cameraPos += (float)dt * sensitivity * cameraFront;
    else
        cameraPos -= (float)dt * sensitivity * cameraFront;
}

void FirstPersonCamera::moveRightCamera(bool dir, int dt){
    if(dir)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * (float)dt * sensitivity;
    else
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * (float)dt * sensitivity;
}
