#include "graphics_viz/orbitcamera.h"

OrbitCamera::OrbitCamera(){
    cameraUp = glm::vec3(0, 0, 1);
    cameraFront = glm::normalize(cameraTarget - cameraPos);
    cameraTarget = glm::vec3(0.0f, 0.0f, 0.0f);
    pitch = M_PI / 4;
    yaw = M_PI / 4;
    ratio = 1.0;
    sensitivity = 0.001;
    updateCamera();
}

void OrbitCamera::mouseMoveCamera(float xoffset, float yoffset, int dt){
    float cameraSpeed = sensitivity * dt;
    yaw += cameraSpeed * xoffset;
    pitch += cameraSpeed * yoffset;
    if (pitch < -0)
        pitch = 0.01;
    if (pitch > M_PI)
        pitch = M_PI - 0.01;
    updateCamera();
}

void OrbitCamera::scrollMoveCamera(float soffset, int dt){
    float cameraSpeed = -dt * sensitivity;
    ratio += cameraSpeed * soffset;
    if (ratio < 0)
        ratio = 0;
    updateCamera();
}

void OrbitCamera::updateCamera(){
    cameraPos.x = cameraTarget.x + ratio * cos(yaw) * sin(pitch);
    cameraPos.z = cameraTarget.z + ratio * cos(pitch);
    cameraPos.y = cameraTarget.y + ratio * sin(yaw) * sin(pitch);
    cameraFront = glm::normalize(cameraTarget - cameraPos);
}
