#include "graphics_viz/thirdpersoncamera.h"

ThirdPersonCamera::ThirdPersonCamera(){
    pitch = glm::radians(20.0f);
    yaw = 0.0f;
    angleAroundTarget = 0.0f;
    angleTarget = 0.0;
    distanceFromTarget = 1.0f;
    sensitivity = 0.001;
    cameraUp = glm::vec3(0.0, 0.0, 1.0);
    updateCamera();
}

void ThirdPersonCamera::mouseMoveCamera(float xoffset, float yoffset, int dt){
    // Camera controls
    float cameraSpeed = sensitivity * dt;

    // Calculate pitch
    pitch -= yoffset * cameraSpeed;
    // Calculate Angle Arround
    angleAroundTarget -= xoffset * cameraSpeed;
    if(pitch > M_PI / 2)
        pitch = M_PI / 2 - 0.01;
    if(pitch < -M_PI / 2)
        pitch = -M_PI / 2 + 0.01;
    updateCamera();
}

void ThirdPersonCamera::scrollMoveCamera(float soffset, int dt){
    // Camera controls
    float cameraSpeed = sensitivity * dt;
    // Calculate zoom
    float zoomLevel = soffset * cameraSpeed;
    distanceFromTarget -= zoomLevel;
    updateCamera();
}

void ThirdPersonCamera::updateCamera(){
    //Calculate Horizontal distance
    float horizontalDistance = distanceFromTarget * cos(pitch);
    //Calculate Vertical distance
    float verticalDistance = distanceFromTarget * sin(pitch);

    //Calculate camera position
    float theta = angleTarget + angleAroundTarget;
    float offsetx = horizontalDistance * sin(theta);
    float offsety = horizontalDistance * cos(theta);
    cameraPos.x = cameraTarget.x - offsetx;
    cameraPos.y = cameraTarget.y - offsety;
    cameraPos.z = cameraTarget.z + verticalDistance;

    yaw = angleTarget - (180 + angleAroundTarget);

    if (distanceFromTarget < 0)
        cameraFront = glm::normalize(cameraPos - cameraTarget);
    else
        cameraFront = glm::normalize(cameraTarget - cameraPos);
}
