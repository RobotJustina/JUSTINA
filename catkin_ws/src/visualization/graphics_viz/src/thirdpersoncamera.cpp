#include "graphics_viz/thirdpersoncamera.h"

ThirdPersonCamera::ThirdPersonCamera(){
    pitch = 20.0f;
    yaw = 0.0f;
    angleAroundTarget = 0.0f;
    angleTarget = 0.0;
    distanceFromTarget = 1.0f;
    sensitivity = 0.01;
    cameraUp = glm::vec3(0.0, 1.0, 0.0);
    updateCamera();
}

void ThirdPersonCamera::mouseMoveCamera(float xoffset, float yoffset, int dt){
    // Camera controls
    float cameraSpeed = sensitivity * dt;

    // Calculate pitch
    pitch -= yoffset * cameraSpeed;
    // Calculate Angle Arround
    angleAroundTarget -= xoffset * cameraSpeed;
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
    float horizontalDistance = distanceFromTarget * glm::cos(glm::radians(pitch));
    //Calculate Vertical distance
    float verticalDistance = distanceFromTarget * glm::sin(glm::radians(pitch));

    //Calculate camera position
    float theta = angleTarget + angleAroundTarget;
    float offsetx = horizontalDistance * glm::sin(glm::radians(theta));
    float offsetz = horizontalDistance * glm::cos(glm::radians(theta));
    cameraPos.x = cameraTarget.x - offsetx;
    cameraPos.z = cameraTarget.z - offsetz;
    cameraPos.y = cameraTarget.y + verticalDistance;

    yaw = angleTarget - (180 + angleAroundTarget);

    if (distanceFromTarget < 0)
        cameraFront = glm::normalize(cameraPos - cameraTarget);
    else
        cameraFront = glm::normalize(cameraTarget - cameraPos);
}
