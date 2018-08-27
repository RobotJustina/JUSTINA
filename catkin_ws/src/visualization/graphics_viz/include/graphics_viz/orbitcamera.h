#ifndef ORBITCAMERA_H
#define ORBITCAMERA_H
#include "camera.h"

class OrbitCamera: public Camera{
public:
    OrbitCamera();
    void mouseMoveCamera(float xoffset, float yoffset, int dt);
    void scrollMoveCamera(float soffset, int dt);
    void updateCamera();
    void setCameraTarget(glm::vec3 cameraTarget){
        this->cameraTarget = cameraTarget;
    }
    glm::vec3 getCameraTarget(){
        return this->cameraTarget;
    }

private:
    glm::vec3 cameraTarget;
    float ratio;
};

#endif // ORBITCAMERA_H
