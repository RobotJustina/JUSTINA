#ifndef FIRSTPERSONCAMERA_H
#define FIRSTPERSONCAMERA_H
#include "graphics_viz/camera.h"

class FirstPersonCamera: public Camera
{
public:
    FirstPersonCamera();
    void mouseMoveCamera(float xoffset, float yoffset, int dt);
    void scrollMoveCamera(float soffset, int dt);
    void updateCamera();
    void moveFrontCamera(bool dir, int dt);
    void moveRightCamera(bool dir, int dt);
};

#endif // FIRSTPERSONCAMERA_H
