#ifndef CYLINDER_H
#define CYLINDER_H
#include "graphics_viz/abstractmodel.h"

class Cylinder: public AbstractModel
{
public:
    Cylinder(int slices, int stacks, float topRadius = 1.0, float bottomRadius = 1.0, float height = 1.0);
    ~Cylinder();
    virtual bool rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection);
};

#endif // CYLINDER_H
