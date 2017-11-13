#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "abstractmodel.h"

#define EPSILON 0.000001

class Triangle: public AbstractModel
{
public:
    Triangle(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3);
    ~Triangle();
    virtual bool rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection);
};

#endif // TRIANGLE_H
