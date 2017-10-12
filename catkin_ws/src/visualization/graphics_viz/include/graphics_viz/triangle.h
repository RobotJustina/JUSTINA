#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "abstractmodel.h"

class Triangle: public AbstractModel
{
public:
    Triangle(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3);
    ~Triangle();
};

#endif // TRIANGLE_H
