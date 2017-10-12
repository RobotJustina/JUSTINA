#ifndef SPHERE_H
#define SPHERE_H
#include "abstractmodel.h"

class Sphere : public AbstractModel
{
public:
    Sphere(int slices, int stacks, float ratio = 1.0);
    ~Sphere();
};

#endif // SPHERE_H
