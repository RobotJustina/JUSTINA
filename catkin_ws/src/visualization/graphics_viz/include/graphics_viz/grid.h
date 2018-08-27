#ifndef GRID_H
#define GRID_H
#include "abstractmodel.h"

class Grid: public AbstractModel
{
public:
    Grid(int gridSize, float cellSize);
    ~Grid();
    virtual bool rayPicking(glm::vec3 init, glm::vec3 end, glm::vec3 &intersection);
    void render(glm::mat4 parentTrans = glm::mat4());
};

#endif // GRID_H
