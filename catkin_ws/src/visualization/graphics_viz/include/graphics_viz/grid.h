#ifndef GRID_H
#define GRID_H
#include "abstractmodel.h"

class Grid: public AbstractModel
{
public:
    Grid(int gridSize, float cellSize);
    ~Grid();

    void render();
};

#endif // GRID_H
