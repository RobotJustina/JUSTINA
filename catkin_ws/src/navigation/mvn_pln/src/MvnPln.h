#pragma once
#include <iostream>
#include "geometry_msgs/Point.h"

class MvnPln
{
public:
    MvnPln();
    ~MvnPln();

    bool GetClose(std::string location);
    bool GetClose(float goalX, float goalY);
    bool GetClose(float goalX, float goalY, float goalTheta);
};
