#include <iostream>
#include <vector>
#include <climits>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

class PathCalculator
{
public:
    PathCalculator();
    ~PathCalculator();

    static bool WaveFront(navig_msgs::OccupancyGrid& map, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose, nav_msgs::Path resultPath);
};
