
#include <iostream>
#include <vector>
#include <climits>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"

class PathCalculator
{
public:
    PathCalculator();
    ~PathCalculator();

    static bool WaveFront(nav_msgs::OccupancyGrid& map, geometry_msgs::Pose& startPose, geometry_msgs::Pose& goalPose,
                          nav_msgs::Path& resultPath);
    static bool WaveFront(nav_msgs::OccupancyGrid& map, geometry_msgs::Pose& startPose, geometry_msgs::Pose& goalPose,
                          int*& resulWaveFront);
    static bool AStar(nav_msgs::OccupancyGrid& map, geometry_msgs::Pose& startPose, geometry_msgs::Pose& goalPose,
                         nav_msgs::Path& resultPath);
    static nav_msgs::OccupancyGrid GrowObstacles(nav_msgs::OccupancyGrid& map, float growDist);
    static bool Brushfire(nav_msgs::OccupancyGrid& map, int*& resultPotentials);
};
