
#include <iostream>
#include <vector>
#include <climits>
#include <cmath>
#include <cstdlib>
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
    static bool NearnessToObstacles(nav_msgs::OccupancyGrid& map, float distOfInfluence, int*& resultPotentials);
    static nav_msgs::Path SmoothPath(nav_msgs::Path& path, float weight_data = 0.1, float weight_smooth = 0.9, float tolerance = 0.00001);
};
