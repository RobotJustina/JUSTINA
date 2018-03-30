#include <iostream>
#include <vector>
#include <climits>
#include <cmath>
#include "ros/ros.h"
#include "navig_msgs/PathFromMap.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "tf/tf.h"
#include "PathCalculator.h"

ros::Publisher pubMapGrown;

bool callbackWaveFrontFromMap(navig_msgs::PathFromMap::Request &req, navig_msgs::PathFromMap::Response &resp)
{
    return PathCalculator::WaveFront(req.map, req.start_pose, req.goal_pose, resp.path);
}

bool callbackAStarFromMap(navig_msgs::PathFromMap::Request &req, navig_msgs::PathFromMap::Response &resp)
{
    std::cout << "Reciving path calculatro request ------------------------------" << std::endl;
    bool success = PathCalculator::AStar(req.map, req.start_pose, req.goal_pose, resp.path);
    pubMapGrown.publish(req.map);
    if(success)
    {
        resp.path = PathCalculator::SmoothPath(resp.path);
    }
    return success;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING PATH CALCULATOR BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "path_calculator");
    ros::NodeHandle n;
    ros::ServiceServer srvPathWaveFrontFromMap = n.advertiseService("path_calculator/wave_front_from_map", callbackWaveFrontFromMap);
    ros::ServiceServer srvPathAStarFromMap = n.advertiseService("path_calculator/a_star_from_map", callbackAStarFromMap);
    pubMapGrown = n.advertise<nav_msgs::OccupancyGrid>("path_calculator/grown_map", 1); 
    ros::Rate loop(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}

