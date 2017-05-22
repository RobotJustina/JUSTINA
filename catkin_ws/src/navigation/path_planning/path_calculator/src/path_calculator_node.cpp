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

bool callbackWaveFrontFromMap(navig_msgs::PathFromMap::Request &req, navig_msgs::PathFromMap::Response &resp)
{
    return PathCalculator::WaveFront(req.map, req.start_pose, req.goal_pose, resp.path);
}

bool callbackAStarFromMap(navig_msgs::PathFromMap::Request &req, navig_msgs::PathFromMap::Response &resp)
{
    bool success = PathCalculator::AStar(req.map, req.start_pose, req.goal_pose, resp.path);
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
    ros::Rate loop(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}

