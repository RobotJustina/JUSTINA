#include <iostream>
#include <vector>
#include <climits>
#include <cmath>
#include "ros/ros.h"
#include "navig_msgs/PathFromMap.h"
#include "navig_msgs/PathFromAll.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "tf/tf.h"
#include "PathCalculator.h"

nav_msgs::Path lastCalcPath;

bool callbackWaveFrontFromMap(navig_msgs::PathFromMap::Request &req, navig_msgs::PathFromMap::Response &resp)
{
    bool success = PathCalculator::WaveFront(req.map, req.start_pose, req.goal_pose, resp.path);
    if(success)
        lastCalcPath = resp.path;
    return success;
}

bool callbackAStarFromMap(navig_msgs::PathFromMap::Request &req, navig_msgs::PathFromMap::Response &resp)
{
    bool success = PathCalculator::AStar(req.map, req.start_pose, req.goal_pose, resp.path);
    if(success)
        lastCalcPath = resp.path;
    return success;
}

bool callbackWaveFrontFromAll(navig_msgs::PathFromAll::Request &req, navig_msgs::PathFromAll::Response &resp)
{
    //TODO: Modify map according to what it is received in the point cloud
    bool success = PathCalculator::WaveFront(req.map, req.start_pose, req.goal_pose, resp.path);
    if(success)
        lastCalcPath = resp.path;
    return success;
}

bool callbackAStarFromAll(navig_msgs::PathFromAll::Request &req, navig_msgs::PathFromAll::Response &resp)
{
    //TODO: Modify map according to what it is received in the point cloud
    bool success = PathCalculator::AStar(req.map, req.start_pose, req.goal_pose, resp.path);
    if(success)
        lastCalcPath = resp.path;
    return success;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING PATH CALCULATOR BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "path_calculator");
    ros::NodeHandle n;
    ros::ServiceServer srvPathWaveFrontFromMap = n.advertiseService("path_calculator/wave_front_from_map", callbackWaveFrontFromMap);
    ros::ServiceServer srvPathAStarFromMap = n.advertiseService("path_calculator/a_star_from_map", callbackAStarFromMap);
    ros::ServiceServer srvPathWaveFrontFromAll = n.advertiseService("path_calculator/wave_front_from_all", callbackWaveFrontFromAll);
    ros::ServiceServer srvPathAStarFromAll = n.advertiseService("path_calculator/a_star_from_all", callbackAStarFromAll);
    ros::Publisher pubLastPath = n.advertise<nav_msgs::Path>("path_calculator/last_calc_path", 1);
    ros::Rate loop(10);

    while(ros::ok())
    {
        pubLastPath.publish(lastCalcPath);
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}

