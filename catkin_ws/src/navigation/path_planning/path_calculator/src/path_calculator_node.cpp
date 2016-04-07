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

nav_msgs::Path lastCalcPath;

bool callbackWaveFront(navig_msgs::PathFromMap::Request &req, navig_msgs::PathFromMap::Response &resp)
{
    return PathCalculator::WaveFront(req.map, req.start_pose, req.goal_pose, resp.path);   
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING PATH CALCULATOR BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "path_calculator");
    ros::NodeHandle n;
    ros::ServiceServer srvPathWaveFront = n.advertiseService("path_calculator/wave_front", callbackWaveFront);
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

