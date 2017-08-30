#include <iostream>
#include "ros/ros.h"
#include "navig_msgs/PathFromMap.h"
#include "tileadaptor.hpp"

int mapSizeX = 10, mapSizeY = 10;
std::vector<std::vector<char> > map;
TileAdaptor adaptor({mapSizeX, mapSizeY}, [&map](const Vectori& vec){return map[vec.x][vec.y] != '#';});
Pathfinder pathfinder(adaptor, 100.f /*weight*/);

bool callbackThetaAlgorithm(navig_msgs::PathFromMap::Request &req, navig_msgs::PathFromMap::Response &resp)
{
    
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING THETA PATH FINDER BY MONSE..." << std::endl;
    ros::init(argc, argv, "theta_path_finder");
    ros::NodeHandle n;
    ros::ServiceServer srvPath = n.advertiseService("path_calculator/a_star_from_map", callbackThetaAlgorithm);
    ros::Rate loop(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
