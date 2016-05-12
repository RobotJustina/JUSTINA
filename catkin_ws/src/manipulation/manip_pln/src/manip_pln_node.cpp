#include <iostream>
#include "ros/ros.h"
#include "ManipPln.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING MANIPULATION PLANNER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "manip_pln");
    ros::NodeHandle n;

    ManipPln manipPln;
    manipPln.setNodeHandle(&n);
    manipPln.spin();

    return 0;
}
