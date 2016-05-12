#include <iostream>
#include "ros/ros.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING RIGHT ARM LOW LEVEL CONTROL BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "ra_control");

    return 0;
}
