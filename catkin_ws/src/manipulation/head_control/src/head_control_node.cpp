#include <iostream>
#include "ros/ros.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING HEAD LOW LEVEL CONTROL BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "head_control");

    return 0;
}
