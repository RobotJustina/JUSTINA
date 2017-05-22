#include <iostream>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LEFT ARM LOW LEVEL CONTROL BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "la_control");

    return 0;
}
