#include <iostream>
#include "ros/ros.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ACT_PLN BY MARCOSOFT..." << std::endl;
    ros::NodeHandle n;
    ros::Rate loop(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
}
