#include <iostream>
#include "ros/ros.h"
#include "JustinaHardware.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ACT_PLN BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::SetNodeHandle(&n);
    ros::Rate loop(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
}
