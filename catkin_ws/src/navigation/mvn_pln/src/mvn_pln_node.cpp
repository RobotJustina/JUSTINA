#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "justina_tools/JustinaNavigation.h"
#include "MvnPln.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING MOVING PLANNER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "mvn_pln");
    ros::NodeHandle n;
    ros::Rate loop(10);
    
    JustinaNavigation::setNodeHandle(&n);
    MvnPln mvnPln;

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
}
