#include <iostream>
#include "ros/ros.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LBG_NODE BY MARCOSOFT" << std::endl;
    ros::init(argc, argv, "lgb_clustering");
    ros::NodeHandle n;
    ros::Subscriber subCloudDownsampled;
    ros::Rate loop(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    
    return 0;
}
