#include <iostream>
#include "ros/ros.h"
#include "ManipPln.h"

int main(int argc, char** argv)
{
    std::string folder = "";
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
            folder = argv[++i];
    }
    
    std::cout << "INITIALIZING MANIPULATION PLANNER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "manip_pln");
    ros::NodeHandle n;

    ManipPln manipPln;
    manipPln.loadPredefinedPosesAndMovements(folder);
    manipPln.setNodeHandle(&n);
    manipPln.spin();

    return 0;
}
