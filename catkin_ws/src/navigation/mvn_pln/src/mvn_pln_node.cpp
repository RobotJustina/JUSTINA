#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "justina_tools/JustinaNavigation.h"
#include "MvnPln.h"

int main(int argc, char** argv)
{
    std::string locationsFilePath = "";
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
            locationsFilePath = argv[++i];
    }
    
    std::cout << "INITIALIZING MOVING PLANNER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "mvn_pln");
    ros::NodeHandle n;
    ros::Rate loop(10);
    
    JustinaNavigation::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    MvnPln mvnPln;
    mvnPln.initROSConnection(&n);
    if(!mvnPln.loadKnownLocations(locationsFilePath))
        return 1;
    mvnPln.spin();

    return 0;
}
