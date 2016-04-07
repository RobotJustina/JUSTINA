#include <iostream>
#include "PcManNode.h"

int main(int argc, char** argv)
{
    bool debugMode = false;
    for (int i = 0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if (strParam.compare("--debug") == 0)
            debugMode = true;
	}
    std::cout << "INITIALIZING POINT CLOUD MAN BY MARCOSOFT..." << std::endl;
    //ROS Initialization
    ros::init(argc, argv, "point_cloud_man");
    ros::NodeHandle n;
    PcManNode pcManNode;
    if(!pcManNode.InitNode(&n, debugMode))
    {
        std::cout << "PointCloudMan.-> Cannot initialize node :'(" << std::endl;
        return 1;
    }

    pcManNode.spin();
	
    return 0;
}
