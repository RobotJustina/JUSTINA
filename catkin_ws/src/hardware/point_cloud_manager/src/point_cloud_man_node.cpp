#include <iostream>
#include <boost/filesystem/path.hpp>
#include "PcManNode.h"

int main(int argc, char** argv)
{
    std::string default_path = "";
    for (int i = 0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if (strParam.compare("--defpath") == 0)
            default_path = std::string(argv[i+1]);
	}
    if(!boost::filesystem::is_directory(default_path))
    {
        std::cout << "PointCloudMan.->Default path is invalid or does not exists :'(" << std::endl;
        std::cout << "PointCloudMan.->Default path for saving files is mandatory." << std::endl;
        std::cout << "PointCloudMan.-Usage: point_cloud_man_node --defpath /a/valid/existing/path/" << std::endl;
        return 1;
    }
    
    std::cout << "INITIALIZING POINT CLOUD MAN BY MARCOSOFT..." << std::endl;
    //ROS Initialization
    ros::init(argc, argv, "point_cloud_man");
    ros::NodeHandle n;
    PcManNode pcManNode;
    if(!pcManNode.InitNode(&n, default_path))
    {
        std::cout << "PointCloudMan.-> Cannot initialize node :'(" << std::endl;
        return 1;
    }

    pcManNode.spin();
	
    return 0;
}
