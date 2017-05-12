#include <iostream>
#include "ros/ros.h"
#include "vision_msgs/DetectObjects.h"
//#include "justina_tools/JustinaTasks.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR GRASP OBJECT BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "grasp_pln");
    ros::NodeHandle n;
    //JustinaTasks::setNodeHandle(&n);

    ros::ServiceClient cltDetectObjectsPCA = n.serviceClient<vision_msgs::DetectObjects>("/detect_object/PCA_calculator");
    vision_msgs::DetectObjects srv;

    ros::Rate loop(10);

    while(ros::ok())
    {
        if(!cltDetectObjectsPCA.call(srv))
        {
            std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
            return false;
        }

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
