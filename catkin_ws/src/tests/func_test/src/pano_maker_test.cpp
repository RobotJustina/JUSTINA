#include <iostream>
#include "justina_tools/JustinaTasks.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST JUST FOR TEST BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    bool fail = false; 
    bool success = false;

    std::vector<float> point;

    while(ros::ok() && !fail && !success){

        sensor_msgs::Image image;
        JustinaTasks::getPanoramic(0, -M_PI_4, -M_PI_4, -M_PI_4, M_PI_4 / 2, M_PI_4, image, 10000);
        JustinaVision::getRecogFromPano(image);


        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
