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

    while(ros::ok() && !fail && !success){
        JustinaTasks::placeObject(true, 0.15);
        boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
        JustinaTasks::placeObject(false, 0.10);
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
