#include <iostream>
#include "justina_tools/JustinaTasks.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST JUST FOR TEST BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    int nextState = 1;
    bool success = false;

    while(ros::ok() && !success){
        switch(nextState){
        case 1:
            success = JustinaTasks::placeBlockOnBlock(0.092, true, "blue", true);
            if(success)
                nextState = 2;
            break;
        default:
            std::cout << "NavigTest.->Somebody very stupid programmed this shit. " << std::endl;
            break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
