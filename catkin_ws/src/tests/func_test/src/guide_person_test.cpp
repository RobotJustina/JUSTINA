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
    bool fail = false;
    bool success = false;
    bool isGuide;
    std::vector<std::string> zonesRes;
    zonesRes.push_back("bedroom");
    zonesRes.push_back("hallway");
    zonesRes.push_back("bathroom");

    while(ros::ok() && !fail && !success){
        switch(nextState){
        case 1:
            std::cout << "Guiding a person " << std::endl;
            isGuide = JustinaTasks::guideAPerson("kitchen", 240000, 1.2, true, zonesRes);
            if(!isGuide){
                std::cout << "Can not guide a person " << std::endl;
                nextState = 1;
            }
            else
                nextState = 2;
            break;
        default:
            std::cout << "NavigTest.->Somebody very stupid programmed this shit. " << std::endl;
            fail = true;
            break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
