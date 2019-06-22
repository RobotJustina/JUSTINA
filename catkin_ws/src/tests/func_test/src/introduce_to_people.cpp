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
    std::vector<std::string> ids;
    ids.push_back("person");
    bool isFound;
    JustinaTasks::POSE poseRecog = JustinaTasks::NONE;

    while(ros::ok() && !fail && !success){
        switch(nextState){
        case 1:
            std::cout << "trying find a person yolo " << std::endl;
            JustinaTasks::introduceOneToPeople("Alex", "kitchen", -1, -1);
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
