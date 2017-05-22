#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"

#define SM_INIT 0
#define SM_SHOW_FRAMES 10
#define SM_TRAINING_OBJET 20


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBJECT TRAINING..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    ros::Rate loop(10);

    int nextState = 0;
    bool fail = false;
    bool success = false;
    std::string name; 
    std::string confirmation;

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {
            case SM_INIT:
                std::cout << "Name objet: " <<;
                std::getline (std::cin,name); 
                nextState = SM_SHOW_FRAMES;
                break;
            case SM_SHOW_FRAMES:
                //JustinaVision::
                //
                std::cout << "Press enter when the image be in focus" <<;
                std::getline(std::cin, confirmation);
                nextState = SM_TRAINING_OBJET;
                break;
            case SM_TRAINING_OBJET:
                //call to function to training
                std::cout << name << "has been training." << "!\n";
                nextState = SM_INIT;
                break;

        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
