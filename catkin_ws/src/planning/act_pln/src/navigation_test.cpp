#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"

#define SM_INIT 0
#define SM_WAIT_FOR_COMMAND 10
#define SM_ASK_REPEAT_COMMAND 20
#define SM_PARSE_SPOKEN_COMMAND 30
#define SM_MOVE_HEAD 40
#define SM_MOVE_LEFT_ARM 50
#define SM_MOVE_RIGHT_ARM 60
#define SM_MOVE_BOTH_ARMS 70
#define SM_NAVIGATE_TO_KITCHEN 80
#define SM_WAITING_FOR_KITCHEN 90
#define SM_NAVIGATE_TO_LIVINGROOM 100
#define SM_WAITING_FOR_LIVINGROOM 110
#define SM_NAVIGATE_TO_BEDROOM 120
#define SM_WAITING_FOR_BEDROOM 130
#define SM_FOLLOW_ME 140
#define SM_STOP_FOLLOWING 150
#define SM_FINAL_STATE 200

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ACT_PLN BY MARCOSOFT..." << std::endl;
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
    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    validCommands.push_back("move your head");
    validCommands.push_back("move your left arm");
    validCommands.push_back("move your right arm");
    validCommands.push_back("move both arms");
    validCommands.push_back("go to the kitchen");
    validCommands.push_back("go to the livingroom");
    validCommands.push_back("go to the bedroom");
    validCommands.push_back("follow me");
    validCommands.push_back("stop following me");

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {
        case SM_INIT:
            JustinaHRI::say("I'm ready for the navigation test");
            nextState = SM_WAIT_FOR_COMMAND;
            break;
        case SM_WAIT_FOR_COMMAND:
            if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000))
                nextState = SM_ASK_REPEAT_COMMAND;
            else
                nextState = SM_PARSE_SPOKEN_COMMAND;
            break;
        case SM_ASK_REPEAT_COMMAND:
            JustinaHRI::say("Please repeat the command");
            nextState = SM_WAIT_FOR_COMMAND;
            break;
        case SM_PARSE_SPOKEN_COMMAND:
            if(lastRecoSpeech.find("head") != std::string::npos)
                nextState = SM_MOVE_HEAD;
            else if(lastRecoSpeech.find("left") != std::string::npos)
                nextState = SM_MOVE_LEFT_ARM;
            else if(lastRecoSpeech.find("right") != std::string::npos)
                nextState = SM_MOVE_RIGHT_ARM;
            else if(lastRecoSpeech.find("both arms") != std::string::npos)
                nextState = SM_MOVE_BOTH_ARMS;
            else if(lastRecoSpeech.find("kitchen") != std::string::npos)
                nextState = SM_NAVIGATE_TO_KITCHEN;
            else if(lastRecoSpeech.find("livingroom") != std::string::npos)
                nextState = SM_NAVIGATE_TO_LIVINGROOM;
            else if(lastRecoSpeech.find("bedroom") != std::string::npos)
                nextState = SM_NAVIGATE_TO_BEDROOM;
            break;
        case SM_MOVE_HEAD:
            JustinaManip::hdGoTo(0.5, 0, 5000);
            JustinaManip::hdGoTo(-0.5, 0, 5000);
            JustinaManip::hdGoTo(0, 0, 5000);
            nextState = SM_WAIT_FOR_COMMAND;
            break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
