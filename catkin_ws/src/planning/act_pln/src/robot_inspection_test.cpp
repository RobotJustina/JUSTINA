#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"

#define SM_INIT 0
#define SM_WAIT_FOR_DOOR 10
#define SM_NAVIGATE_TO_KITCHEN 20
#define SM_WAITING_FOR_KITCHEN 30
#define SM_WAIT_FOR_COMMAND 40
#define SM_REPEAT_COMMAND 50
#define SM_PARSE_SPOKEN_COMMAND 60
#define SM_FINAL_STATE 70
#define SM_WAIT_FOR_CONFIRMATION 80
#define SM_PARSE_SPOKEN_CONFIRMATION 90
#define SM_WAIT_FOR_QR 15
#define SM_WAIT_FOR_INSPECTION 25

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
    validCommands.push_back("continue");
    validCommands.push_back("robot yes");
    validCommands.push_back("robot no");

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {
            case SM_INIT:
                JustinaHRI::say("I'm waiting for the door to be open");
                nextState = SM_WAIT_FOR_DOOR;
                break;
            case SM_WAIT_FOR_DOOR:
                if(!JustinaNavigation::obstacleInFront())
                    nextState = SM_NAVIGATE_TO_KITCHEN;
                break;
            case SM_NAVIGATE_TO_KITCHEN:
                JustinaHRI::say("I'm going to inspection stage");
                if(!JustinaNavigation::getClose("inspection", 180000))
                    if(!JustinaNavigation::getClose("inspection", 180000))
                        if(!JustinaNavigation::getClose("inspection", 180000))
                JustinaHRI::say("I've arrive to inspection stage");
                nextState = SM_WAIT_FOR_COMMAND;
                break;
            case SM_WAIT_FOR_COMMAND:
                sleep(4);
                JustinaHRI::say("I'm waiting for a command");
                if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 12000))
                {
                    nextState = SM_WAIT_FOR_COMMAND;
                }
                else
                {
                    std::cout << "Parsing word..." << std::endl;
                    nextState = SM_PARSE_SPOKEN_COMMAND;
                }
                break;
            case SM_REPEAT_COMMAND:
                JustinaHRI::say("Please repeat the command");
                sleep(2);
                nextState = SM_WAIT_FOR_COMMAND;
                break;
            case SM_PARSE_SPOKEN_COMMAND:
                if(lastRecoSpeech.find("head") != std::string::npos)
                {
                    JustinaHRI::say("Do you mean: move your head?");
                    sleep(4);
                    nextState = SM_WAIT_FOR_CONFIRMATION;
                }
                else if(lastRecoSpeech.find("left") != std::string::npos)
                {
                    JustinaHRI::say("Do you mean: move your left arm?");
                    sleep(4);
                    nextState = SM_WAIT_FOR_CONFIRMATION;
                }
                else if(lastRecoSpeech.find("right") != std::string::npos)
                {
                    JustinaHRI::say("Do you mean: move your right arm?");
                    sleep(4);
                    nextState = SM_WAIT_FOR_CONFIRMATION;
                }
                else if(lastRecoSpeech.find("both arms") != std::string::npos)
                {
                    JustinaHRI::say("Do you mean: move both amrs");
                    sleep(4);
                    nextState = SM_WAIT_FOR_CONFIRMATION;
                }
                else if(lastRecoSpeech.find("kitchen") != std::string::npos)
                {
                    JustinaHRI::say("Do you mean: go to the kitchen");
                    sleep(4);
                    nextState = SM_WAIT_FOR_CONFIRMATION;
                }
                else if(lastRecoSpeech.find("livingroom") != std::string::npos)
                {
                    JustinaHRI::say("Do you mean: go to the livingroom");
                    sleep(4);
                    nextState = SM_WAIT_FOR_CONFIRMATION;
                }
                else if(lastRecoSpeech.find("bedroom") != std::string::npos)
                {
                    JustinaHRI::say("Do you mean: go to the bedroom");
                    sleep(4);
                    nextState = SM_WAIT_FOR_CONFIRMATION;
                }
                else
                {
                    JustinaHRI::say("I can't recognize this command");
                    sleep(4);
                    nextState = SM_REPEAT_COMMAND;
                }
                break;
            case SM_WAIT_FOR_CONFIRMATION:
                JustinaHRI::say("I'm waiting for confirmation");
                sleep(4);
                if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 9000))
                    if(lastRecoSpeech.find("yes") != std::string::npos)
                        nextState = SM_WAIT_FOR_INSPECTION;
                else
                    nextState = SM_WAIT_FOR_COMMAND;
                break;
            case SM_WAIT_FOR_INSPECTION:
                JustinaHRI::say("I'm waiting for inspection");
                sleep(4);
                if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 20000))
                    if(lastRecoSpeech.find("continue") != std::string::npos)
                        nextState = SM_WAIT_FOR_QR;
                else
                    nextState = SM_WAIT_FOR_INSPECTION;
                break;
            case SM_WAIT_FOR_QR:
                JustinaHRI::say("I'm waiting for a QR code");
                sleep(6);

                //if(!Justina::QR)
                //    nextState = SM_WAIT_FOR_QR;
                //else
                JustinaHRI::say("I've scan the QR. I can continue");
                sleep(4);
                JustinaHRI::say("I can continue");
                nextState = SM_FINAL_STATE;
                break;
            case SM_FINAL_STATE:
                JustinaHRI::say("I'm going to the exit");
                sleep(4);
                if(!JustinaNavigation::getClose("entrance", 180000))
                    if(!JustinaNavigation::getClose("entrance", 180000))
                        if(!JustinaNavigation::getClose("entrance", 180000))
                success = true;
                nextState = 1000;
                break;
            default:
                sleep(15);
                JustinaHRI::say("I've finish the inspection");
                fail = true;
                success = true;
                break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
