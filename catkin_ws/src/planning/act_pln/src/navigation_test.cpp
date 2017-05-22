#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"

#define SM_INIT 3
#define SM_WAIT_FOR_DOOR 14
#define SM_GOTO_A 15
#define SM_GOTO_B 92
#define SM_GOTO_FOLLOW 65
#define SM_TRAIN_FOLLOW 35
#define SM_START_FOLLOW 89
#define SM_WAIT_FOR_STOP_SIGNAL 79
#define SM_STOP_RECEIVED 32
#define SM_TRY_OPEN_DOOR 38
#define SM_WAIT_FOR_LEGS_FOUND 46
#define SM_FINAL_STATE 100

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

    int nextState = SM_INIT;
    bool fail = false;
    bool success = false;
    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    validCommands.push_back("robot stop");
    validCommands.push_back("robot stop follow me");
    validCommands.push_back("robot stop following me");
    validCommands.push_back("stop");
    validCommands.push_back("stop follow me");
    validCommands.push_back("stop following me");
    std::string waypoint1 = "waypoint_a";
    std::string waypoint2 = "waypoint_b";
    std::string waypointFollow = "waypoint_c";
    std::string frontdoor = "entrance";

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {
        case SM_INIT:
            boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
            JustinaHRI::say("I start the navigation test");
            nextState = SM_WAIT_FOR_DOOR;
            break;
        case SM_WAIT_FOR_DOOR:
            if(!JustinaNavigation::obstacleInFront())
                nextState = SM_GOTO_A;
            break;
        case SM_GOTO_A:
            JustinaHRI::say("I am going to the exitdoor");
            std::cout << "NavigTest.->First try to move" << std::endl;
            if(!JustinaNavigation::getClose(waypoint1, 10000))
                JustinaNavigation::getClose(waypoint1, 10000);
            if(!JustinaNavigation::getClose(waypoint1, 180000))
            {
                std::cout << "NavigTest.->Second try to move" << std::endl;
                if(!JustinaNavigation::getClose(waypoint1, 180000))
                {
                    std::cout << "NavigTest.->Third try to move" << std::endl;
                    if(!JustinaNavigation::getClose(waypoint1, 180000))
		            {
			             JustinaHRI::say("I cannot arrive to the check point. Please move the obstacle");
		            }
                }
            }
            JustinaHRI::say("I've arrived to exitdoor");
            nextState = SM_TRY_OPEN_DOOR;
            break;
        case SM_GOTO_B:
            JustinaHRI::say("I'm going to the corridor");
            std::cout << "NavigTest.->First try to move" << std::endl;
            if(!JustinaNavigation::getClose(waypoint2, 180000))
            {
                std::cout << "NavigTest.->Second try to move" << std::endl;
                if(!JustinaNavigation::getClose(waypoint2, 180000))
                {
                    std::cout << "NavigTest.->Third try to move" << std::endl;
                    JustinaNavigation::getClose(waypoint2, 180000);
                }
            }
            JustinaHRI::say("I've arrived to the corridor");
            nextState = SM_GOTO_FOLLOW;
            break;
        case SM_GOTO_FOLLOW:
            JustinaHRI::say("I'm going to search for a person in the third spot");
            std::cout << "NavigTest.->First try to move" << std::endl;
            if(!JustinaNavigation::getClose(waypointFollow, 180000))
            {
                std::cout << "NavigTest.->Second try to move" << std::endl;
                if(!JustinaNavigation::getClose(waypointFollow, 180000))
                {
                    std::cout << "NavigTest.->Third try to move" << std::endl;
                    JustinaNavigation::getClose(waypointFollow, 180000);
                }
            }
            JustinaHRI::say("I've arrived to the third spot");
            nextState = SM_TRAIN_FOLLOW;
            break;
        case SM_TRAIN_FOLLOW:
            std::cout << "NavigTest.->Starting training for follow" << std::endl;
            JustinaHRI::say("Dear human. Please stand in front of me.");
            JustinaHRI::say("Please, stand close to me.");
            std::cout << "NavigText.->Starting leg finder :D " << std::endl;
            JustinaHRI::enableLegFinder(true);
            nextState = SM_WAIT_FOR_LEGS_FOUND;
            break;
        case SM_WAIT_FOR_LEGS_FOUND:
            if(JustinaHRI::frontalLegsFound())
            {
                std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                nextState = SM_START_FOLLOW;
            }
            break;
        case SM_START_FOLLOW:
            JustinaHRI::say("I have trained you.");
            JustinaHRI::say("For stop following you, please say");
            JustinaHRI::say("Robot, stop following me");
            JustinaHRI::say("Now I am going to follow you. ");
            JustinaHRI::say("Please start walking.");
            JustinaHRI::startFollowHuman();
            nextState = SM_WAIT_FOR_STOP_SIGNAL;
            break;
        case SM_WAIT_FOR_STOP_SIGNAL:
            std::cout << "NavigTest.->Waiting for stop command" << std::endl;
            if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 5000))
                nextState = SM_STOP_RECEIVED;
            break;
        case SM_STOP_RECEIVED:
            std::cout << "NavigTest.->Stop signal received. " << std::endl;
            JustinaHRI::say("O.K. I will stop following you");
            JustinaHRI::stopFollowHuman();
            JustinaHRI::say("I will return to waypoint three");
            if(!JustinaNavigation::getClose(waypointFollow, 180000))
            {
                std::cout << "NavigTest.->Second try to move" << std::endl;
                if(!JustinaNavigation::getClose(waypointFollow, 180000))
                {
                    std::cout << "NavigTest.->Third try to move" << std::endl;
                    JustinaNavigation::getClose(waypointFollow, 180000);
                }
            }

            if(!JustinaNavigation::getClose(waypoint2, 180000))
            {
                std::cout << "NavigTest.->Second try to move" << std::endl;
                if(!JustinaNavigation::getClose(waypoint2, 180000))
                {
                    std::cout << "NavigTest.->Third try to move" << std::endl;
                    JustinaNavigation::getClose(waypoint2, 180000);
                }
            }

            if(!JustinaNavigation::getClose("initial", 180000))
            {
                std::cout << "NavigTest.->Second try to move" << std::endl;
                if(!JustinaNavigation::getClose("initial", 180000))
                {
                    std::cout << "NavigTest.->Third try to move" << std::endl;
                    JustinaNavigation::getClose("initial", 180000);
                }
            }
            break;
        case SM_TRY_OPEN_DOOR:
            std::cout << "NavigTest.->Trying to open door" << std::endl;
            nextState = -1;
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
