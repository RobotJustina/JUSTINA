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
#define SM_NAVIGATE_TO_INSPECTION 20
#define SM_WAITING_FOR_KITCHEN 30
#define SM_WAIT_FOR_COMMAND 40 
#define SM_REPEAT_COMMAND 50 
#define SM_PARSE_SPOKEN_COMMAND 60
#define SM_FINAL_STATE 70 
#define SM_FINAL_STATE_2 55
#define SM_WAIT_FOR_CONFIRMATION 80 
#define SM_PARSE_SPOKEN_CONFIRMATION 90
#define SM_WAIT_FOR_INSPECTION 25 
#define SM_ROBOT_STOP 35 
#define SM_MOVE_HEAD 45 

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ACT_PLN BY MARCOSOFT..." << std::endl; //cout
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
    validCommands.push_back("continue");
    validCommands.push_back("robot yes");
    validCommands.push_back("robot no");
    validCommands.push_back("robot stop");
    validCommands.push_back("move your head");

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {
            case SM_INIT:
                JustinaHRI::say("I am waiting for the door to be open");
                nextState = SM_WAIT_FOR_DOOR;
                break;
            case SM_WAIT_FOR_DOOR:
                if(!JustinaNavigation::obstacleInFront())
                    nextState = SM_NAVIGATE_TO_INSPECTION;
                break;
            case SM_NAVIGATE_TO_INSPECTION:
                JustinaHRI::say("I can see that the door is open, I am going to the arena");
                sleep(3);
                if(!JustinaNavigation::getClose("arena", 180000))
                    if(!JustinaNavigation::getClose("arena", 180000))
                        if(!JustinaNavigation::getClose("arena", 180000))
                JustinaHRI::say("I have arrived to inspection point");

				nextState=SM_WAIT_FOR_COMMAND;

            	sleep(2);
            	JustinaHRI::say("You can tell me this command:");
            	sleep(2);
            	JustinaHRI::say("continue, and I am going to exit point");
            	sleep(1);
            	nextState=SM_WAIT_FOR_COMMAND;
                break;
            case SM_WAIT_FOR_COMMAND:
                JustinaHRI::say("I am going to stay at this point till you say a command");
                sleep(2);
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
                    JustinaHRI::say("Did you say move your head?");
                    nextState = SM_WAIT_FOR_CONFIRMATION;
                }
               	else if(lastRecoSpeech.find("continue") != std::string::npos)
                {
                    JustinaHRI::say("I am going to continue the robot inspection");
                    sleep(2);
                    nextState = SM_FINAL_STATE;
                }
                else
                {
                    JustinaHRI::say("I can't recognize this command");
                    sleep(2);
                    nextState = SM_REPEAT_COMMAND;
                }
                break;
            case SM_WAIT_FOR_CONFIRMATION:
                JustinaHRI::say("I am waiting for confirmation");
                sleep(2);
                if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 9000))
                    if(lastRecoSpeech.find("yes") != std::string::npos)

			{
			JustinaHardware::setHeadGoalPose(0.5, 0.0);
                    	sleep(1);
                    	JustinaHardware::setHeadGoalPose(-0.5, 0.0);
                    	sleep(1);
                    	JustinaHardware::setHeadGoalPose(0.0, 0.0);
                       	JustinaHRI::say("I am waiting for continue command");

					nextState = SM_WAIT_FOR_COMMAND;

                       	sleep(3);
			nextState = SM_WAIT_FOR_COMMAND;
			}
				if(lastRecoSpeech.find("no") != std::string::npos)
                 nextState = SM_WAIT_FOR_COMMAND;
                break;     
            case SM_ROBOT_STOP:
                         JustinaHardware::stopRobot();
                sleep(3);
                nextState = SM_FINAL_STATE;
                break;
            case SM_FINAL_STATE:
                JustinaHRI::say("I am going to the exit point");
                sleep(4);
                if(!JustinaNavigation::getClose("table", 180000))
                    if(!JustinaNavigation::getClose("table", 180000))
                        if(!JustinaNavigation::getClose("table", 180000))
                        success = true;
                nextState = SM_FINAL_STATE_2;
                break;
            case SM_FINAL_STATE_2:
                sleep(5);
                JustinaHRI::say("I have finished robot inspection test");
                fail = true;
                break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
