#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTools.h"

enum STATE{
    SM_INIT,
    SM_SAY_WAIT_FOR_DOOR,
    SM_WAIT_FOR_DOOR,
    SM_GET_CLOSE_WAYPOINT,
    SM_FOLLOWING_INSTRUCTIONS,
    SM_WAIT_FOR_OPERATOR,
    SM_MEMORIZING_OPERATOR,
    SM_WAIT_FOR_LEGS_FOUND,
    SM_FOLLOWING_PHASE,
    SM_FINISH_TEST
};

STATE state;

std::string task("Visit my home");

std::stringstream ss;

bool fail = false, success = false;

std::string locations [5] = {"waypoint_1", "waypoint_2", "waypoint_3", "exitdoor", "waypoint_3"};
int locationsMaxAttemps [5] = {4, 4, 4, 4, 4};
int locationsAttemps = 0;
int currLocation = 0;

bool follow_start = false;

int minDelayAfterSay = 0;
int maxDelayAfterSay = 300;

int cont_z=3;
bool userConfirmation = false;
bool startSignalSM = true;
bool returnLocation = false;

int main(int argc, char ** argv)
{
    
    std::cout << "INITIALIZING VISIT MY HOME TEST ..." << std::endl;
    ros::init(argc, argv, "visit_my_home_test");
    ros::NodeHandle n;
    ros::Rate rate(10);
        
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
    
    // TODO Generate the pdf
    // JustinaTools::pdfStart("HelpMeCarry_Plans");

    while(ros::ok() && !fail && !success)
    {
        switch(state)
        {
            case SM_INIT:
                std::cout << task << " state machine: SM_INIT" << std::endl;
                if (startSignalSM) {
                    JustinaHRI::waitAfterSay("I am ready for the visit my home test", 5000, minDelayAfterSay);
                    JustinaHRI::loadGrammarSpeechRecognized("visit_my_home.xml");//load the grammar
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    // JustinaTools::pdfAppend("HelpMeCarry_Plans", "Starting the Help me Carry Test");
                    state = SM_SAY_WAIT_FOR_DOOR;
                }
                break;
            case SM_SAY_WAIT_FOR_DOOR:
                JustinaHRI::waitAfterSay("I am waiting for the door to be open", 4000, minDelayAfterSay);
                state = SM_WAIT_FOR_DOOR;
                break;
            case SM_WAIT_FOR_DOOR:
                std::cout << task << " state machine: SM_WAIT_OPEN_DOOR" << std::endl;
                if (!JustinaNavigation::obstacleInFront()){
                    JustinaHRI::waitAfterSay("Now I can see that the door is open", 4000);
                    JustinaNavigation::moveDist(1.0, 4000);
                    currLocation = 0;
                    locationsAttemps = 1;
                    returnLocation = false;
                    state = SM_GET_CLOSE_WAYPOINT;
                }
                break;

            case SM_GET_CLOSE_WAYPOINT:
                std::cout << task << " state machine: SM_GET_CLOSE_WAYPOINT" << std::endl;
                if(currLocation == 3 && !returnLocation) {
                    follow_start = false;
                    state = SM_FOLLOWING_INSTRUCTIONS;
                }
                else{
                    if(!(locationsAttemps > locationsMaxAttemps[currLocation])){
                    	if(currLocation == 1)
                    		JustinaNavigation::enableAvoidanceTypeObstacle(true);
                    	else
                    		JustinaNavigation::enableAvoidanceTypeObstacle(false);
                        ss.str("");
                        ss << "I will navigate to the " << locations[currLocation];
                        JustinaHRI::waitAfterSay(ss.str(), 3000, minDelayAfterSay);
                        if(!JustinaNavigation::getClose(locations[currLocation], 240000)){
                            locationsAttemps++;
                            break;
                        }
                        ss.str("");
                        ss << "I have reached the " << locations[currLocation];
                    }
                    else{
                        ss.str("");
                        ss << "I can not reached the " << locations[currLocation];
                        //JustinaHRI::waitAfterSay(ss.str(), 1000, minDelayAfterSay);
                    }
                    JustinaHRI::waitAfterSay(ss.str(), 3000, minDelayAfterSay);
                    currLocation++;
                    locationsAttemps = 1;
                    if(currLocation > 4)
                        state = SM_FINISH_TEST;
                }
                break;
            case SM_FOLLOWING_INSTRUCTIONS:
                std::cout << task << " state machine: SM_INSTRUCTIONS" << std::endl;
                JustinaHRI::waitAfterSay("Tell me, stop follow me, when we reached the waypoint 4", 6000, minDelayAfterSay);
                //JustinaHRI::waitAfterSay("Tell me, stop follow me", 6000, maxDelayAfterSay);
                JustinaHRI::waitAfterSay("Please tell me, follow me, for start following you", 6000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                cont_z = 0;
                state=SM_WAIT_FOR_OPERATOR;
                break;    
            case SM_WAIT_FOR_OPERATOR:
                std::cout << task << " state machine: SM_WAIT_FOR_OPERATOR" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("follow me" , 10000)){
                    state = SM_MEMORIZING_OPERATOR;
                    // JustinaTools::pdfAppend("HelpMeCarry_Plans", "Follow me command was recognized");
                }
                else                    
                    cont_z++;    		
                if(cont_z > 3){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("Please repeat the command", 5000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    cont_z=0;
                }
                break;
            case SM_MEMORIZING_OPERATOR:
                std::cout << task << " state machine: SM_MEMORIZING_OPERATOR" << std::endl;
                if(!follow_start){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("Human, please put in front of me", 3000, minDelayAfterSay);
                    // TODO create a PDF
                    // JustinaTools::pdfAppend("HelpMeCarry_Plans", "Starting the search of human");
                    JustinaHRI::enableLegFinder(true);
                }
                else
                    JustinaHRI::enableLegFinder(true);    
                state=SM_WAIT_FOR_LEGS_FOUND;
                break;
            case SM_WAIT_FOR_LEGS_FOUND:
                std::cout << task << " state machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
                if(JustinaHRI::frontalLegsFound()){
                    if(follow_start){
                        std::cout << task << " : Frontal legs found!" << std::endl;
                        // JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("I found you, please walk", 4000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        // JustinaTools::pdfAppend("HelpMeCarry_Plans", "Human was found with Hokuyo Laser");
                        JustinaHRI::startFollowHuman();
                        ros::spinOnce();
                        rate.sleep();
                        JustinaHRI::startFollowHuman();
                        state = SM_FOLLOWING_PHASE;
                    }
                    else{
                        std::cout << task << " : Frontal legs found!" << std::endl;
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("I found you, i will start to follow you human, please walk", 10000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//disable recognized speech
                        // JustinaTools::pdfAppend("HelpMeCarry_Plans", "Human was found with Hokuyo Laser");
                        JustinaHRI::startFollowHuman();
                        follow_start=true;
                        state = SM_FOLLOWING_PHASE;
                    }
                }
                break;
            case SM_FOLLOWING_PHASE:
                std::cout << task << " state machine: SM_FOLLOWING_PHASE" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("stop follow me" , 10000)){
                    // JustinaTools::pdfAppend("HelpMeCarry_Plans", "Here is the car command was recognized");
                    // JustinaTools::pdfAppend("HelpMeCarry_Plans", "Waiting for user confirmation");
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("is it the waypoint four, please tell me robot yes, or robot no", 10000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    JustinaHRI::waitForUserConfirmation(userConfirmation, 5000);
                    if(userConfirmation){
                        JustinaHRI::stopFollowHuman();
                        JustinaHRI::enableLegFinder(false);
                        JustinaKnowledge::addUpdateKnownLoc("waypoint_4");
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("I stopped", 2000, minDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        //JustinaTools::pdfAppend("HelpMeCarry_Plans", "Robot Yes command was recognized");
                        //JustinaTools::pdfAppend("HelpMeCarry_Plans", "Saving the car location");
                        locationsAttemps = 1;
                        currLocation = 3;
                        returnLocation = true;
                        state = SM_GET_CLOSE_WAYPOINT;
                        // cont_z=8;
                        break;
                    }
                    else{
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("Ok, please walk", 3000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        //JustinaTools::pdfAppend("HelpMeCarry_Plans", "Robot No command was recognized");
                    }
                    if(!JustinaHRI::frontalLegsFound()){
                        std::cout << task << " state machine: SM_FOLLOWING_PHASE -> Lost Human" << std::endl;
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("I lost you, please put in front of me again", 5500, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        //JustinaTools::pdfAppend("HelpMeCarry_Plans", "Human Lost");
                        //JustinaTools::pdfAppend("HelpMeCarry_Plans", "Starting the search of human");
                        JustinaHRI::stopFollowHuman();
                        JustinaHRI::enableLegFinder(false);
                        state=SM_MEMORIZING_OPERATOR;
                    }
                }
                break;
            case SM_FINISH_TEST:
                std::cout << task << " state machine: SM_FINISH_TEST" << std::endl;
                JustinaHRI::waitAfterSay("I have finished the test", 2500);
                //JustinaTools::pdfStop("HelpMeCarry_Plans");
                success = true;
                break;
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 1;
}
