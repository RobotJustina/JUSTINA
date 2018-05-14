#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaRepresentation.h"
#include "std_msgs/Bool.h"
#include "string"
#include "vision_msgs/FindPlane.h"

#define SM_INIT 0
#define SM_WAIT_FOR_INIT_COMMAND 10
#define SM_SEARCH_BAR 20
#define SM_SEARCH_WAVING 30
#define SM_ALIGN_WAVING 32
#define SM_FIND_PERSONS 33
#define SM_WAIT_FOR_TAKE_ORDER 35
#define SM_CLOSE_TO_CLIENT 36
#define SM_TAKE_ORDER 40
#define SM_ORDER_CONFIRM 50
#define SM_RETURN_BAR 60

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING RESTAURANT TEST..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaKnowledge::setNodeHandle(&n);
    JustinaRepresentation::setNodeHandle(&n);
   
    ros::Rate loop(10);
    
    int nextState = 0;
    bool fail = false;
    bool success = false;
    bool stop=false;
    bool findGesture = false;

    std::string lastRecoSpeech;
    std::string lastInteSpeech;
    int timeoutspeech = 10000;
    std::string obj1, obj2, typeOrder;

    float robot_y, robot_x, robot_a;    
    float gx_w, gy_w, gz_w;    
    float goalx, goaly, goala, angleError;    
    float dist_to_head;

    std::vector<std::string> attendCommands;
    attendCommands.push_back("justina take the order");
    attendCommands.push_back("justina wait");
    std::vector<std::string> confirmCommands;
    confirmCommands.push_back("justina yes");
    confirmCommands.push_back("justina no");

    std::stringstream ss;

    Eigen::Vector3d centroidGesture;

    bool findGestureOrAttendOrder = false;

    int attempsNavigation = 1;
    int maxAttempsNavigation = 2;

    int attempsSpeechReco = 1;
    int maxAttempsSpeechReco = 4;
    int attempsSpeechInt = 1;
    int maxAttempsSpeechInt = 2;
    int attempsWaitConfirmation = 1;
    int maxAttempsWaitConfirmation = 3;
    int attempsConfirmation = 1;
    int maxAttempsConfirmation = 3;

    std::string bar_search="";

    while(ros::ok() && !fail && !success){
        switch(nextState){
            case SM_INIT:
                std::cout << "State machine: SM_INIT" << std::endl;	
                JustinaManip::hdGoTo(0, 0, 2000);
                JustinaHRI::waitAfterSay("I'm ready for the restaurant test", timeoutspeech);
                nextState = SM_WAIT_FOR_INIT_COMMAND;
                break;

            case SM_WAIT_FOR_INIT_COMMAND:
                std::cout << "State machine: SM_WAIT_FOR_INIT_COMMAND" << std::endl;
				if(JustinaHRI::waitForSpecificSentence("justina start", timeoutspeech)){
                    JustinaHRI::waitAfterSay("I will search the bar", 3500);
                    nextState = SM_SEARCH_BAR;
                }else
                    nextState = SM_WAIT_FOR_INIT_COMMAND;
                break;

            case SM_SEARCH_BAR:
                std::cout << "State machine: SM_SERACH_BAR" << std::endl;
                JustinaTasks::findTable(bar_search);  
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                // TODO delimited the table to find
                if (bar_search.compare("center") == 0){
                    JustinaHRI::waitAfterSay("I see the bar in front of me", 10000);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    JustinaKnowledge::addUpdateKnownLoc("kitchen_bar", robot_a);
                }
                else if (bar_search.compare("right") == 0){
                    JustinaHRI::waitAfterSay("I see the bar in my right side", 10000);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    JustinaKnowledge::addUpdateKnownLoc("kitchen_bar", robot_a + M_PI_2);
                    JustinaNavigation::startMoveDistAngle(0.0, M_PI_2);
                }else if (bar_search.compare("left") == 0){
                    JustinaHRI::waitAfterSay("I see the bar in my left side", 10000);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    JustinaKnowledge::addUpdateKnownLoc("kitchen_bar", robot_a - M_PI_2);
                    JustinaNavigation::startMoveDistAngle(0.0, -M_PI_2);
                }else{
                    std::cout << "SM_SERACH_BAR: Bar default" << std::endl;
                    JustinaHRI::waitAfterSay("I see the bar in my left side", 10000);       
                }
                findGestureOrAttendOrder = true;
                JustinaHRI::waitAfterSay("I will find to the client", 5000);
                JustinaVision::startSkeletonFinding();
                nextState = SM_SEARCH_WAVING;     
                break;

            case SM_SEARCH_WAVING:
                std::cout << "State machine: SM_SEARCH_WAVING" << std::endl;
                // findGesture = JustinaTasks::turnAndRecognizeGesture("waving", -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2f, -0.2f, -0.2f, 0.0f, 0.0f, centroidGesture, "");
                findGesture = JustinaTasks::turnAndRecognizeGesture("waving", 0, 0, 0, -0.2f, -0.2f, -0.2f, 0.0f, 0.0f, 9.0, centroidGesture, "");
                if(findGesture){
                    nextState = SM_WAIT_FOR_TAKE_ORDER;
                    JustinaVision::stopSkeletonFinding();
                }else
                    nextState = SM_SEARCH_WAVING;
                break;

            case SM_WAIT_FOR_TAKE_ORDER:
                std::cout << "State machine: SM_WAIT_FOR_TAKE_ORDER" << std::endl;
                JustinaHRI::waitAfterSay("I noticed that somebody are asking for my service", 5000);
                if(JustinaHRI::waitForSpecificSentence(attendCommands, lastRecoSpeech, timeoutspeech)){
                    if(lastRecoSpeech.find("take the order") != std::string::npos){
                        JustinaHRI::waitAfterSay("Ok, I am going to approach to the client", 6000);
                        nextState = SM_CLOSE_TO_CLIENT;
                    }
                    else if(lastRecoSpeech.find("wait") != std::string::npos){
                        JustinaHRI::waitAfterSay("I will find to the client", 5000);
                        JustinaVision::startSkeletonFinding();
                        nextState = SM_SEARCH_WAVING;
                    }
                }
                break;

            case SM_CLOSE_TO_CLIENT:
                std::cout << "State machine: SM_CLOSE_TO_CLIENT" << std::endl;

                JustinaTools::transformPoint("/base_link", centroidGesture(0, 0), centroidGesture(1, 0) , centroidGesture(2, 0), "/map", gx_w, gy_w, gz_w);
                JustinaTasks::closeToGoalWithDistanceTHR(gx_w, gy_w, 1.5, 120000);
                dist_to_head = sqrt( pow(gx_w, 2) + pow(gy_w, 2));
                JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                // JustinaManip::startHdGoTo(atan2(gy_w, gx_w) - robot_a, atan2(gz_w - 1.6, dist_to_head)); 

                JustinaKnowledge::addUpdateKnownLoc("table_1", robot_a);
                
                JustinaHRI::waitAfterSay("Hello my name is Justina, and for today I will take your order, please tell me what order do you want me to bring", 12000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                attempsSpeechReco = 1;
                attempsSpeechInt = 1;
                 attempsConfirmation = 1;

                nextState = SM_TAKE_ORDER;
                break;

            case SM_TAKE_ORDER:
                std::cout << "State machine: SM_FIRST_ORDER" << std::endl;
                if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeoutspeech)){
                    if(JustinaRepresentation::stringInterpretation(lastRecoSpeech, lastInteSpeech)){
                        if(JustinaRepresentation::orderInterpeted(lastInteSpeech, typeOrder, obj1, obj2)){
                            // TODO Review what happend with object with two words
                            ss.str("");
                            ss << "Do you want ";
                            if(typeOrder.compare("take_order_beverage") == 0)
                                ss << obj1;
                            else if(typeOrder.compare("take_order_combo") == 0)
                                ss << obj1 << " and " << obj2;
                            ss << ", please tell me justina yes or justina no";
                            JustinaHRI::waitAfterSay(ss.str(), 10000);
                            nextState = SM_ORDER_CONFIRM;
                            break;
                        }
                    }
                    if(maxAttempsSpeechInt <= maxAttempsSpeechInt){
                        JustinaHRI::waitAfterSay("Sorry I did not understand you, please tell me what order do you want me to bring", 10000);
                        attempsSpeechInt++;
                        nextState = SM_TAKE_ORDER;
                    }
                    else{
                        JustinaHRI::waitAfterSay("Sorry I did not understand you, I'm going back to the kitchen bar", 12000);
                        attempsSpeechReco = 1;
                        attempsSpeechInt = 1;
                        findGestureOrAttendOrder = true;
                        nextState = SM_RETURN_BAR;
                    }
                }
                else{
                    if(attempsSpeechReco <= maxAttempsSpeechReco){
                        JustinaHRI::waitAfterSay("please tell me what order do you want me to bring", 10000);
                        attempsSpeechReco++;
                        nextState = SM_TAKE_ORDER;
                    }
                    else{
                        JustinaHRI::waitAfterSay("Sorry I did not understand you, I'm going back to the kitchen bar", 12000);
                        attempsSpeechReco = 1;
                        attempsSpeechInt = 1;
                        findGestureOrAttendOrder = true;
                        nextState = SM_RETURN_BAR;
                    }
                }
                break;

            case SM_ORDER_CONFIRM:
                std::cout << "State machine: SM_FIRST_ORDER_CONFIRM" << std::endl;
                attempsSpeechReco = 1;
                attempsSpeechInt = 1;
                if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, timeoutspeech)){
                    if(lastRecoSpeech.find("yes") != std::string::npos){
                        JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000);
                        findGestureOrAttendOrder = false;
                        nextState = SM_RETURN_BAR;
                    }
                    else{
                        if(attempsConfirmation <= maxAttempsConfirmation){
                            attempsConfirmation++;
                            JustinaHRI::waitAfterSay("Ok", 2000);
                            nextState = SM_TAKE_ORDER;
                        }
                        else{
                            JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000);
                            findGestureOrAttendOrder = false;
                            nextState = SM_RETURN_BAR;
                        }
                    }
                }
                else {
                    if(attempsWaitConfirmation <= maxAttempsWaitConfirmation){
                        attempsConfirmation++;
                        JustinaHRI::waitAfterSay(ss.str(), 10000);
                        nextState = SM_ORDER_CONFIRM;
                    }
                    else{
                        JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000);
                        findGestureOrAttendOrder = false;
                        nextState = SM_RETURN_BAR;
                    }
                }
                break;	

            case SM_RETURN_BAR:
                std::cout << "State machine: SM_RETURN_BAR" << std::endl;
                if(!JustinaNavigation::getClose("kitchen_bar", 120000)){
                    attempsNavigation++;
                    if(attempsNavigation <= maxAttempsNavigation)
                        break;
                    else{
                        JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                        JustinaKnowledge::getKnownLocation("kitchen_bar", goalx, goaly, goala);
                        angleError = goala - robot_a;
                        if(angleError > M_PI) angleError -= 2*M_PI;
                        if(angleError <= -M_PI) angleError += 2*M_PI;
                        JustinaNavigation::moveDistAngle(0.0, angleError, 3000);
                    }
                }
                JustinaHRI::waitAfterSay("I arrived to the kitchen bar", 2000);
                if(findGestureOrAttendOrder){
                    JustinaHRI::waitAfterSay("I will find to the client", 5000);
                    JustinaVision::startSkeletonFinding();
                    nextState=SM_SEARCH_WAVING;
                }
                else{
                    // TODO This is for attend the order in kitchen bar
                }
                break;    
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

