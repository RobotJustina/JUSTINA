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
#define SM_FIRST_ORDER 40
#define SM_FIRST_ORDER_CONFIRM 50
#define SM_RETURN_BAR 60

#define SM_FOLLOWING_PHASE 300
#define SM_FOLLOWING_PAUSE 400
#define SM_FOLLOWING_TABLE_1 500
#define SM_FOLLOWING_TABLE_2 600
#define SM_FOLLOWING_TABLE_3 70
#define SM_FOLLOWING_RETURN_KITCHEN 80
#define SM_FOLLOWING_RETURN_PAUSE 90
#define SM_ORDERING_PHASE 100
#define SM_FIRST_ORDER_WHICH_TABLE 110
#define SM_FIRST_ORDER_TABLE_A 120
#define SM_FIRST_ORDER_TABLE_B 130
#define SM_DELIVERING_PHASE 140
#define SM_DELIVERING_TAKING_ORDER 150
#define SM_DELIVERING_BEVERAGE 160
#define SM_DELIVERING_RETURN_KITCHEN 170
#define SM_FIRST_ORDER_RETURN_KITCHEN 180
#define SM_DELIVERING_PUT_ORDER 190
#define SM_FINAL_STATE 200
#define SM_WAIT_FOR_LEGS_FOUND 210
#define SM_WAIT_FOR_LEGS 220

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
   
    ros::Rate loop(10);
    
    std::string reco_sentence;
    std::vector<std::string> validItems;

    validItems.push_back("soda");
    validItems.push_back("milk");
    validItems.push_back("tea");
    validItems.push_back("beer");
    validItems.push_back("wine");
    validItems.push_back("chips");
    validItems.push_back("egg");
    validItems.push_back("eggs");
    validItems.push_back("candy");
    validItems.push_back("candies");
    validItems.push_back("paprika");
    validItems.push_back("apple");
    validItems.push_back("pumper");

    int nextState = 0;
    bool fail = false;
    bool success = false;
    bool stop=false;
    bool findGesture = false;

    std::string lastRecoSpeech;

    float robot_y, robot_x, robot_a;    
    float gx_w, gy_w, gz_w;    
    float dist_to_head;

    std::vector<std::string> confirmCommands;
    confirmCommands.push_back("justina take the order");
    confirmCommands.push_back("justina wait");

    std::stringstream ss;

    bool userConfirmation;
    
    Eigen::Vector3d centroidGesture;

    std::string bar_search="";

    while(ros::ok() && !fail && !success){
        switch(nextState){
            case SM_INIT:
                std::cout << "State machine: SM_INIT" << std::endl;	
                JustinaManip::hdGoTo(0, 0, 2000);
                JustinaHRI::waitAfterSay("I'm ready for the restaurant test", 10000);
                nextState = SM_WAIT_FOR_INIT_COMMAND;
                break;

            case SM_WAIT_FOR_INIT_COMMAND:
                std::cout << "State machine: SM_WAIT_FOR_INIT_COMMAND" << std::endl;
				if(JustinaHRI::waitForSpecificSentence("justina start", 10000)){
                    JustinaHRI::waitAfterSay("I will search the bar", 3500);
                    nextState = SM_SEARCH_BAR;
                }else
                    nextState = SM_WAIT_FOR_INIT_COMMAND;
                break;

            case SM_SEARCH_BAR:
                std::cout << "State machine: SM_SERACH_BAR" << std::endl;
                JustinaTasks::findTable(bar_search);  
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
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
                JustinaHRI::waitAfterSay("I will find to the client", 5000);
                JustinaVision::startSkeletonFinding();
                nextState = SM_SEARCH_WAVING;     
                break;

            case SM_SEARCH_WAVING:
                std::cout << "State machine: SM_SEARCH_WAVING" << std::endl;
                // findGesture = JustinaTasks::turnAndRecognizeGesture("waving", -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2f, -0.2f, -0.2f, 0.0f, 0.0f, centroidGesture, "");
                findGesture = JustinaTasks::turnAndRecognizeGesture("waving", 0, 0, 0, -0.2f, -0.2f, -0.2f, 0.0f, 0.0f, centroidGesture, "");
                if(findGesture){
                    nextState = SM_WAIT_FOR_TAKE_ORDER;
                    JustinaVision::stopSkeletonFinding();
                }else
                    nextState = SM_SEARCH_WAVING;
                break;

            case SM_WAIT_FOR_TAKE_ORDER:
                std::cout << "State machine: SM_WAIT_FOR_TAKE_ORDER" << std::endl;
                JustinaHRI::waitAfterSay("I noticed that somebody are asking for my service", 5000);
                if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, 10000)){
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
                JustinaManip::startHdGoTo(atan2(gy_w, gx_w) - robot_a, atan2(gz_w - 1.6, dist_to_head)); 

                JustinaHRI::waitAfterSay("Hi, I am Justina, I'm going to take you order", 10000);

                nextState = SM_FIRST_ORDER;
                
                /*locations = JustinaKnowledge::getKnownLocations();
                it_locations = locations.find(ss.str());

                if(it_locations != locations.end()){
                    it_locations->second[0];
                    JustinaTasks::closeToGoalWithDistanceTHR();
                }else
                    nextState = SM_FIRST_ORDER;*/
                break;

            case SM_FIRST_ORDER:
                std::cout << "State machine: SM_FIRST_ORDER" << std::endl;
                /*JustinaHRI::waitAfterSay("What is your order", 10000);	
                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                if(JustinaHRI::waitForSpecificSentence(startCommands, lastRecoSpeech, 10000)){
                    JustinaHRI::waitAfterSay("Do you want me", 10000);
                    JustinaHRI::waitAfterSay(lastRecoSpeech, 10000);
                    nextState=SM_FIRST_ORDER_CONFIRM;           
                }
                else{
                    contador_order++;
                    JustinaHRI::waitAfterSay("I am sorry, i can not understand you", 10000);
                    if(contador_order>4){
                        JustinaHRI::waitAfterSay("Ok, i will go to the bar location and i will be back with your order", 10000);
                        contador_order=0;
                        nextState=SM_FIRST_ORDER_CONFIRM;
                    }
                }*/
                break;

            case SM_FIRST_ORDER_CONFIRM:
                std::cout << "State machine: SM_FIRST_ORDER_CONFIRM" << std::endl;
                JustinaHRI::waitForUserConfirmation(userConfirmation, 7000);
                if(userConfirmation){
                    JustinaHRI::waitAfterSay("Ok, i will go to the bar location and i will be back with your order", 10000);
                    nextState = SM_RETURN_BAR;
                }
                else {
                     JustinaHRI::waitAfterSay("Ok", 10000);
                     SM_FIRST_ORDER;
                }
                break;	

            case SM_RETURN_BAR:
                std::cout << "State machine: SM_RETURN_BAR" << std::endl;
                if(!JustinaNavigation::getClose(0,0,0, 200000))
                    if(!JustinaNavigation::getClose(0,0,0, 200000))
                        JustinaNavigation::getClose(0,0,0, 200000);
                JustinaHRI::waitAfterSay("I arrived to bar location", 2000);
                JustinaHRI::waitAfterSay("I waiting for the next client", 2000);
                nextState=SM_SEARCH_WAVING;
            break;    
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

