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
//#include "string"

#define SM_INIT 0
#define SM_WAIT_FOR_OPERATOR 10
#define SM_MEMORIZING_OPERATOR 20
#define SM_WAIT_FOR_LEGS_FOUND 25
#define SM_FOLLOWING_PHASE 30
#define SM_BRING_GROCERIES 40
#define SM_BRING_GROCERIES_TAKE 41
#define SM_BAG_DELIVERY 50
#define SM_BAG_DELIVERY_PLACE 60
#define SM_LOOKING_HELP 70
#define SM_GUIDING_ASK 75
#define SM_GUIDING_HELP 80
#define SM_GUIDING_MEMORIZING_OPERATOR 90
#define SM_GUIDING_MEMORIZING_OPERATOR_SAY 91
#define SM_GUIDING_PHASE 100
#define SM_GUIDING_STOP 101
#define SM_GUIDING_CAR 102
#define SM_FINAL_STATE 110

bool stop=false;

void callbackLegsFoundRear(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data == true){
        std::cout << "Is the human behind? --> Yes" << std::endl;
        stop=false;
    }   
    else
    {
        std::cout << "Is the human behind? --> No" << std::endl;
        stop = true;
    }
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING HELP ME CARRY TEST..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(30);

    boost::posix_time::ptime prev;
    boost::posix_time::ptime curr;

    //int c_point=0,i=1;
    int nextState = 0;
    bool fail = false;
    bool success = false;
    float x, y ,z;

    std::string lastRecoSpeech;
    std::string location;
    std::vector<std::string> validCommandsStop;
    std::vector<std::string> validCommandsTake;
    validCommandsStop.push_back("here is the car");
    validCommandsStop.push_back("stop follow me");
    //places
    validCommandsTake.push_back("take this bag to the sofa");
    validCommandsTake.push_back("take this bag to the kitchen");
    validCommandsTake.push_back("take this bag to the bed");
    validCommandsTake.push_back("take this bag to the bedroom table");
    validCommandsTake.push_back("take this bag to the dinner table");
    validCommandsTake.push_back("take this bag to the shelf");
    validCommandsTake.push_back("take this bag to the bookcase");
    validCommandsTake.push_back("take this bag to the cabinet");
    validCommandsTake.push_back("take this bag to the t.v.	|");
    validCommandsTake.push_back("take this bag to the fridge");
    validCommandsTake.push_back("take this bag to the stove");
    validCommandsTake.push_back("get this bag to the sofa");
    validCommandsTake.push_back("get this bag to the kitchen");
    validCommandsTake.push_back("get this bag to the bed");
    validCommandsTake.push_back("get this bag to the bedroom table");
    validCommandsTake.push_back("get this bag to the dinner table");
    validCommandsTake.push_back("get this bag to the shelf");
    validCommandsTake.push_back("get this bag to the bookcase");
    validCommandsTake.push_back("get this bag to the cabinet");
    validCommandsTake.push_back("get this bag to the t.v.	|");
    validCommandsTake.push_back("get this bag to the fridge");
    validCommandsTake.push_back("get this bag to the stove");
    //validCommands.push_back("return home");
    //validCommands.push_back("help me");
    //validCommands.push_back("robot no");

    ros::Publisher pubLegsFoundRear = n.advertise<std_msgs::Bool>("/hri/leg_finder/enable_rear", 1);

    ros::Subscriber subLegsFoundRear = n.subscribe("/hri/leg_finder/legs_found_rear", 1, callbackLegsFoundRear);

    std_msgs::Bool startFollow;
    std_msgs::Bool hokuyoRear;
    bool userConfirmation = false;

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {

            case SM_INIT:

                std::cout << "State machine: SM_INIT" << std::endl;	
                JustinaHRI::waitAfterSay("I am ready for the help me carry test", 2000);
                nextState = SM_WAIT_FOR_OPERATOR;

                break;

            case SM_WAIT_FOR_OPERATOR:

                std::cout << "State machine: SM_WAIT_FOR_OPERATOR" << std::endl;
                JustinaHRI::waitAfterSay("Please, tell me, follow me for start following you", 3000);
                if(JustinaHRI::waitForSpecificSentence("follow me" , 15000))
                    nextState = SM_MEMORIZING_OPERATOR;
                else
                    nextState = SM_WAIT_FOR_OPERATOR;    		
                break;

            case SM_MEMORIZING_OPERATOR:

                std::cout << "State machine: SM_MEMORIZING_OPERATOR" << std::endl;
                JustinaHRI::waitAfterSay("Human, please put in front of me", 2500);
                JustinaHRI::enableLegFinder(true);
                nextState=SM_WAIT_FOR_LEGS_FOUND;	    
                break;

            case SM_WAIT_FOR_LEGS_FOUND:

                std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
                if(JustinaHRI::frontalLegsFound()){
                    std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                    JustinaHRI::waitAfterSay("I found you, i will start to follow you human, please walk and tell me, here is the car, when we reached the car location", 10000);
                    nextState = SM_FOLLOWING_PHASE;
                }


                break;

            case SM_FOLLOWING_PHASE:

                std::cout << "State machine: SM_FOLLOWING_PHASE" << std::endl;
                JustinaHRI::startFollowHuman();

                if(JustinaHRI::waitForSpecificSentence(validCommandsStop, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find("here is the car") != std::string::npos || lastRecoSpeech.find("stop follow me") != std::string::npos){
                        JustinaHRI::stopFollowHuman();
                        JustinaKnowledge::addUpdateKnownLoc("car_location");	
                        JustinaHRI::waitAfterSay("I stopped", 1500);
                        JustinaHRI::stopFollowHuman();
                        JustinaHRI::stopFollowHuman();
                        JustinaHRI::stopFollowHuman();
                        nextState = SM_BRING_GROCERIES;
                        break;
                    }
                }
                if(!JustinaHRI::frontalLegsFound()){
                    std::cout << "State machine: SM_FOLLOWING_PHASE -> Lost human!" << std::endl;
                    JustinaHRI::waitAfterSay("I lost you", 1500);
                }        

                break;

            case SM_BRING_GROCERIES:
                std::cout << "State machine: SM_BRING_GROCERIES" << std::endl; 
                JustinaHRI::stopFollowHuman();
                JustinaHRI::waitAfterSay("I am ready to help you, Please tell me, take this bag to some location", 3000);

                if(JustinaHRI::waitForSpecificSentence(validCommandsTake, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find("this bag to the sofa") != std::string::npos){
                        location = "sofa";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the bed") != std::string::npos){
                        location = "bed";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the bedroom") != std::string::npos){
                        location = "bedroom_table";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the bedroom table") != std::string::npos){
                        location = "bedroom_table";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the dinning room") != std::string::npos){
                        location = "dinner_table";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the dinner table") != std::string::npos){
                        location = "dinner_table";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the shelf") != std::string::npos){
                        location = "shelf";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the bookcase") != std::string::npos){
                        location = "bookcase";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the cabinet") != std::string::npos){
                        location = "cabinet";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the t.v.") != std::string::npos){
                        location = "tv";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the fridge") != std::string::npos){
                        location = "fridge";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the stove") != std::string::npos){
                        location = "stove";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                    else if(lastRecoSpeech.find("this bag to the hall") != std::string::npos){
                        location = "bookcase";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }

                    else if(lastRecoSpeech.find("this bag to the kitchen") != std::string::npos){
                        location = "kitchen_table";
                        nextState=SM_BRING_GROCERIES_TAKE;
                    }
                }
                break;

            case SM_BRING_GROCERIES_TAKE:    
                std::cout << "State machine: SM_BRING_GROCERIES_TAKE" << std::endl;
                JustinaHRI::waitAfterSay("Please, wait to the next command, for put the bag in my hand", 4000);
                JustinaManip::laGoTo("take", 4000);
                JustinaManip::startLaOpenGripper(0.6);
                JustinaManip::hdGoTo(0, -0.9, 3000);

                JustinaManip::getLeftHandPosition(x, y, z);
                boost::this_thread::sleep(boost::posix_time::milliseconds(200));
                std::cout << "helMeCarry.->Point(" << x << "," << y << "," << z << ")" << std::endl;
                JustinaVision::startHandDetectBB(x, y, z);
                prev = boost::posix_time::second_clock::local_time();
                curr = prev;
                JustinaHRI::waitAfterSay("Please put the bag in my hand", 4000);
                while(ros::ok() && !JustinaVision::getDetectionHandBB() && (curr - prev).total_milliseconds() < 30000){
                    loop.sleep();
                    ros::spinOnce();
                    curr = boost::posix_time::second_clock::local_time();
                }
                JustinaVision::stopHandDetectBB();
                JustinaHRI::waitAfterSay("Thank you", 1000);                    
                JustinaManip::startLaCloseGripper(0.4);
                JustinaManip::laGoTo("navigation", 10000);

                if(location == "sofa")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the sofa and i will be back to the car", 4000);
                else if(location == "bed")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the bed and i will be back to the car", 4000);
                else if(location == "bedroom_table")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the bedroom_table and i will be back to the car", 4000);
                else if(location == "dinner_table")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the dinner table and i will be back to the car", 4000);
                else if(location == "shelf")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the shelf and i will be back to the car", 4000);
                else if(location == "bookcase")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the bookcase and i will be back to the car", 4000);
                else if(location == "cabinet")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the cabinet and i will be back to the car", 4000);
                else if(location == "cabinet")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the cabinet and i will be back to the car", 4000);
                else if(location == "tv")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the t.v. and i will be back to the car", 4000);
                else if(location == "fridge")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the fridge and i will be back to the car", 4000);
                else if(location == "stove")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the stove and i will be back to the car", 4000);
                else if(location == "kitchen_table")
                    JustinaHRI::waitAfterSay("Ok human, I will go to the kitchen table and i will be back to the car", 4000);
                else
                    JustinaHRI::waitAfterSay("Ok human, I will go to the kitchen table and i will be back to the car", 4000);
                nextState=SM_BAG_DELIVERY;     

                break;

            case SM_BAG_DELIVERY:
                std::cout << "State machine: SM_BAG_DELIVERY" << std::endl;
                std::cout << "Location -> " << location << std::endl;
                JustinaNavigation::getClose(location,200000);
                JustinaHRI::waitAfterSay("I arrived", 1000);
                nextState=SM_BAG_DELIVERY_PLACE;

                break;

            case SM_BAG_DELIVERY_PLACE:
                std::cout << "State machine: SM_BAG_DELIVERY_PLACE" << std::endl;
                JustinaHRI::waitAfterSay("I will delivery the bags", 2000);
                if(!JustinaTasks::alignWithTable(0.35)){
                    JustinaNavigation::moveDist(0.15, 3000);
                    if(!JustinaTasks::alignWithTable(0.35)){
                        JustinaNavigation::moveDist(0.15, 3000);
                        JustinaTasks::alignWithTable(0.35);
                    }
                }

                if(!JustinaTasks::placeObject(true))
                    if(!JustinaTasks::placeObject(true))
                        JustinaTasks::placeObject(true);

                nextState=SM_LOOKING_HELP;

                break;

            case SM_LOOKING_HELP:
                std::cout << "State machine: SM_LOOKING_HELP" << std::endl;
                JustinaHRI::waitAfterSay("I will look for help", 2000);
                if(JustinaTasks::findPerson())
                    nextState=SM_GUIDING_ASK;
                else
                    JustinaHRI::waitAfterSay("I did not find anyone", 2000);    
                break;

            case SM_GUIDING_ASK:
                std::cout << "State machine: SM_GUIDING_ASK" << std::endl;
                JustinaHRI::waitAfterSay("Human, can you help me bring some bags please", 5000);
                JustinaHRI::waitForUserConfirmation(userConfirmation, 15000);
                if(userConfirmation)
                    nextState = SM_GUIDING_MEMORIZING_OPERATOR_SAY;
                else {
                    nextState = SM_LOOKING_HELP;
                    JustinaNavigation::moveDistAngle(0.0, 1.5708, 10000);
                }	    

                break;        

            case SM_GUIDING_MEMORIZING_OPERATOR_SAY:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR_SAY" << std::endl;
                JustinaHRI::waitAfterSay("I will guide you to the car location", 2000);
                location="car_location";
                JustinaHRI::enableLegFinderRear(true);
                nextState=SM_GUIDING_MEMORIZING_OPERATOR;

                break;

            case SM_GUIDING_MEMORIZING_OPERATOR:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR" << std::endl;
                JustinaHRI::waitAfterSay("Human, stand behind me", 3000);
                if(!stop){
                    JustinaHRI::waitAfterSay("Ok, let us go", 2500);
                    nextState=SM_GUIDING_PHASE;
                    JustinaNavigation::startGetClose(location);
                }        

                break;    

            case SM_GUIDING_PHASE:
                std::cout << "State machine: SM_GUIDING_PHASE" << std::endl;
                std::cout << "Location -> " << location << std::endl;
                if(stop)
                    nextState=SM_GUIDING_STOP;

                if(JustinaNavigation::isGlobalGoalReached())
                    nextState=SM_GUIDING_CAR;

                break;

            case SM_GUIDING_STOP:
                std::cout << "State machine: SM_GUIDING_STOP" << std::endl;
                JustinaHardware::stopRobot();
                JustinaHardware::stopRobot();
                JustinaHardware::stopRobot();
                ros::spinOnce();
                JustinaHRI::waitAfterSay("I lost you", 1500);
                nextState=SM_GUIDING_MEMORIZING_OPERATOR;
                break;

            case SM_GUIDING_CAR:
                std::cout << "State machine: SM_GUIDING_CAR" << std::endl;
                JustinaHRI::waitAfterSay("Here is the car, please help us", 2500);
                nextState=SM_FINAL_STATE;    
                break;

            case SM_FINAL_STATE:
                std::cout << "State machine: SM_FINAL_STATE" << std::endl;
                success = true;
                break;

        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

