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
    ros::Rate loop(10);

    //int c_point=0,i=1;
    int nextState = SM_GUIDING_MEMORIZING_OPERATOR;
    bool fail = false;
    bool success = false;
    float x, y ,z;

    std::string lastRecoSpeech;
    std::string location;
    std::vector<std::string> validCommands;
    validCommands.push_back("follow me");
    validCommands.push_back("here is the car");
    validCommands.push_back("take this bag to the kitchen table");
    validCommands.push_back("robot yes");
    validCommands.push_back("robot no");
    validCommands.push_back("stop follow me");
    //validCommands.push_back("return home");
    //validCommands.push_back("help me");
    //validCommands.push_back("robot no");

    ros::Publisher pubFollow = n.advertise<std_msgs::Bool>("/hri/human_following/start_follow",1); 
    ros::Subscriber subLegsFoundRear = n.subscribe("/hri/leg_finder/legs_found_rear", 1, callbackLegsFoundRear);

	std_msgs::Bool startFollow;
    

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {
        
        case SM_INIT:
		
			std::cout << "State machine: SM_INIT" << std::endl;	
	       	JustinaHRI::say("I'm ready for the help me carry test");
			sleep(1);
			nextState = SM_WAIT_FOR_OPERATOR;
		
        break;

        case SM_WAIT_FOR_OPERATOR:
		
			std::cout << "State machine: SM_WAIT_FOR_OPERATOR" << std::endl;
            	if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
                	JustinaHRI::say("Please repeat the command");
            	else{
                	if(lastRecoSpeech.find("follow me") != std::string::npos)
                		nextState = SM_MEMORIZING_OPERATOR;
                	else
                		nextState = SM_WAIT_FOR_OPERATOR;    		
            		}
		
        break;
       
        case SM_MEMORIZING_OPERATOR:
		
			std::cout << "State machine: SM_MEMORIZING_OPERATOR" << std::endl;
			JustinaHRI::say("Human, please put in front of me");
	    	JustinaHRI::enableLegFinder(true);
			nextState=SM_WAIT_FOR_LEGS_FOUND;	    
		
		break;

		case SM_WAIT_FOR_LEGS_FOUND:
		
			std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
            if(JustinaHRI::frontalLegsFound())
            {
                std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                JustinaHRI::say("I found you");
                sleep(1);	
                JustinaHRI::say("I will start to follow you human");
        		nextState = SM_FOLLOWING_PHASE;
            }
            
        	
        break;

        case SM_FOLLOWING_PHASE:
		
			std::cout << "State machine: SM_FOLLOWING_PHASE" << std::endl;
			JustinaHRI::startFollowHuman();
			ros::spinOnce();
			
		   	if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
				if(lastRecoSpeech.find("here is the car") != std::string::npos)
                		JustinaHRI::stopFollowHuman();
                		JustinaKnowledge::addUpdateKnownLoc("car_location");	
		                JustinaHRI::say("I stopped");
		                sleep(1);	
		                nextState = SM_BRING_GROCERIES;	
            		}
            	else if(lastRecoSpeech.find("stop follow me") != std::string::npos){
                		JustinaHRI::stopFollowHuman();
                		JustinaKnowledge::addUpdateKnownLoc("car_location");	
		                JustinaHRI::say("I stopped");
		                sleep(1);	
		                nextState = SM_BRING_GROCERIES;	
            	}	
                if(!JustinaHRI::frontalLegsFound()){
                    std::cout << "State machine: SM_FOLLOWING_PHASE -> Lost human!" << std::endl;
                    JustinaHRI::say("I lost you");
                    sleep(5);    
                }        

        
        break;

        case SM_BRING_GROCERIES:
        	std::cout << "State machine: SM_BRING_GROCERIES" << std::endl;    
            JustinaHRI::say("I'm ready to help you");
            if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
                if(lastRecoSpeech.find("take this bag to the kitchen table") != std::string::npos){
                    location = "kitchen";
                    JustinaManip::raGoTo("take", 10000);
                    JustinaManip::startRaOpenGripper(0.6);
                    JustinaManip::hdGoTo(0, -0.9, 5000);
                    JustinaHRI::say("Please put the bag in my hand");
                    
                    JustinaManip::getRightHandPosition(x, y, z);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(200));
                    std::cout << "helMeCarry.->Point(" << x << "," << y << "," << z << ")" << std::endl;
                    JustinaVision::startHandDetectBB(x, y, z);
                    ros::Rate rate(10);
		    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
		    boost::posix_time::ptime curr = prev;
                    while(ros::ok() && !JustinaVision::getDetectionHandBB() && (curr - prev).total_milliseconds() < 30000){
                        rate.sleep();
                        ros::spinOnce();
			curr = boost::posix_time::second_clock::local_time();
                    }
                    JustinaVision::stopHandDetectBB();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                    JustinaHRI::say("Thank you");                    
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                    JustinaManip::startRaCloseGripper(0.4);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                    JustinaManip::raGoTo("navigation", 10000);
                    JustinaHRI::say("Ok human, I will go to the kitchen table and i will be back");
                    nextState=SM_BAG_DELIVERY;    
                }

            }
        	        
        break;

        case SM_BAG_DELIVERY:
        	std::cout << "State machine: SM_BAG_DELIVERY" << std::endl;
        	std::cout << "Location -> " << location << std::endl;
            if(!JustinaNavigation::getClose(location,200000))
               if(!JustinaNavigation::getClose(location,200000))
                    JustinaNavigation::getClose(location,200000);
            JustinaHRI::say("I arrived");
            nextState=SM_BAG_DELIVERY_PLACE;

        break;

        case SM_BAG_DELIVERY_PLACE:
        	std::cout << "State machine: SM_BAG_DELIVERY_PLACE" << std::endl;
            JustinaHRI::say("I will delivery the bags");
            JustinaTasks::alignWithTable(0.35);
            JustinaTasks::placeObject(false);
            nextState=SM_LOOKING_HELP;

        break;

        case SM_LOOKING_HELP:
            std::cout << "State machine: SM_LOOKING_HELP" << std::endl;
            JustinaHRI::say("I will look for help");
            
            if(JustinaTasks::findPerson())
                nextState=SM_GUIDING_MEMORIZING_OPERATOR;

            else{
                JustinaHRI::say("I did not find anyone");    
            }
        
        break;

        case SM_GUIDING_ASK:
        	std::cout << "State machine: SM_GUIDING_ASK" << std::endl;
            JustinaHRI::say("Human, can you help me bring some bags please?");
            if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
                    JustinaHRI::say("Please repeat the command");
                
            else{


                if(lastRecoSpeech.find("robot yes") != std::string::npos)
                    nextState = SM_GUIDING_MEMORIZING_OPERATOR;
                else{
                    nextState = SM_LOOKING_HELP;
		    		JustinaNavigation::moveDistAngle(0.0, 1.5708, 10000);
	 			}	    
            }  

        break;        

        case SM_GUIDING_MEMORIZING_OPERATOR_SAY:
            std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR_SAY" << std::endl;
            JustinaHRI::say("I will guide you to the car location");
            location="car_location";
            sleep(1);
            nextState=SM_GUIDING_MEMORIZING_OPERATOR;

        break;

        case SM_GUIDING_MEMORIZING_OPERATOR:
        	std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR" << std::endl;
            JustinaHRI::say("Human, stand behind me");
			//boost::this_thread::sleep(boost::time);
            sleep(5);            
            
            if(!stop){
	            JustinaHRI::say("Ok, let's go");
	            nextState=SM_GUIDING_PHASE;
    		}        

        break;    

        case SM_GUIDING_PHASE:
        	std::cout << "State machine: SM_GUIDING_PHASE" << std::endl;
            JustinaNavigation::startGetClose(location);
            std::cout << "Location -> " << location << std::endl;
            if(stop)
                nextState=SM_GUIDING_STOP;

            if(JustinaNavigation::isGlobalGoalReached())
            	nextState=SM_GUIDING_CAR;

        break;
        
        case SM_GUIDING_STOP:
        	std::cout << "State machine: SM_GUIDING_STOP" << std::endl;
            JustinaHardware::stopRobot();
            JustinaHRI::say("I lost you");
            nextState=SM_GUIDING_MEMORIZING_OPERATOR;

        break;

        case SM_GUIDING_CAR:
        	std::cout << "State machine: SM_GUIDING_CAR" << std::endl;
            JustinaHRI::say("Here is the car, please help us");
            nextState=SM_FINAL_STATE;    
	   
        break;

        case SM_FINAL_STATE:
            
        	std::cout << "State machine: SM_FINAL_STATE" << std::endl;
            
        break;

        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

