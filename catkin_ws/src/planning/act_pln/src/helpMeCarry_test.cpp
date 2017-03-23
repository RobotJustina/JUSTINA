#include <iostream>
#include "ros/ros.h"
//#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
//#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaTools.h"
//#include "justina_tools/JustinaVision.h"
#include "std_msgs/Bool.h"
//#include "string"

#define SM_INIT 0
#define SM_WAIT_FOR_OPERATOR 10
#define SM_MEMORIZING_OPERATOR 20
#define SM_WAIT_FOR_LEGS_FOUND 25
#define SM_FOLLOWING_PHASE 30
#define SM_BRING_GROCERIES 40
#define SM_BAG_PICKUP 50
#define SM_BAG_DELIVERY 60
#define SM_GUIDING_HELP 70
#define SM_GUIDING_MEMORIZING_OPERATOR 80
#define SM_GUIDING_PHASE 90
#define SM_FINAL_STATE 100

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING HELP ME CARRY TEST..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    //JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    //JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    //JustinaVision::setNodeHandle(&n);
    ros::Rate loop(10);

    //int c_point=0,i=1;
    int nextState = 0;
    bool fail = false;
    bool success = false;
    //bool stop=false;
    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    validCommands.push_back("follow me");
    validCommands.push_back("here is the car");
    //validCommands.push_back("continue");
    //validCommands.push_back("checkpoint");
    //validCommands.push_back("goal");
    //validCommands.push_back("return home");
    //validCommands.push_back("help me");
    //validCommands.push_back("robot no");

    ros::Publisher pubFollow = n.advertise<std_msgs::Bool>("/hri/human_following/start_follow",1); 
	std_msgs::Bool startFollow;
    

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {
        
        case SM_INIT:
		{
			std::cout << "State machine: SM_INIT" << std::endl;	
	       	JustinaHRI::say("I'm ready for the help me carry test");
			sleep(1);
			nextState = SM_WAIT_FOR_OPERATOR;
		}
        break;

        case SM_WAIT_FOR_OPERATOR:
		{
			std::cout << "State machine: SM_WAIT_FOR_OPERATOR" << std::endl;
            	if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
                	JustinaHRI::say("Please repeat the command");
            	else{
                	if(lastRecoSpeech.find("follow me") != std::string::npos)
                		nextState = SM_MEMORIZING_OPERATOR;
                	else
            			nextState = SM_WAIT_FOR_OPERATOR;    		
            		}
		}
        break;
       
        case SM_MEMORIZING_OPERATOR:
		{
			std::cout << "State machine: SM_MEMORIZING_OPERATOR" << std::endl;
			JustinaHRI::say("Human, please put in front of me");
	    	JustinaHRI::enableLegFinder(true);
			nextState=SM_WAIT_FOR_LEGS_FOUND;	    
		}
		break;

		case SM_WAIT_FOR_LEGS_FOUND:
		{
			std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
            if(JustinaHRI::frontalLegsFound())
            {
                std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                JustinaHRI::say("I found you");
                sleep(1);	
                JustinaHRI::say("I will start to follow you human");
        		nextState = SM_FOLLOWING_PHASE;
            }
        }    
        	
        break;

        case SM_FOLLOWING_PHASE:
		{
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
        }
        break;

        case SM_BRING_GROCERIES:
        {
        	nextState = SM_FINAL_STATE;
        }
        break;
	
        case SM_FINAL_STATE:
        {
        	std::cout << "State machine: SM_FINAL_STATE" << std::endl;

        }    
        break;

        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

