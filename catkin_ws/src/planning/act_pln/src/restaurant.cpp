#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "std_msgs/Bool.h"
#include "string"

#define SM_INIT 0

#define SM_WAIT_FOR_INIT_COMMAND 10
#define SM_TRAINING_PHASE 20
#define SM_FOLLOWING_PHASE 30
#define SM_FOLLOWING_PAUSE 40
#define SM_FOLLOWING_CHECKPOINT 50
#define SM_FOLLOWING_GOALPOINT 60
#define SM_RETURN_HOME_COMMAND 70
#define SM_RETURN_HOME 80
#define SM_FINAL_STATE 200
#define SM_WAIT_FOR_LEGS_FOUND 110

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ACT_PLN-FOLLOW ME BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    ros::Rate loop(10);

    int c_point=0,i=1;
    int nextState = 0;
    bool fail = false;
    bool success = false;
    bool stop=false;
    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    validCommands.push_back("robot follow me");
    validCommands.push_back("stop");
    validCommands.push_back("continue");
    validCommands.push_back("table A");
    validCommands.push_back("table B");
    validCommands.push_back("table C");

    validCommands.push_back("goal");
    validCommands.push_back("return home");

    ros::Publisher pubFollow = n.advertise<std_msgs::Bool>("/hri/human_following/start_follow",1); 
	std_msgs::Bool startFollow;
    

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {
        
        case SM_INIT:
		{
			std::cout << "State machine: SM_INIT" << std::endl;	
	       	JustinaHRI::say("I'm ready for the restaurant test");
			sleep(2);
			//JustinaHRI::say("You can tell me one of the next commands: robot start, stop follow me, continue, checkpoint, goal, return to home");
			//sleep(2);
			JustinaHRI::say("I'm waiting for the Professional Waiter");
	       	nextState = SM_WAIT_FOR_INIT_COMMAND;
		}
        break;

        case SM_WAIT_FOR_INIT_COMMAND:
		{
				std::cout << "State machine: SM_WAIT_FOR_INIT_COMMAND" << std::endl;
            	if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
                	JustinaHRI::say("Please repeat the command");
            	else{
                	if(lastRecoSpeech.find("robot follow me") != std::string::npos)
                		nextState = SM_TRAINING_PHASE;
                	else
            			nextState = SM_WAIT_FOR_INIT_COMMAND;    		
            		}
		}
        break;
       
        case SM_TRAINING_PHASE:
		{
			std::cout << "State machine: SM_TRAINING_PHASE" << std::endl;
			JustinaHRI::say("Human, please put in front of me");
	    	JustinaHRI::enableLegFinder(true);
			nextState=SM_WAIT_FOR_LEGS_FOUND;	    
		}
		break;

		case SM_WAIT_FOR_LEGS_FOUND:
			std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
            if(JustinaHRI::frontalLegsFound())
            {
                std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                JustinaHRI::say("You can tell me one of the next commands: continue, stop, table A, table B, table C");
                sleep(1);	
                JustinaNavigation::addLocation("kitchen");
                JustinaHRI::say("I will start to follow you human, please walk");
        		nextState = SM_FOLLOWING_PHASE;
            }
        break;

        case SM_FOLLOWING_PHASE:
		{
			std::cout << "State machine: SM_FOLLOWING_PHASE" << std::endl;
			stop=false;
	   		JustinaHRI::startFollowHuman();
			ros::spinOnce();
			
			while(!stop){
	                	if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
							if(lastRecoSpeech.find("stop") != std::string::npos){
								stop=true;
		                    	JustinaHRI::stopFollowHuman();
								nextState = SM_FOLLOWING_PAUSE;
								JustinaHRI::say("I stopped");
		                    	sleep(1);
		                    	JustinaHRI::say("I'm waiting for the continue commnad");
								

							}
							else if(lastRecoSpeech.find("table a") != std::string::npos){
								stop=true;
								nextState = SM_FOLLOWING_CHECKPOINT;
							}
							else if(lastRecoSpeech.find("table b") != std::string::npos){
								stop=true;
								nextState = SM_FOLLOWING_CHECKPOINT;
							}
							else if(lastRecoSpeech.find("table c") != std::string::npos){
								stop=true;
								nextState = SM_FOLLOWING_CHECKPOINT;
							}
		                   
							else{
								std::cout << "Command ERROR!" << std::endl;
								JustinaHRI::say("Please repeat the command");
								}
							}
						}		

        }
        break;


	case SM_FOLLOWING_PAUSE:
		{
		std::cout << "State machine: SM_FOLLOWING_PAUSE" << std::endl;
		stop=false;
        while(!stop){
            if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find("continue") != std::string::npos){
                            std::cout << "Command CONTINUE!" << std::endl;
                            stop=true;
                            nextState = SM_TRAINING_PHASE;
                            JustinaHRI::say("OK");
                    }
                    
                    else{
                            std::cout << "Command ERROR!" << std::endl;
                            JustinaHRI::say("Please repeat the command");
                        	}
    					}
					}
		}
		break;

	 case SM_FOLLOWING_TABLE_A:
		{         
			std::cout << "State machine: SM_FOLLOWING_TABLE_A" << std::endl;
						
					JustinaHRI::say("I saved the checkpoint 1");
					JustinaNavigation::addLocation("checkpoint_1");	
					i++;					
				}
			else if (i==2){                            
					JustinaHRI::say("I saved the checkpoint 2");
					JustinaNavigation::addLocation("checkpoint_2");
					i++;				
				}
			else if (i==3){                     
					JustinaHRI::say("I saved the checkpoint 3");
					JustinaNavigation::addLocation("checkpoint_3");
					i++;				
				}		

			nextState = SM_FOLLOWING_PHASE;
		}               
        break;
	
	case SM_FOLLOWING_GOALPOINT:
		{
            std::cout << "State machine: SM_FOLLOWING_GOALPOINT" << std::endl;
            JustinaHRI::stopFollowHuman();
            JustinaNavigation::addLocation("goal_point");
            JustinaHRI::say("I saved the goal location");
			std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_FollowMe") << std::endl;
			nextState = SM_RETURN_HOME_COMMAND;
		}
        break;
	
	case SM_RETURN_HOME_COMMAND:
		{
		JustinaHRI::say("I'm waiting the command to back home ");
                if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
                		if(lastRecoSpeech.find("return home") != std::string::npos)
								nextState = SM_RETURN_HOME;
                
		}
        break;

        case SM_RETURN_HOME:
		{
        JustinaHRI::say("I will go to arena");
        if(!JustinaNavigation::getClose("checkpoint_3",200000))
        	if(!JustinaNavigation::getClose("checkpoint_3",200000))
        		JustinaNavigation::getClose("checkpoint_3",200000);
		JustinaHRI::say("I arrived to checkpoint 3");
	    
	    if(!JustinaNavigation::getClose("checkpoint_2",200000))
        	if(!JustinaNavigation::getClose("checkpoint_2",200000))
        		JustinaNavigation::getClose("checkpoint_2",200000);
		JustinaHRI::say("I arrived to checkpoint 2");
		
		if(!JustinaNavigation::getClose("checkpoint_1",200000))
        	if(!JustinaNavigation::getClose("checkpoint_1",200000))
        		JustinaNavigation::getClose("checkpoint_1",200000);
		JustinaHRI::say("I arrived to checkpoint 1");
		
		if(!JustinaNavigation::getClose("arena",200000))
        	if(!JustinaNavigation::getClose("arena",200000))
        		JustinaNavigation::getClose("arena",200000);
		JustinaHRI::say("I arrived to arena");
			nextState=SM_FINAL_STATE;

		}

            break;

        case SM_FINAL_STATE:
        {
        	std::cout << "State machine: SM_FINAL_STATE" << std::endl;
        }    
        
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}




