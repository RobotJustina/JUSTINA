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
#define SM_ASK_REPEAT_COMMAND 90
#define SM_PARSE_SPOKEN_COMMAND 100
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
    validCommands.push_back("robot start");
    validCommands.push_back("stop follow me");
    validCommands.push_back("continue");
    validCommands.push_back("checkpoint");
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
		std::cout << "Initial State" << std::endl;	
       	JustinaHRI::say("I'm ready for the follow me test");
		sleep(4);
		JustinaHRI::say("I'm waiting for the voice command");
       	nextState = SM_WAIT_FOR_INIT_COMMAND;
		}
            break;

        case SM_WAIT_FOR_INIT_COMMAND:
		{
            	if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000))
                	nextState = SM_ASK_REPEAT_COMMAND;
            	else
                	nextState = SM_PARSE_SPOKEN_COMMAND;
		}
            break;

        case SM_ASK_REPEAT_COMMAND:
		{
            	//JustinaHRI::say("Please repeat the command");
            	nextState = SM_WAIT_FOR_INIT_COMMAND;
		}
            break;

        case SM_PARSE_SPOKEN_COMMAND:
		{
       	if(lastRecoSpeech.find("robot start") != std::string::npos)
                	nextState = SM_TRAINING_PHASE;
		 
		else 
			nextState = SM_WAIT_FOR_INIT_COMMAND;
		}
            break;

        case SM_TRAINING_PHASE:
		{
		std::cout << "TrainingPhase State" << std::endl;
	    JustinaHRI::say("You can tell me one of the next commands: robot start, stop follow me, continue, checkpoint, goal, return to home");	
	    sleep(4);	          
	    JustinaHRI::enableLegFinder(true);
		nextState=SM_WAIT_FOR_LEGS_FOUND;	    
		}
		break;

		case SM_WAIT_FOR_LEGS_FOUND:
			std::cout << "Followme.->Wait frontal legs" << std::endl;
            if(JustinaHRI::frontalLegsFound())
            {
                std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                JustinaHRI::say("I will start to follow you human");
				JustinaNavigation::addLocation("arena");
        		nextState = SM_FOLLOWING_PHASE;
            }
        break;

        case SM_FOLLOWING_PHASE:
		{
		std::cout << "FollowPhase State" << std::endl;
		stop=false;
	   	//startFollow.data=1;
		//pubFollow.publish(startFollow);
		JustinaHRI::startFollowHuman();
		ros::spinOnce();
			

		while(!stop){
                	if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
					if(lastRecoSpeech.find("stop follow me") != std::string::npos){
						std::cout << "Command PAUSE!" << std::endl;
                    	stop=true;
                    	JustinaHRI::stopFollowHuman();
						//startFollow.data=0;
						//pubFollow.publish(startFollow);
				        nextState = SM_FOLLOWING_PAUSE;

					}
					else if(lastRecoSpeech.find("checkpoint") != std::string::npos){
						std::cout << "Command CHECKPOINT!" << std::endl;
			        	stop=true;
						nextState = SM_FOLLOWING_CHECKPOINT;
					}
                        		else if(lastRecoSpeech.find("goal") != std::string::npos  && i>3){
						std::cout << "Command GOALPOINT!" << std::endl;
				        stop=true;
						nextState = SM_FOLLOWING_GOALPOINT;					
					}
					else
						std::cout << "Command ERROR!" << std::endl;
			}
		}		

            	}
            break;


	case SM_FOLLOWING_PAUSE:
		{
		std::cout << "FollowPause State" << std::endl;
		stop=false;
                

                while(!stop){
                        if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
                                        if(lastRecoSpeech.find("continue") != std::string::npos){
                                                std::cout << "Command CONTINUE!" << std::endl;
                                                stop=true;
                                                nextState = SM_FOLLOWING_PHASE;

                                        }
                                        
                                        else
                                                std::cout << "Command ERROR!" << std::endl;
                        }
                }
		}
		break;

	 case SM_FOLLOWING_CHECKPOINT:
		{         
			
			std::cout << "Follow Checkpoint State!" << std::endl;
			if (i==1){				
				
					JustinaHRI::say("I saved the checkpoint");
					JustinaNavigation::addLocation("checkpoint_1");	
					i++;					
				
				}
			else if (i==2){
                            
					JustinaHRI::say("I saved the checkpoint");
					JustinaNavigation::addLocation("checkpoint_2");
					i++;
				
				}
			else if (i==3){
                     
					JustinaHRI::say("I saved the checkpoint");
					JustinaNavigation::addLocation("checkpoint_3");
					i++;
				
				}
			

			nextState = SM_FOLLOWING_PHASE;
		}               
                break;
	
	case SM_FOLLOWING_GOALPOINT:
		{
            std::cout << "Follow GoalPoint State!" << std::endl;
            JustinaHRI::stopFollowHuman();
            JustinaNavigation::addLocation("goal_point");
            JustinaHRI::say("I saved the goal location");
			std::cout << system("rosrun map_server map_server -f home/marco/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_FollowMe") << std::endl;
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
        		if(!JustinaNavigation::getClose("checkpoint_3",200000))
		JustinaHRI::say("I arrived to checkpoint 3");
	    
	    if(!JustinaNavigation::getClose("checkpoint_2",200000))
        	if(!JustinaNavigation::getClose("checkpoint_2",200000))
        		if(!JustinaNavigation::getClose("checkpoint_2",200000))
		JustinaHRI::say("I arrived to checkpoint 2");
		
		if(!JustinaNavigation::getClose("checkpoint_1",200000))
        	if(!JustinaNavigation::getClose("checkpoint_1",200000))
        		if(!JustinaNavigation::getClose("checkpoint_1",200000))
		JustinaHRI::say("I arrived to checkpoint 1");
		
		if(!JustinaNavigation::getClose("arena",200000))
        	if(!JustinaNavigation::getClose("arena",200000))
        		if(!JustinaNavigation::getClose("arena",200000))
		JustinaHRI::say("I arrived to arena");

		}

            break;

        
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}




