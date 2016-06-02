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
    validCommands.push_back("start follow me");
    validCommands.push_back("stop follow me");
    validCommands.push_back("continue follow me");
    validCommands.push_back("this is a checkpoint");
    validCommands.push_back("this is a goal location");
    validCommands.push_back("return to home");

    

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
            	if(lastRecoSpeech.find("follow me") != std::string::npos)
                	nextState = SM_TRAINING_PHASE;
		else  if(lastRecoSpeech.find("return home") != std::string::npos)
			nextState = SM_RETURN_HOME;
		}
            break;

        case SM_TRAINING_PHASE:
		{
		std::cout << "TrainingPhase State" << std::endl;
	    	JustinaHRI::say("You can tell me one of the next commands: stop follow me, continue follow me, this is a checkpoint, this is a goal location, return to home");	
	        sleep(4);	          
		JustinaHRI::say("I will start to follow you human");
		//JustinaNavigation::addlocation("arena ");
            	nextState = SM_FOLLOWING_PHASE;
		}
            break;

        case SM_FOLLOWING_PHASE:
		{
		std::cout << "FollowPhase State" << std::endl;
		stop=false;
	    	ros::Publisher pubFollow = n.advertise<std_msgs::Bool>("/hri/human_following/start_follow",1); 
	    	std_msgs::Bool startFollow;
	    	startFollow.data=true;
		pubFollow.publish(startFollow);
		ros::spinOnce();

		while(!stop){
                	if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
					if(lastRecoSpeech.find("stop follow me") != std::string::npos){
						std::cout << "Command PAUSE!" << std::endl;
                            			stop=true;
						startFollow.data=false;
					        nextState = SM_FOLLOWING_PAUSE;

					}
					else if(lastRecoSpeech.find("this is a checkpoint") != std::string::npos){
						std::cout << "Command CHECKPOINT!" << std::endl;
				        	stop=true;
						nextState = SM_FOLLOWING_CHECKPOINT;
					}
                        		else if(lastRecoSpeech.find("this is a goal location") != std::string::npos){
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
                                        if(lastRecoSpeech.find("continue follow me") != std::string::npos){
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
				//if(JustinaNavigation::addLocation("Checkpoint_1" )){
					JustinaHRI::say("I saved the checkpoint");
					i++;					
				//	}
				}
			else if (i==2){
                                //if(JustinaNavigation::addLocation("Checkpoint_2" )){
					JustinaHRI::say("I saved the checkpoint");
					i++;
				//}
				}
			else if (i==3){
                                //if(JustinaNavigation::addLocation("Checkpoint_3" )){
					JustinaHRI::say("I saved the checkpoint");
					i++;
				//}
				}
			

			nextState = SM_FOLLOWING_PHASE;
		}               
                break;
	
	case SM_FOLLOWING_GOALPOINT:
		{
                	std::cout << "Follow GoalPoint State!" << std::endl;
                	//JustinaNavigation::addLocation("Goalpoint" );
                	JustinaHRI::say("I saved the goal location");
			nextState = SM_RETURN_HOME_COMMAND;
		}
                break;
	
	case SM_RETURN_HOME_COMMAND:
		{
		 JustinaHRI::say("I'm waiting the command to back home ");
                if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000))
                        nextState = SM_ASK_REPEAT_COMMAND;
                else
                        nextState = SM_PARSE_SPOKEN_COMMAND;
		}
            break;

        case SM_RETURN_HOME:
		{
                JustinaHRI::say("I will go to arena");
                JustinaNavigation::getClose("checkpoint_3",200000);
		JustinaHRI::say("I arrived to checkpoint 3");
		JustinaNavigation::getClose("checkpoint_2",200000);
		JustinaHRI::say("I arrived to checkpoint 2");
		JustinaNavigation::getClose("checkpoint_1",200000);
		JustinaHRI::say("I arrived to checkpoint 1");
		JustinaNavigation::getClose("arena",200000);
		JustinaHRI::say("I arrived to arena");

		}

            break;

        
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
