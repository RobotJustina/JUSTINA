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
#include "vision_msgs/FindPlane.h"


#define SM_INIT 0
#define SM_WAIT_FOR_INIT_COMMAND 10
#define SM_TRAINING_PHASE 20
#define SM_FOLLOWING_PHASE 30
#define SM_FOLLOWING_PAUSE 40
#define SM_FOLLOWING_TABLE_1 50
#define SM_FOLLOWING_TABLE_2 60
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
    
    vision_msgs::FindPlane fp;
    fp.request.name="";
    
    ros::ServiceClient client = n.serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/findPlane");
    

    validCommands.push_back("robot follow me");
    validCommands.push_back("stop");
    validCommands.push_back("continue");
    validCommands.push_back("table");
    //validCommands.push_back("table two");
    //validCommands.push_back("table three");
    validCommands.push_back("kitchen");


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
			sleep(1);
			JustinaHRI::say("I'm waiting for the Professional Waiter");
			JustinaNavigation::addLocation("kitchen");
	       	nextState = SM_WAIT_FOR_INIT_COMMAND;
		}
        break;

        case SM_WAIT_FOR_INIT_COMMAND:
		{
				std::cout << "State machine: SM_WAIT_FOR_INIT_COMMAND" << std::endl;
            	if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 10000))
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
			JustinaHRI::say("Hi Professional Waiter, please put in front of me");
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
                //JustinaHRI::say("You can tell me one of the next commands: stop follow me, continue, checkpoint, goal");
				JustinaHRI::say("I found you");
                sleep(1);	
                
                JustinaHRI::say("You can tell me one of the next commands: continue, stop, table 1, table 2, table 3");
                sleep(1);	                
                
                JustinaHRI::say("I will start to follow you Professional Waiter, please walk");
        		nextState = SM_FOLLOWING_PHASE;
            }
        }    
        	
        break;

		case SM_FOLLOWING_PHASE:
		{
			std::cout << "State machine: SM_FOLLOWING_PHASE" << std::endl;
			stop=false;
	   		JustinaHRI::startFollowHuman();
			ros::spinOnce();
			
			while(!stop){
						if(i>=3){
							nextState = SM_FOLLOWING_RETURN_KITCHEN;
							JustinaHRI::say("I saved the tables");
							JustinaHRI::stopFollowHuman();
							sleep(1);
							JustinaHRI::say("I will follow you to return kitchen");
							stop=true;
						}

						else{
							if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
								if(lastRecoSpeech.find("stop") != std::string::npos){
									stop=true;
			                    	JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_PAUSE;
									JustinaHRI::say("I stopped");
			                    	sleep(1);
			                    	JustinaHRI::say("I'm waiting for the continue commnad");
									

								}
								else if(lastRecoSpeech.find("table") != std::string::npos){
									stop=true;
									JustinaHRI::say("I stopped");
									JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_TABLE_1;
								}
	/*							else if(lastRecoSpeech.find("table two") != std::string::npos){
									stop=true;
									JustinaHRI::say("I stopped");
									JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_TABLE_2;
								}
								else if(lastRecoSpeech.find("table three") != std::string::npos){
									stop=true;
									JustinaHRI::say("I stopped");
									JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_TABLE_3;
								}
			                   
	*/							else{
									std::cout << "Command ERROR!" << std::endl;
									JustinaHRI::say("Please repeat the command");
									}
								}
							}
						}			

        }
        break;

        case SM_FOLLOWING_TABLE_1:
		{         

			JustinaHardware::setHeadGoalPose(0,0);
			std::cout << "State machine: SM_FOLLOWING_TABLE_1" << std::endl;

			if (i==1){

			JustinaHardware::setHeadGoalPose(1, -0.9);
			sleep(3);
				if(client.call(fp)){	
					JustinaHRI::say("I see the table in  my left side");
					JustinaNavigation::addLocation("table_1");
					sleep(1);
					JustinaNavigation::addLocation("i saved the table one");
					i++;
					}					
				else{
					JustinaHardware::setHeadGoalPose(-1, -0.9);
					sleep(3);
						if(client.call(fp)){	
							JustinaHRI::say("I see the table in  my right side");
							JustinaNavigation::addLocation("table_1");
							sleep(1);
							JustinaNavigation::addLocation("i saved the table one");
							i++;
						}
						else
							JustinaHRI::say("I can't see the table");
					}


			}
			
			else if (i==2){

			JustinaHardware::setHeadGoalPose(1, -0.9);
			sleep(3);
				if(client.call(fp)){	
					JustinaHRI::say("I see the table in  my left side");
					JustinaNavigation::addLocation("table_1");
					i++;
					sleep(1);
					JustinaNavigation::addLocation("i saved the table two");
					}					
				else{
					JustinaHardware::setHeadGoalPose(-1, -0.9);
					sleep(3);
						if(client.call(fp)){	
							JustinaHRI::say("I see the table in  my right side");
							JustinaNavigation::addLocation("table_1");
							sleep(1);
							JustinaNavigation::addLocation("i saved the table two");
							i++;
						}
						else
							JustinaHRI::say("I can't see the table");
					}


			}

			else if (i==3){

			JustinaHardware::setHeadGoalPose(1, -0.9);
			sleep(3);
				if(client.call(fp)){	
					JustinaHRI::say("I see the table in  my left side");
					JustinaNavigation::addLocation("table_1");
					i++;
					sleep(1);
					JustinaNavigation::addLocation("i saved the table three");
					}					
				else{
					JustinaHardware::setHeadGoalPose(-1, -0.9);
					sleep(3);
						if(client.call(fp)){	
							JustinaHRI::say("I see the table in  my right side");
							JustinaNavigation::addLocation("table_1");
							sleep(1);
							JustinaNavigation::addLocation("i saved the table three");
							i++;
						}
						else
							JustinaHRI::say("I can't see the table");
					}


			}		
						
			nextState = SM_FOLLOWING_PHASE;
			
		}               
        break;
		

        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}








