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

/*
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
*/
const int SM_INIT = 0;
const int SM_WAIT_FOR_INIT_COMMAND= 10          ;
const int SM_TRAINING_PHASE =20                 ;
const int SM_FOLLOWING_PHASE= 30                ;
const int SM_FOLLOWING_PAUSE =40                ;
const int SM_FOLLOWING_TABLE_1= 50              ;
const int SM_FOLLOWING_TABLE_2= 60              ;
const int SM_FOLLOWING_TABLE_3= 70              ;
const int SM_FOLLOWING_RETURN_KITCHEN =80       ;
const int SM_FOLLOWING_RETURN_PAUSE =90         ;
const int SM_ORDERING_PHASE =100                ;
const int SM_FIRST_ORDER_WHICH_TABLE =110       ;
const int SM_FIRST_ORDER_TABLE_A =120           ;
const int SM_FIRST_ORDER_TABLE_B =130           ;
const int SM_DELIVERING_PHASE =140              ;
const int SM_DELIVERING_TAKING_ORDER =150       ;
const int SM_DELIVERING_BEVERAGE =160           ;
const int SM_DELIVERING_RETURN_KITCHEN =170     ;
const int SM_FIRST_ORDER_RETURN_KITCHEN= 180    ;
const int SM_DELIVERING_PUT_ORDER =190          ;
const int SM_FINAL_STATE =200                   ;
const int SM_WAIT_FOR_LEGS_FOUND =210           ;


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
    vision_msgs::FindPlane fp;
    fp.request.name="";
    ros::Rate loop(10);
    ros::ServiceClient client = n.serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/findPlane");
    

    int c_point=0,i=0;
    int nextState = 0;
    bool fail = false;
    bool success = false;
    bool stop=false;
    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    validCommands.push_back("robot follow me");
    validCommands.push_back("stop");
    validCommands.push_back("continue");
    validCommands.push_back("table 1");
    validCommands.push_back("table 2");
    validCommands.push_back("table 3");
    validCommands.push_back("kitchen");


    //ros::Publisher pubFollow = n.advertise<std_msgs::Bool>("/hri/human_following/start_follow",1); 
	//std_msgs::Bool startFollow;
    

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
			JustinaNavigation::addLocation("kitchen", -0.5, 0);
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
                JustinaHRI::say("I found you");
                sleep(1);
                JustinaHRI::say("You can tell me one of the next commands: continue, stop, table 1, table 2, table 3");
                sleep(1);	                
                JustinaHRI::say("I will start to follow you Professional Waiter, please walk");
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
								else if(lastRecoSpeech.find("table 1") != std::string::npos){
									stop=true;
									JustinaHRI::say("I stopped");
									JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_TABLE_1;
								}
								else if(lastRecoSpeech.find("table 2") != std::string::npos){
									stop=true;
									JustinaHRI::say("I stopped");
									JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_TABLE_2;
								}
								else if(lastRecoSpeech.find("table 3") != std::string::npos){
									stop=true;
									JustinaHRI::say("I stopped");
									JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_TABLE_3;
								}
			                   
								else{
									std::cout << "Command ERROR!" << std::endl;
									JustinaHRI::say("Please repeat the command");
									}
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

	 case SM_FOLLOWING_TABLE_1:
		{         
			std::cout << "State machine: SM_FOLLOWING_TABLE_1" << std::endl;
			JustinaHardware::setHeadGoalPose(1, 1);
			sleep(2);
				if(client.call(fp)){	
					JustinaHRI::say("I see the table in  my left side");
					JustinaNavigation::addLocation("table_1");
					i++;
					}					
				else{
					JustinaHardware::setHeadGoalPose(-1, 1);
					sleep(2);
						if(client.call(fp)){	
							JustinaHRI::say("I see the table in  my right side");
							JustinaNavigation::addLocation("table_1");
							i++;
						}
						else
							JustinaHRI::say("I can't see the table");
				}
						
			nextState = SM_TRAINING_PHASE;
			
		}               
        break;
		
	case SM_FOLLOWING_TABLE_2:
		{         
			std::cout << "State machine: SM_FOLLOWING_TABLE_2" << std::endl;
			JustinaHardware::setHeadGoalPose(1, 1);
			sleep(2);
				if(client.call(fp)){	
					JustinaHRI::say("I see the table in  my left side");
					JustinaNavigation::addLocation("table_2");
					i++;
					}					
				else{
					JustinaHardware::setHeadGoalPose(-1, 1);
					sleep(2);
						if(client.call(fp)){	
							JustinaHRI::say("I see the table in  my right side");
							JustinaNavigation::addLocation("table_2");
							i++;
						}
						else
							JustinaHRI::say("I can't see the table");
				}
						
			nextState = SM_TRAINING_PHASE;
			
		}               
    break;

    case SM_FOLLOWING_TABLE_3:
		{         
			std::cout << "State machine: SM_FOLLOWING_TABLE_3" << std::endl;
			JustinaHardware::setHeadGoalPose(1, 1);
			sleep(2);
				if(client.call(fp)){	
					JustinaHRI::say("I see the table in  my left side");
					JustinaNavigation::addLocation("table_3");
					i++;
					}					
				else{
					JustinaHardware::setHeadGoalPose(-1, 1);
					sleep(2);
						if(client.call(fp)){	
							JustinaHRI::say("I see the table in  my right side");
							JustinaNavigation::addLocation("table_3");
							i++;
						}
						else
							JustinaHRI::say("I can't see the table");
				}
						
			nextState = SM_TRAINING_PHASE;
			
		}               
    break;    



	case SM_FOLLOWING_RETURN_KITCHEN:
		{
            std::cout << "State machine: SM_FOLLOWING_RETURN_KITCHEN" << std::endl;
            JustinaHRI::say("Human, please put in front of me");
            stop=false;
	    	while (!stop){
	    		std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
            		if(JustinaHRI::frontalLegsFound())
	            	{
	                std::cout << "NavigTest.->Frontal legs found!" << std::endl;
	                JustinaHRI::say("I will start to follow you human, please walk");
	        		stop=true;
	        		}	

	    	}

            JustinaHRI::startFollowHuman();
            std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_Restaurant") << std::endl;
			stop=false;
	   		ros::spinOnce();
			
			while(!stop){
						//if llegó a kitchen
							//nextState = SM_ORDERING_PHASE;
						//else{
		                	if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
								if(lastRecoSpeech.find("stop") != std::string::npos){
									stop=true;
			                    	JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_RETURN_PAUSE;
									JustinaHRI::say("I stopped");
			                    	sleep(1);
			                    	JustinaHRI::say("I'm waiting for the continue commnad");
			                    }
								else if(lastRecoSpeech.find("kitchen") != std::string::npos){
									stop=true;
									nextState=SM_ORDERING_PHASE;
								}								
			                   
								else{
									std::cout << "Command ERROR!" << std::endl;
									JustinaHRI::say("Please repeat the command");
								}
								
							//}
							
						}			

		}
	}
        break;

    case SM_FOLLOWING_RETURN_PAUSE:
    {
    	std::cout << "State machine: SM_FOLLOWING_RETURN_PAUSE" << std::endl;
		stop=false;
        while(!stop){
            if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find("continue") != std::string::npos){
                            std::cout << "Command CONTINUE!" << std::endl;
                            stop=true;
                            nextState = SM_FOLLOWING_RETURN_KITCHEN;
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
	
	case SM_ORDERING_PHASE:
		{
			std::cout << "State machine: SM_ORDERING_PHASE" << std::endl;
			JustinaHRI::stopFollowHuman();
			JustinaHRI::say("We back to the kitchen");
			sleep(1);
			JustinaHRI::say("I will start the ordering phase");
			sleep(1);
			JustinaHRI::say("Wich table should i go?");
			nextState=SM_FIRST_ORDER_WHICH_TABLE;

		}
        break;

    case SM_FIRST_ORDER_WHICH_TABLE:
		{
    	std::cout << "State machine: SM_WHICH_TABLE" << std::endl;
		stop=false;
        while(!stop){
            if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find("table one") != std::string::npos){
                            stop=true;
                            JustinaHRI::say("I will go to table a for the first order");
					        if(!JustinaNavigation::getClose("table_1",200000))
					        	if(!JustinaNavigation::getClose("table_1",200000))
					        		JustinaNavigation::getClose("table_1",200000);
							JustinaHRI::say("I arrived to  table 1");
							nextState=SM_FIRST_ORDER_TABLE_A;
                    }

                    else if(lastRecoSpeech.find("table two") != std::string::npos){
                            stop=true;
                            JustinaHRI::say("I will go to table 2");
					        if(!JustinaNavigation::getClose("table_2",200000))
					        	if(!JustinaNavigation::getClose("table_2",200000))
					        		JustinaNavigation::getClose("table_2",200000);
							JustinaHRI::say("I arrived to  table 2");
							nextState=SM_FIRST_ORDER_TABLE_A;
                    }

                    else if(lastRecoSpeech.find("table three") != std::string::npos){
                            stop=true;
                            JustinaHRI::say("I will go to table 3");
					        if(!JustinaNavigation::getClose("table_3",200000))
					        	if(!JustinaNavigation::getClose("table_3",200000))
					        		JustinaNavigation::getClose("table_3",200000);
							JustinaHRI::say("I arrived to  table 3");
							nextState=SM_FIRST_ORDER_TABLE_A;						

                    }
                    
                    else{
                            std::cout << "Command ERROR!" << std::endl;
                            JustinaHRI::say("Please repeat the command");
                        	}
    					}
					}
    }
    break;

    case SM_FIRST_ORDER_TABLE_A:
    {
    		JustinaHRI::say("Good day human, i will take your order");
    		JustinaHRI::say("Your order");
    		//Guardar la orden MESA A
    		//Reconocer la orden
    		//if(Topico de Isra)
    		// Voltear para decir que serán atendidos
    		// despues de tomar la primer orden ir a la table 2
    		//nextState=SM_FIRST_ORDER_TABLE_B;
    		//else
    		nextState=SM_FOLLOWING_RETURN_KITCHEN;
    		
    }
    break;

    case SM_FIRST_ORDER_TABLE_B:
    {
    		JustinaHRI::say("Good day human, i will take your order");
    		JustinaHRI::say("Your order");
    		//guardar la orden MESA B
    		nextState=SM_FIRST_ORDER_RETURN_KITCHEN;
    		
    }
    break;

    case SM_FIRST_ORDER_RETURN_KITCHEN:
    {
    		JustinaHRI::say("I will go to the kitchen for your order");
    		if(!JustinaNavigation::getClose("kitchen",200000))
				if(!JustinaNavigation::getClose("kitchen",200000))
		     		JustinaNavigation::getClose("kitchen",200000);
		    JustinaHRI::say("I arrived to the kitchen");
		    nextState = SM_DELIVERING_PHASE;
    }
    break;

    case SM_DELIVERING_PHASE:
    {
    	JustinaHRI::say("Order table A:");
    	//if (isra topic)
    	//	JustinaHRI::say("Order table B:");
    	nextState= SM_DELIVERING_TAKING_ORDER;
    }

    case SM_DELIVERING_TAKING_ORDER:
    {
    	JustinaHRI::say("I will wait for the order");
    	//if (order)
    		nextState=SM_DELIVERING_BEVERAGE;
    	//	JustinaHRI::say("Order table B:");
    }		

    case SM_DELIVERING_BEVERAGE:
    {
    	JustinaHRI::say("I will navigate to the table --");
    		if(!JustinaNavigation::getClose("table--",200000))
				if(!JustinaNavigation::getClose("table--",200000))
		     		JustinaNavigation::getClose("table--",200000);
		    JustinaHRI::say("I arrived to the table--");
		    nextState=SM_DELIVERING_PUT_ORDER;
    }

    case SM_DELIVERING_PUT_ORDER:
    {
    	JustinaHRI::say("I will put the order in the table--");
    		//poner la orden en el table
    	nextState=SM_DELIVERING_RETURN_KITCHEN;
    }

    case SM_DELIVERING_RETURN_KITCHEN:
    {
		JustinaHRI::say("I will navigate to the kitchen");
    		if(!JustinaNavigation::getClose("kitchen",200000))
				if(!JustinaNavigation::getClose("kitchen",200000))
		     		JustinaNavigation::getClose("kitchen",200000);
		    JustinaHRI::say("I arrived to the kitchen");
		    sleep(1);
		    JustinaHRI::say("Finish Restaurant test ");
		    nextState=SM_FINAL_STATE;   	

    }

    case SM_FINAL_STATE:
        {
        	std::cout << "State machine: SM_FINAL_STATE" << std::endl;
        }    
        break;
        
        ros::spinOnce();
        loop.sleep();
    }
}
    return 0;
}




