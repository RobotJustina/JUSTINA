#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaKnowledge.h"
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
    JustinaNavigation::setNodeHandle(&n);
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
    
    int c_point=0,i=1;
    int nextState = 0;
    bool fail = false;
    bool success = false;
    bool stop=false;

    int d_table_1=0, d_table_2=0, d_table_3=0;

    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    
    vision_msgs::FindPlane fp;
    fp.request.name="";
    
    ros::ServiceClient client = n.serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/findPlane");
	
	float robot_y,robot_x,robot_a;    

    validCommands.push_back("robot follow me");
    validCommands.push_back("stop");
    validCommands.push_back("continue");
    validCommands.push_back("this is the table one");
    validCommands.push_back("this is the table two");
    validCommands.push_back("this is the table three");
    //validCommands.push_back("table two");
    //validCommands.push_back("table three");
    validCommands.push_back("this is the kitchen");
	validCommands.push_back("go to the table one");
	validCommands.push_back("go to the table two");
	validCommands.push_back("go to the table three");
	bool userConfirmation;



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
                
                JustinaHRI::say("You can tell me one of the next commands:"); 
                JustinaHRI::say("To save a table: this is the table one");
                JustinaHRI::say("this is the table two");
                JustinaHRI::say("or");
                JustinaHRI::say("this is the table three");             
                JustinaHRI::say("To set kitchen's location: this is the kitchen");             
		JustinaHRI::say("For sending me to a table, please say:");
		JustinaHRI::say("Go to the table one");
                JustinaHRI::say("Go to the table two");
		JustinaHRI::say("or");
		JustinaHRI::say("Go to the table three");
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
			JustinaHardware::setHeadGoalPose(0,0);
			

			while(!stop){
						if(i>3){
							nextState = SM_FOLLOWING_RETURN_KITCHEN;
							JustinaHRI::say("I saved 3 tables");
							JustinaHRI::stopFollowHuman();
							sleep(1);
							JustinaHRI::say("Profesional Waiter, lets go back to the kitchen");
							sleep(1);
							JustinaHRI::say("I will follow");
							stop=true;
						}

						else{
							if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
								/*if(lastRecoSpeech.find("stop") != std::string::npos){
									stop=true;
			                    	JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_PAUSE;
									JustinaHRI::say("I stopped");
			                    	sleep(1);
			                    	JustinaHRI::say("I'm waiting for the continue commnad");
									

								}*/
								if(lastRecoSpeech.find("this is the table one") != std::string::npos){
									stop=true;
									JustinaHRI::say("I stopped");
									JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_TABLE_1;
									d_table_1=1;
									//nextState = SM_FIRST_ORDER_TABLE_A;
								}
								else if(lastRecoSpeech.find("this is the table two") != std::string::npos){
									stop=true;
									JustinaHRI::say("I stopped");
									JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_TABLE_1;
									d_table_1=2;
								}
								else if(lastRecoSpeech.find("this is the table three") != std::string::npos){
									stop=true;
									JustinaHRI::say("I stopped");
									JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_TABLE_1;
									d_table_1=3;
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

        case SM_FOLLOWING_TABLE_1:
		{         

			JustinaHardware::setHeadGoalPose(0,0);
			std::cout << "State machine: SM_FOLLOWING_TABLE_1" << std::endl;

			if (i==1){

			JustinaHardware::setHeadGoalPose(1, -0.7);
			sleep(3);
				if(client.call(fp)){	
					JustinaManip::startLaGoTo("table");
					JustinaHRI::say("I see the table in  my left side");

					JustinaNavigation::getRobotPose(robot_x,robot_y,robot_a);

					if (d_table_1=1)
						JustinaKnowledge::addUpdateKnownLoc("table_1", robot_a+1.5708);
					else if (d_table_1=2)
						JustinaKnowledge::addUpdateKnownLoc("table_2", robot_a+1.5708);
					else if (d_table_1=3)
						JustinaKnowledge::addUpdateKnownLoc("table_3", robot_a+1.5708);


					sleep(1);
					std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_Restaurant") << std::endl;
					d_table_1=1;	
					i++;
					JustinaManip::startLaGoTo("home");
					}					
				else{
					JustinaHardware::setHeadGoalPose(-1, -0.7);
					sleep(3);
						if(client.call(fp)){	
							JustinaManip::startRaGoTo("table");
							JustinaHRI::say("I see the table in  my right side");
							JustinaNavigation::getRobotPose(robot_x,robot_y,robot_a);

					if (d_table_1=1)
						JustinaKnowledge::addUpdateKnownLoc("table_1", robot_a-1.5708);
					else if (d_table_1=2)
						JustinaKnowledge::addUpdateKnownLoc("table_2", robot_a-1.5708);
					else if (d_table_1=3)
						JustinaKnowledge::addUpdateKnownLoc("table_3", robot_a-1.5708);
		sleep(1);
							std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_Restaurant") << std::endl;
							d_table_1=2;
							i++;
							JustinaManip::startRaGoTo("home");
						}
						else
							JustinaHRI::say("I can't see the table");
					}


			}
			
			else if (i==2){

			JustinaHardware::setHeadGoalPose(1, -0.7);
			sleep(3);
				if(client.call(fp)){
				    JustinaManip::startLaGoTo("table");	
					JustinaHRI::say("I see the table in  my left side");
					JustinaNavigation::getRobotPose(robot_x,robot_y,robot_a);
					if (d_table_1=1)
						JustinaKnowledge::addUpdateKnownLoc("table_1", robot_a+1.5708);
					else if (d_table_1=2)
						JustinaKnowledge::addUpdateKnownLoc("table_2", robot_a+1.5708);
					else if (d_table_1=3)
						JustinaKnowledge::addUpdateKnownLoc("table_3", robot_a+1.5708);

					i++;
					sleep(1);
					JustinaKnowledge::addUpdateKnownLoc("i saved the table two");
					std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_Restaurant") << std::endl;
					d_table_2=1;
					JustinaManip::startLaGoTo("home");
			
					}					
				else{
					JustinaHardware::setHeadGoalPose(-1, -0.7);
					sleep(3);
						if(client.call(fp)){	
							JustinaManip::startRaGoTo("table");
							JustinaHRI::say("I see the table in  my right side");
							JustinaNavigation::getRobotPose(robot_x,robot_y,robot_a);

					if (d_table_1=1)
						JustinaKnowledge::addUpdateKnownLoc("table_1", robot_a-1.5708);
					else if (d_table_1=2)
						JustinaKnowledge::addUpdateKnownLoc("table_2", robot_a-1.5708);
					else if (d_table_1=3)
						JustinaKnowledge::addUpdateKnownLoc("table_3", robot_a-1.5708);
		sleep(1);
							JustinaKnowledge::addUpdateKnownLoc("i saved the table two");
							std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_Restaurant") << std::endl;
							d_table_2=2;
							i++;
							JustinaManip::startRaGoTo("home");
						}
						else
							JustinaHRI::say("I can't see the table");
					}


			}

			else if (i==3){

			JustinaHardware::setHeadGoalPose(1, -0.7);
			sleep(3);
				if(client.call(fp)){	
					JustinaManip::startLaGoTo("table");
					JustinaHRI::say("I see the table in  my left side");
					JustinaNavigation::getRobotPose(robot_x,robot_y,robot_a);
if (d_table_1=1)
					JustinaKnowledge::addUpdateKnownLoc("table_1", robot_a+1.5708);
					else if (d_table_1=2)
						JustinaKnowledge::addUpdateKnownLoc("table_2", robot_a+1.5708);
					else if (d_table_1=3)
						JustinaKnowledge::addUpdateKnownLoc("table_3", robot_a+1.5708);

					i++;
					sleep(1);
					JustinaKnowledge::addUpdateKnownLoc("i saved the table three");
					std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_Restaurant") << std::endl;
					d_table_3=1;
					JustinaManip::startLaGoTo("home");

					}					
				else{
					JustinaHardware::setHeadGoalPose(-1, -0.7);
					sleep(3);
						if(client.call(fp)){	
							JustinaManip::startRaGoTo("table");
							JustinaHRI::say("I see the table in  my right side");
							JustinaNavigation::getRobotPose(robot_x,robot_y,robot_a);

					if (d_table_1=1)
						JustinaKnowledge::addUpdateKnownLoc("table_1", robot_a-1.5708);
					else if (d_table_1=2)
						JustinaKnowledge::addUpdateKnownLoc("table_2", robot_a-1.5708);
					else if (d_table_1=3)
						JustinaKnowledge::addUpdateKnownLoc("table_3", robot_a-1.5708);
		sleep(1);
							JustinaKnowledge::addUpdateKnownLoc("i saved the table three");
							std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_Restaurant") << std::endl;
							d_table_3=2;
							i++;
							JustinaManip::startRaGoTo("home");
						}
						else
							JustinaHRI::say("I can't see the table");
					}


			}		
						
			nextState = SM_FOLLOWING_PHASE;
			
		}               
        break;


	case SM_FOLLOWING_RETURN_KITCHEN:
		{
            JustinaHRI::startFollowHuman();
			ros::spinOnce();
			
            std::cout << "State machine: SM_FOLLOWING_RETURN_KITCHEN" << std::endl;
            stop=false;
	    	
	        std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_Restaurant") << std::endl;
			
	   		ros::spinOnce();
			
			while(!stop){
			            	if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
								/*if(lastRecoSpeech.find("stop") != std::string::npos){
									stop=true;
			                    	JustinaHRI::stopFollowHuman();
									nextState = SM_FOLLOWING_RETURN_PAUSE;
									JustinaHRI::say("I stopped");
			                    	sleep(1);
			                    	JustinaHRI::say("I'm waiting for the continue commnad");
			                    }*/
								if(lastRecoSpeech.find("this is the kitchen") != std::string::npos){
									stop=true;
									nextState=SM_ORDERING_PHASE;
									JustinaKnowledge::addUpdateKnownLoc("kitchen");
	      							JustinaHRI::say("Profesional waiter, we return to the kitchen");
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
			sleep(1);
			JustinaHRI::say("I will start the ordering phase");
			sleep(1);
			JustinaHRI::say("Wich table should i go?, table one, table two or table three");
			nextState=SM_FIRST_ORDER_WHICH_TABLE;

		}
        break;

        case SM_FIRST_ORDER_WHICH_TABLE:
		{
    	std::cout << "State machine: SM_WHICH_TABLE" << std::endl;
		stop=false;
        while(!stop){
            if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find("go to the table one") != std::string::npos){
                            stop=true;
                            JustinaHRI::say("I will go to table a for the first order");
					        if(!JustinaNavigation::getClose("table_1",200000))
					        	if(!JustinaNavigation::getClose("table_1",200000))
					        		JustinaNavigation::getClose("table_1",200000);
					        /*if(d_table_1==1){
					        	JustinaNavigation::moveDistAngle(0,1.5708, 5000);

					        }
					        else if(d_table_1==2){
					        	JustinaNavigation::moveDistAngle(0,-1.5708,5000);

					        }*/	
							JustinaHRI::say("I arrived to  table a");
							nextState=SM_FIRST_ORDER_TABLE_A;
                    }

                    else if(lastRecoSpeech.find("go to the table two") != std::string::npos){
                            stop=true;
                            JustinaHRI::say("I will go to table b for the first order");
					        if(!JustinaNavigation::getClose("table_2",200000))
					        	if(!JustinaNavigation::getClose("table_2",200000))
					        		JustinaNavigation::getClose("table_2",200000);
					       /* if(d_table_2==1){
					        	JustinaNavigation::moveDistAngle(0,1.5708, 5000);

					        }
					        else if(d_table_2==2){
					        	JustinaNavigation::moveDistAngle(0,-1.5708, 5000);

					        }*/	
							JustinaHRI::say("I arrived to  table b");
							nextState=SM_FIRST_ORDER_TABLE_A;
                    }

                    else if(lastRecoSpeech.find("go to the table three") != std::string::npos){
                            stop=true;
                            JustinaHRI::say("I will go to table c for the first order");
					        if(!JustinaNavigation::getClose("table_3",200000))
					        	if(!JustinaNavigation::getClose("table_3",200000))
					        		JustinaNavigation::getClose("table_3",200000);
					        /*if(d_table_3==1){
					        	JustinaNavigation::moveDistAngle(0,1.5708, 5000);

					        }
					        else if(d_table_3==2){
					        	JustinaNavigation::moveDistAngle(0,-1.5708, 5000);

					        }*/	
							JustinaHRI::say("I arrived to  table c");
							nextState=SM_FIRST_ORDER_TABLE_A;						

                    }
                    
                    else{
                            std::cout << "Command ERROR!" << std::endl;
                            JustinaHRI::say("Please repeat the table");
                        	}
    					}
					}
    }
    break;

    	case SM_FIRST_ORDER_TABLE_A:
    {
    		JustinaHRI::say("Good day human, Please tell me what do you want");
  			while (!JustinaHRI::waitForSpeechRecognized(reco_sentence,10000) && ros::ok());
		if(reco_sentence.find("I want") != std::string::npos)
		  {
		    JustinaHRI::say("Did you say?: " + reco_sentence);
		    JustinaHRI::say("Please answer, robot yes or robot no");
		    JustinaHRI::waitForUserConfirmation(userConfirmation, 20000);
		    if(userConfirmation)
		    {		    	
		      JustinaHRI::say("O.K. I will bring your order");
		      JustinaHRI::say("Do you want anything else?");
		      JustinaHRI::say("Please answer robot yes or robot no");
		      JustinaHRI::waitForUserConfirmation(userConfirmation, 20000);
		      if(userConfirmation)
			{
			  JustinaHRI::say("I will take another order");
			}
		      else
			{
			  JustinaHRI::say("Ok. I will go to the kitchen to serve your order");
			  JustinaNavigation::getClose("kitchen", 180000);
			  nextState = -1;
			}
		    }
		    else
		      {
			JustinaHRI::say("O.k");
		      }
		}
		else
		  {
		    JustinaHRI::say("Please repeat your order");
		  }
    		//Guardar la orden MESA A
    		//Reconocer la orden
    		//if(Topico de Isra)
    		// Voltear para decir que serán atendidos
    		// despues de tomar la primer orden ir a la table 2
    		//nextState=SM_FIRST_ORDER_TABLE_B;
    		//else
    		//nextState=SM_FOLLOWING_RETURN_KITCHEN;
    		
    }
    break;

        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}








