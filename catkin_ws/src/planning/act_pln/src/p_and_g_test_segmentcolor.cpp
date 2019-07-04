#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaAudio.h"
#include "justina_tools/JustinaRepresentation.h"
#include "justina_tools/JustinaTasks.h"
#include "std_msgs/Bool.h"
#include "string"

#define SM_InitialState 0
#define	SM_InspectTheObjetcs 10
#define SM_TakeObject 20
#define SM_DeliverObject 30
#define SM_NAVIGATE_TO_THE_ARENA 40
#define SM_NAVIGATE_TO_THE_TABLE 50
#define SM_NAVIGATE_TO_THE_DISHWASHER 60
#define SM_NAVIGATE_TO_THE_EXIT 70
#define	SM_FinalState 80
#define SM_SAY_WAIT_FOR_DOOR 90
#define SM_WAIT_FOR_DOOR 100
#define SM_WAIT_FOR_COMMAND 110
#define SM_Wait_Door_Opened 120

int main(int argc, char** argv)
{
	std::cout << "Initializing P & G Test..." << std::endl;
  	ros::init(argc, argv, "act_pln");
  	ros::NodeHandle n;
  	JustinaHardware::setNodeHandle(&n);
  	JustinaHRI::setNodeHandle(&n);
  	JustinaManip::setNodeHandle(&n);
  	JustinaNavigation::setNodeHandle(&n);
  	JustinaTools::setNodeHandle(&n);
  	JustinaVision::setNodeHandle(&n);
	JustinaAudio::setNodeHandle(&n);
	JustinaRepresentation::setNodeHandle(&n);
	JustinaTasks::setNodeHandle(&n);
	JustinaKnowledge::setNodeHandle(&n);//knowledge
	
	bool fail = false;
	bool success = false;
	bool door_open = false;

  	//int nextState = SM_WaitBlindGame;
  	int nextState = 0;
  	
  	//set the KINECT as the input device 
  	JustinaHRI::setInputDevice(JustinaHRI::RODE);

  	JustinaHRI::loadGrammarSpeechRecognized("p_and_g.xml");//load the grammar
	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech

	vision_msgs::VisionObjectList my_cutlery;	  
	my_cutlery.ObjectList.resize(8);  

	
	my_cutlery.ObjectList[0].id="green";	
	my_cutlery.ObjectList[1].id="lemon";
	my_cutlery.ObjectList[2].id="melon";
	my_cutlery.ObjectList[3].id="blue";
	my_cutlery.ObjectList[4].id="pink_1";
	my_cutlery.ObjectList[5].id="yellow";
	my_cutlery.ObjectList[6].id="red";
	my_cutlery.ObjectList[7].id="green_2";

	bool cutlery_found = false;
    /////
	//geometry_msgs::Pose pose;
    vision_msgs::VisionObject objCutlery;
	bool withLeft = false;
    /////
	//std::string id_cutlery;
	int objTaken = 0;
	int chances =0;
	int attempts = 0;
	int maxDelayAfterSay = 300;
	int cont_z;
	int cont =0;
    /////
	//int type;
	bool openDWFlag=false; //set this flag as false due to the dishwasher will be open by default
	bool takeCascadePod=false;
	int contObj =0;



  	while(ros::ok() && !fail && !success)
  	{
		ros::Rate loop(10);
  		switch(nextState)
    	{

    		case SM_InitialState:
      			std::cout << "P & G Test...-> start the P & G test" << std::endl;
        		JustinaManip::startHdGoTo(0.0, 0.0);
        		JustinaHRI::say("I am ready for the Procter & Gamble challenge");
        		ros::Duration(2.0).sleep();
           		nextState = SM_SAY_WAIT_FOR_DOOR;
      		break;

      		case SM_SAY_WAIT_FOR_DOOR:
				JustinaHRI::waitAfterSay("I am waiting for the door to be open", 4000);
				nextState = SM_WAIT_FOR_DOOR;
			break;

			case SM_WAIT_FOR_DOOR:
				if (!JustinaNavigation::obstacleInFront())
					nextState = SM_NAVIGATE_TO_THE_ARENA;
			break;

			case SM_NAVIGATE_TO_THE_ARENA:
				JustinaHRI::waitAfterSay("Now I can see that the door is open",4000);
				std::cout << "P & G Test...->First attempt to move" << std::endl;
            	JustinaNavigation::moveDist(1.0, 4000);

				JustinaHRI::waitAfterSay("Human, please remove all the chairs from the kitchen table",6000);
				
				if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen_table", 120000)) {
					std::cout << "P & G Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen_table", 120000)) {
						std::cout << "P & G Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("kitchen_table", 120000)) {
							std::cout << "P & G Test...->moving to the voice command point" << std::endl;
							nextState = SM_InspectTheObjetcs;
						}
					} 
					else{
						std::cout << "P & G Test...->moving to the voice command point" << std::endl;
						nextState = SM_InspectTheObjetcs;
					}
				} 
				else {
					std::cout << "P & G Test...->moving to the voice command point" << std::endl;
					nextState = SM_InspectTheObjetcs;
				}
            	std::cout << "P & G Test...->moving to the voice command point" << std::endl;
				nextState = SM_InspectTheObjetcs;

			break;

			//if needed a voice comand discomment next states
			
			/*case SM_WAIT_FOR_COMMAND:
				JustinaHRI::waitAfterSay("Tell me, justina start, in order to perform the task", 12000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                cont_z=0;
                std::cout << "State machine: SM_WAIT_FOR_COMMAND" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("justina start", 15000)){
                    nextState = SM_NAVIGATE_TO_THE_TABLE;
                }
                else                    
                    cont_z++;    		

                if(cont_z>3){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("Please repeat the command", 5000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    cont_z=0;
                }
			break;*/

			case SM_NAVIGATE_TO_THE_TABLE:
				std::cout << "P & G Test...->moving to the table" << std::endl;
				if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen_table", 120000)) {
					std::cout << "P & G Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen_table", 120000)) {
						std::cout << "P & G Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("kitchen_table", 120000)) {
							nextState = SM_InspectTheObjetcs;
						}
					} 
					else{
						nextState = SM_InspectTheObjetcs;
					}
				} 
				else {
					nextState = SM_InspectTheObjetcs;
				}
			break;


      		case SM_InspectTheObjetcs:

			  	JustinaManip::startTorsoGoTo(0.1, 0, 0);
				JustinaManip::waitForTorsoGoalReached(0.5);

				if((objTaken == 1 && attempts >3) || (objTaken==3 && attempts >3)){
					JustinaHRI::say("Sorry I could not grasp another object now");
        			ros::Duration(2.0).sleep();
					
					nextState = SM_NAVIGATE_TO_THE_DISHWASHER;
					cont ++;
					break;
				}
				else{
      				std::cout << "P & G Test...-> inspecting the objets on the table" << std::endl;
      				if(!JustinaTasks::alignWithTable(0.42)){
      					if(!JustinaTasks::alignWithTable(0.42)){
      						std::cout << "P & G Test...-> Can not align with table." << std::endl;
							JustinaNavigation::moveDist(0.1, 3000);    
      						nextState = SM_InspectTheObjetcs;
      					}
      				}
      				else{
      					std::cout << "P & G Test...-> trying to detect the objects" << std::endl;
      					JustinaHRI::say("I am looking for objects on the table");
        				ros::Duration(2.0).sleep();
      					if(!JustinaVision::getObjectSeg(my_cutlery)){
      						if(!JustinaVision::getObjectSeg(my_cutlery)){
								JustinaHRI::say("I can not see any object on the table");
        						ros::Duration(2.0).sleep();
      							std::cout << "P & G Test...-> Can not detect any object" << std::endl;
      							nextState = SM_InspectTheObjetcs;
      						}
      					}

      					else{
      						std::cout << "P & G Test...-> sorting the objects" << std::endl;

      						if(!JustinaTasks::sortObjectColor(my_cutlery))
      							if(!JustinaTasks::sortObjectColor(my_cutlery)) 

      						std::cout << "P & G Test...-> selecting one object" << std::endl;

      						for(int i=0; i < my_cutlery.ObjectList.size(); i ++){
      							if(my_cutlery.ObjectList[i].graspable == true){
      								std::cout << "P & G Test...-> detect the " << my_cutlery.ObjectList[i].id << " object" << std::endl;
                                    objCutlery = my_cutlery.ObjectList[i];
                                    /////
      								/*pose.position.x = my_cutlery.ObjectList[i].pose.position.x;
                					pose.position.y = my_cutlery.ObjectList[i].pose.position.y;
                					pose.position.z = my_cutlery.ObjectList[i].pose.position.z;
                					id_cutlery = my_cutlery.ObjectList[i].id;
                					type = my_cutlery.ObjectList[i].type_object;*/
                					JustinaHRI::say("I have found an object on the table");
        							ros::Duration(2.0).sleep();
                					nextState = SM_TakeObject;
                					break;
      							}
							} 
      					}
      				}
				}

      		break;

      		case SM_TakeObject:
      			std::cout << "P & G Test...-> taking objects" << std::endl;

      			if(objTaken == 0){
                    /////
      				//if(pose.position.y > 0){
      				if(objCutlery.pose.position.y > 0){
						withLeft = true;
						std::cout << "P & G Test...-> using  left arm" << std::endl;
      				}
					else{
						withLeft = false;
						std::cout << "P & G Test...-> using  right arm" << std::endl;
					}
      			}

      			else{
      				if(!withLeft){
      					std::cout << "P & G Test...-> using  right arm" << std::endl;
      				}
      				else{
						std::cout << "P & G Test...-> using  left arm" << std::endl;
      				}
      			}
				
      			

				//if(!JustinaTasks::graspObjectColorFeedback(pose.position.x, pose.position.y, pose.position.z, withLeft, id_cutlery, true)){
				if(!JustinaTasks::graspObjectColorFeedback(objCutlery, withLeft, objCutlery.id, true)){
      				JustinaHRI::say("I can not grasp the object");
        			ros::Duration(2.0).sleep();
					std::cout << "P & G Test...-> cannot take the object" << std::endl;
      				std::cout << "P & G Test...-> trying again" << std::endl;
				}
				else{
					
					if(!withLeft && contObj != 4)
						withLeft=true;
					else
						withLeft=false;

					objTaken ++;
					contObj ++;
				}

				if(contObj == 5){
					JustinaManip::startTorsoGoTo(0.1, 0, 0);
					JustinaManip::waitForTorsoGoalReached(0.5);
					cont ++;
					nextState = SM_NAVIGATE_TO_THE_DISHWASHER;
				}
				else{
					if(objTaken==2){
						cont ++;
      					nextState = SM_NAVIGATE_TO_THE_DISHWASHER;
						JustinaManip::startTorsoGoTo(0.1, 0, 0);
						JustinaManip::waitForTorsoGoalReached(0.5);
					}
      				else
      					nextState = SM_InspectTheObjetcs;
				}

				attempts ++;

      		break;

      		case SM_NAVIGATE_TO_THE_DISHWASHER:
			  	JustinaNavigation::moveDist(-0.35, 3000);
			  	JustinaManip::startTorsoGoTo(0.1, 0, 0);
				JustinaManip::waitForTorsoGoalReached(0.5);

				std::cout << "P & G Test...->moving to the dish washer" << std::endl;
				if (!JustinaTasks::sayAndSyncNavigateToLoc("dishwasher", 120000)) {
					std::cout << "P & G Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("dishwasher", 120000)) {
						std::cout << "P & G Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("dishwasher", 120000)) {
							nextState = SM_DeliverObject;
							JustinaHRI::say("I am going to deliver the objects");
						}
					} 
					else{
						nextState = SM_DeliverObject;
						JustinaHRI::say("I am going to deliver the objects");
					}
				} 
				else {
					nextState = SM_DeliverObject;
					JustinaHRI::say("I am going to deliver the objects");
				}
			break;

      		case SM_DeliverObject:
      			std::cout << "P & G Test...-> delivering the objects" << std::endl;
				if(openDWFlag){
					JustinaHRI::say("Human, please, open the dishwasher just until the half");
					ros::Duration(0.5).sleep();
					JustinaHRI::say("for example just open it 45 degrees");
					ros::Duration(0.5).sleep();
					JustinaHRI::say("Human, please, pull off the rack");
					ros::Duration(5.0).sleep();
					JustinaHRI::say("thank you");
					ros::Duration(0.5).sleep();
				}

      			
				//JustinaHRI::say("I am going to deliver the objects");
				if(!JustinaTasks::placeObjectDishWasher(0.7, 0.2, 0.0)){
      				std::cout << "P & G Test...-> cannot deliver the object" << std::endl;
					JustinaNavigation::moveDist(0.1, 3000);  
					nextState = SM_DeliverObject;
					break;
				}
				

      			if(contObj == 5){
      				nextState = SM_NAVIGATE_TO_THE_EXIT;
					JustinaHRI::say("human close the diswasher, please");
					ros::Duration(0.5).sleep();
					JustinaManip::startTorsoGoTo(0.1, 0, 0);
					JustinaManip::waitForTorsoGoalReached(0.5);
				}
				else{
					objTaken = 0;
					nextState = SM_NAVIGATE_TO_THE_TABLE;
					JustinaHRI::say("human, please, keep the diswasher open for me");
					ros::Duration(0.5).sleep();
					attempts = 0;
					JustinaManip::startTorsoGoTo(0.1, 0, 0);
					JustinaManip::waitForTorsoGoalReached(0.5);
				}

      			
      		break;

      		case SM_NAVIGATE_TO_THE_EXIT:
				std::cout << "P & G Test...->moving to the exit" << std::endl;
				if (!JustinaTasks::sayAndSyncNavigateToLoc("exit", 120000)) {
					std::cout << "P & G Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("exit", 120000)) {
						std::cout << "P & G Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("exit", 120000)) {
							nextState = SM_Wait_Door_Opened;
						}
					} 
					else{
						nextState = SM_Wait_Door_Opened;
					}
				} 
				else {
					nextState = SM_Wait_Door_Opened;
				}
			break;

			case SM_Wait_Door_Opened:
                std::cout << "Farewell Test...-> SM_Wait_Door_Opened" << std::endl;
                JustinaHRI::waitAfterSay("I am waiting for the door to be open", 4000);

                 door_open = JustinaNavigation::doorIsOpen(0.9, 2000);

                 if(door_open){
                    JustinaManip::hdGoTo(0.0, 0.0, 2000);
                    if (!JustinaTasks::sayAndSyncNavigateToLoc("exitdoor", 120000)) {
				    	std::cout << "Farewell Test...->Second attempt to move" << std::endl;
				    	if (!JustinaTasks::sayAndSyncNavigateToLoc("exitdoor", 120000)) {
				    		std::cout << "Farewell Test...->Third attempt to move" << std::endl;
				    		if (JustinaTasks::sayAndSyncNavigateToLoc("exitdoor", 120000)) {
				    			std::cout << "Farewell...->moving to the initial point" << std::endl;
				    		}
				    	} 
				    }
                    nextState = SM_FinalState;
                 }

                 else
                    nextState = SM_Wait_Door_Opened;
                break;

			case SM_FinalState:
				std::cout <<"P & G Test...->finalState reached" << std::endl;
				JustinaHRI::say("I have finished the procter & Gamble challenge");
				std::cout <<"total objetos: " << contObj << std::endl; 
				ros::Duration(2.0).sleep();
				success=true;
			break;

    }
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
