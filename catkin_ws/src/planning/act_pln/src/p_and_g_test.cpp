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
#define SM_Ask_Plate 130
#define SM_Take_Cutlery 140
#define SM_GRASP_OBJECT_R 150
#define SM_WAIT_OBJECT_R 160


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
	bool plate = false;


	//Reynaldo vars
	std::vector<std::string> objsToGrasp;
    std::vector<std::string> objsToTake;
    objsToGrasp.push_back("bowl");
    objsToGrasp.push_back("glass");
    objsToTake.push_back("bowl");
    objsToTake.push_back("glass");
    std::string idObject;
    bool withLeftOrRightArm;
	bool alignWithTable = false;
	int attempsGrasp = 1;
    int maxAttempsGrasp = 2;
	geometry_msgs::Pose pose;
    bool armsFree[2] = {true, true};
    std::string objsToDeliv[2] = {"", ""};
	std::stringstream ss;

  	//int nextState = SM_WaitBlindGame;
  	//int nextState = 0;
	int nextState = SM_NAVIGATE_TO_THE_ARENA;
  	
  	//set the KINECT as the input device 
  	JustinaHRI::setInputDevice(JustinaHRI::RODE);

  	JustinaHRI::loadGrammarSpeechRecognized("p_and_g.xml");//load the grammar
	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech

	vision_msgs::VisionObjectList my_cutlery;	  
	//my_cutlery.ObjectList.resize(8); 
	my_cutlery.ObjectList.resize(2); 

	my_cutlery.ObjectList[0].id="green_2";
	my_cutlery.ObjectList[1].id="red";
	
	/*my_cutlery.ObjectList[0].id="green";	
	my_cutlery.ObjectList[1].id="lemon";
	my_cutlery.ObjectList[2].id="melon";
	my_cutlery.ObjectList[3].id="blue";
	my_cutlery.ObjectList[4].id="pink_1";
	my_cutlery.ObjectList[5].id="yellow";
	my_cutlery.ObjectList[6].id="red";
	my_cutlery.ObjectList[7].id="green_2";*/

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
				
				if (!JustinaTasks::sayAndSyncNavigateToLoc("table_4", 120000)) {
					std::cout << "P & G Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("table_4", 120000)) {
						std::cout << "P & G Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("table_4", 120000)) {
							std::cout << "P & G Test...->moving to the voice command point" << std::endl;
							nextState = SM_GRASP_OBJECT_R;
						}
					} 
					else{
						std::cout << "P & G Test...->moving to the voice command point" << std::endl;
						nextState = SM_GRASP_OBJECT_R;
					}
				} 
				else {
					std::cout << "P & G Test...->moving to the voice command point" << std::endl;
					nextState = SM_GRASP_OBJECT_R;
				}
            	std::cout << "P & G Test...->moving to the voice command point" << std::endl;
				nextState = SM_GRASP_OBJECT_R;

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

			/*case SM_Grasp_Normal_Objects:
				//estados de Reynaldo
				std::cout << "P & G Test...->moving to the table" << std::endl;
				cont = 2;
				contObj = 2;
				nextState = SM_NAVIGATE_TO_THE_DISHWASHER;
				break;*/

			case SM_GRASP_OBJECT_R:
                std::cout << "State machine: SM_GRASP_OBJECT" << std::endl;
                if(objsToGrasp.size() > 0){
                    idObject = objsToGrasp[0];
                    if(!alignWithTable && !JustinaTasks::alignWithTable(0.4)){
                        std::cout << "I can´t align with table   :´(" << std::endl;
                        JustinaNavigation::moveDistAngle(-0.05, M_PI_4/4, 2000);
                        JustinaTasks::alignWithTable(0.4);
                        JustinaTasks::alignWithTable(0.4);
                    }
                    alignWithTable = true;
                    if(attempsGrasp <= maxAttempsGrasp){
                        attempsGrasp++;
                        if(JustinaTasks::findObject(idObject, pose, withLeftOrRightArm)){
                            // index 0 is right arm index 1 is left arm
                            /*if(!(withLeftOrRightArm && armsFree[1]))
                              withLeftOrRightArm = false;
                              else if(!(!withLeftOrRightArm && armsFree[0]))
                              withLeftOrRightArm = true;*/
                            if(withLeftOrRightArm){
                                if(!armsFree[1])
                                    withLeftOrRightArm = false;
                            }
                            else{
                                if(!armsFree[0])
                                    withLeftOrRightArm = true;
                            }
                            //if(JustinaTasks::moveActuatorToGrasp(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, idObject)){
                            // If we want to use another frame we need to pass de id how not empty
                            if(JustinaTasks::graspObject(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, "", true)){
                                if(withLeftOrRightArm){
                                    objsToDeliv[1] = idObject;
                                    armsFree[1] = false;
                                }else{
                                    objsToDeliv[0] = idObject;
                                    armsFree[0] = false;
                                }
                                objsToGrasp.erase(objsToGrasp.begin());
                                objsToTake.erase(objsToTake.begin());
                            }
                        }
                    }
                    else{
                        alignWithTable = false;
                        attempsGrasp = 1;
                        nextState = SM_WAIT_OBJECT_R;
                    }
                }
                else{
                    alignWithTable = false;
                    attempsGrasp = 1;
                    nextState = SM_WAIT_OBJECT_R;
                }
                break;

            case SM_WAIT_OBJECT_R:
                std::cout << "State machine: SM_WAIT_OBJECT" << std::endl;
                if(objsToTake.size() > 0){
                    idObject = objsToTake[0];
                    ss.str("");
                    ss << "I can not take the " << idObject << ", but i will take the " << idObject << " if you put it in my gripper";
                    JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
                    if(armsFree[0]){
                        JustinaManip::raGoTo("navigation", 3000);
                        JustinaTasks::detectObjectInGripper(idObject, false, 7000);
                        objsToDeliv[0] = idObject;
                        armsFree[0] = false;
                        contObj++;
                    }else if(armsFree[1]){
                        JustinaManip::laGoTo("navigation", 3000);
                        objsToDeliv[1] = idObject;
                        JustinaTasks::detectObjectInGripper(idObject, true, 7000);
                        armsFree[1] = false;
                        contObj++;
                    }
                    objsToTake.erase(objsToTake.begin());
                    std::vector<std::string>::iterator it = std::find(objsToGrasp.begin(), objsToGrasp.end(), idObject);
                    if(it != objsToGrasp.end())
                        objsToGrasp.erase(it);
                    nextState = SM_WAIT_OBJECT_R;
                    if(objsToGrasp.size() > 0)
                        nextState = SM_GRASP_OBJECT_R;
                }
                else{
                    // THIS IS FOR NAVIGATION TO THE DISH WASHER

					openDWFlag = true;
					cont = 2;
					//contObj = 2;
					nextState = SM_NAVIGATE_TO_THE_DISHWASHER;
                }
                break;

			case SM_NAVIGATE_TO_THE_TABLE:
				std::cout << "P & G Test...->moving to the table" << std::endl;
				if (!JustinaTasks::sayAndSyncNavigateToLoc("table_4", 120000)) {
					std::cout << "P & G Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("table_4", 120000)) {
						std::cout << "P & G Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("table_4", 120000)) {
							if(contObj==4)
								nextState = SM_InspectTheObjetcs;
							else
								nextState = SM_Ask_Plate;
						}
					} 
					else{
						if(contObj==4)
							nextState = SM_InspectTheObjetcs;
						else
							nextState = SM_Ask_Plate;
					}
				} 
				else {
					if(contObj==4)
						nextState = SM_InspectTheObjetcs;
					else
						nextState = SM_Ask_Plate;
				}
			break;



			case SM_Ask_Plate:
				std::cout << "P & G Test...->ask for the plate" << std::endl;
				JustinaHRI::say("Human, I need your help, I cannot take the plate by myself");
        		ros::Duration(1.5).sleep();
				JustinaHRI::say("please take the plate and put in my gripper");
                ros::Duration(1.5).sleep();
                JustinaTasks::detectObjectInGripper("plate", true, 10000);
				contObj ++;
				nextState = SM_Take_Cutlery;
				

				break;
			
			case SM_Take_Cutlery:
				std::cout << "P & G Test...->take cutlery" << std::endl;
				std::cout << "P & G Test...-> inspecting the objets on the table" << std::endl;
      			if(!JustinaTasks::alignWithTable(0.42)){
      				if(!JustinaTasks::alignWithTable(0.42)){
      					std::cout << "P & G Test...-> Can not align with table." << std::endl;
						JustinaNavigation::moveDist(0.1, 3000);    
      					nextState = SM_Take_Cutlery;
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
      						nextState = SM_Take_Cutlery;
      					}
      				}
      				else{
      					std::cout << "P & G Test...-> sorting the objects" << std::endl;
      					if(!JustinaTasks::sortObjectColor(my_cutlery))
      						if(!JustinaTasks::sortObjectColor(my_cutlery)) 
      					
						std::cout << "P & G Test...-> selecting one object" << std::endl;
						for(int i=0; i < my_cutlery.ObjectList.size(); i ++){
      						if(my_cutlery.ObjectList[i].graspable == true && my_cutlery.ObjectList[i].type_object == 0){
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

								if(!JustinaTasks::graspObjectColorFeedback(objCutlery, withLeft, objCutlery.id, true)){
      								JustinaHRI::say("Sorry, I can not grasp the object");
									ros::Duration(0.5).sleep();
			  						JustinaNavigation::moveDist(-0.35, 3000);  
									JustinaManip::startTorsoGoTo(0.1, 0, 0);
									JustinaManip::waitForTorsoGoalReached(0.5);
									JustinaHRI::say("human, please take this cutlery and put in my gripper");
                					ros::Duration(1.5).sleep();
                					JustinaTasks::detectObjectInGripper("cutlery", false, 10000);
									std::cout << "P & G Test...-> cannot take the object" << std::endl;
      								std::cout << "P & G Test...-> trying again" << std::endl;
								}
								plate = true;
                				nextState = SM_NAVIGATE_TO_THE_DISHWASHER;
                				break;
      						}
						} 
					}
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
      				JustinaHRI::say("Sorry, I can not grasp the object");
					ros::Duration(0.5).sleep();
			  		JustinaNavigation::moveDist(-0.35, 3000);  
					JustinaManip::startTorsoGoTo(0.1, 0, 0);
					JustinaManip::waitForTorsoGoalReached(0.5);
					JustinaHRI::say("human, please take this cutlery and put in my gripper");
                	ros::Duration(1.5).sleep();
                	JustinaTasks::detectObjectInGripper("cutlery", false, 10000);
					std::cout << "P & G Test...-> cannot take the object" << std::endl;
      				std::cout << "P & G Test...-> trying again" << std::endl;
					objTaken ++;
					contObj ++;
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

			  	if(openDWFlag){
					JustinaHRI::say("Human i need your hel, please, open the dishwasher");
					ros::Duration(0.5).sleep();
					JustinaHRI::say("then pull off the rack");
					ros::Duration(0.5).sleep();
					JustinaHRI::say("finally, close the dishwasher until its door touches the rack");
					ros::Duration(5.0).sleep();
					JustinaHRI::say("thank you");
					ros::Duration(0.5).sleep();
					openDWFlag = false;
				}

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

      			if(plate){
					if(!JustinaTasks::placeObjectDishWasher(0.275, 0.2, 0.075, true, true)){
      					std::cout << "P & G Test...-> cannot deliver the object" << std::endl;
						JustinaNavigation::moveDist(0.1, 3000);  
						nextState = SM_DeliverObject;
						break;
					}

					else{
						JustinaHRI::say("human, please, keep the diswasher open for me");
						ros::Duration(0.5).sleep();
						attempts = 0;
						JustinaManip::startTorsoGoTo(0.1, 0, 0);
						JustinaManip::waitForTorsoGoalReached(0.5);
						nextState = SM_NAVIGATE_TO_THE_TABLE;
						//contObj = contObj + 2;
						plate = false;
					}
				}
				//JustinaHRI::say("I am going to deliver the objects");
				else{
					if(!JustinaTasks::placeObjectDishWasher(0.275, 0.2, 0.075, false, false)){
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
