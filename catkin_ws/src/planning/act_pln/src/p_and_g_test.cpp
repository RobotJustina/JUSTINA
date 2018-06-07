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

  	//int nextState = SM_WaitBlindGame;
  	int nextState = 0;
  	
  	//set the KINECT as the input device 
  	JustinaHRI::setInputDevice(JustinaHRI::KINECT);

  	JustinaHRI::loadGrammarSpeechRecognized("p_and_g.xml");//load the grammar
	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech

  	vision_msgs::CubesSegmented my_cutlery;
  	my_cutlery.recog_cubes.resize(6);

	my_cutlery.recog_cubes[0].color="red";
	my_cutlery.recog_cubes[1].color="green";
	my_cutlery.recog_cubes[2].color="blue";
	my_cutlery.recog_cubes[3].color="purple";
	my_cutlery.recog_cubes[4].color="yellow";
	my_cutlery.recog_cubes[5].color="orange";

	bool cutlery_found = false;
	geometry_msgs::Pose pose;
	bool withLeft = false;
	std::string id_cutlery;
	int objTaken = 0;
	int chances =0;
	int maxDelayAfterSay = 300;
	int cont_z;



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
				if (!JustinaTasks::sayAndSyncNavigateToLoc("coffee_table", 120000)) {
					std::cout << "P & G Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("coffee_table", 120000)) {
						std::cout << "P & G Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("coffee_table", 120000)) {
							std::cout << "P & G Test...->moving to the voice command point" << std::endl;
							nextState = SM_WAIT_FOR_COMMAND;
						}
					} 
					else{
						std::cout << "P & G Test...->moving to the voice command point" << std::endl;
						nextState = SM_WAIT_FOR_COMMAND;
					}
				} 
				else {
					std::cout << "P & G Test...->moving to the voice command point" << std::endl;
					nextState = SM_WAIT_FOR_COMMAND;
				}
            	std::cout << "P & G Test...->moving to the voice command point" << std::endl;
				nextState = SM_WAIT_FOR_COMMAND;

			break;

			case SM_WAIT_FOR_COMMAND:
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
			break;

			case SM_NAVIGATE_TO_THE_TABLE:
				std::cout << "P & G Test...->moving to the table" << std::endl;
				if (!JustinaTasks::sayAndSyncNavigateToLoc("coffee_table", 120000)) {
					std::cout << "P & G Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("coffee_table", 120000)) {
						std::cout << "P & G Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("coffee_table", 120000)) {
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
      			std::cout << "P & G Test...-> inspecting the objets on the table" << std::endl;
      			if(!JustinaTasks::alignWithTable(0.42)){
      				if(!JustinaTasks::alignWithTable(0.42)){
      					std::cout << "P & G Test...-> Can not align with table." << std::endl;
      					nextState = SM_InspectTheObjetcs;
      				}
      			}
      			else{
      				std::cout << "P & G Test...-> trying to detect the objects" << std::endl;
      				JustinaHRI::say("I am looking for an object on the table");
        			ros::Duration(2.0).sleep();
      				if(!JustinaVision::getCutlerySeg(my_cutlery)){
      					if(!JustinaVision::getCutlerySeg(my_cutlery)){
      						std::cout << "P & G Test...-> Can not detect any object" << std::endl;
      						nextState = SM_InspectTheObjetcs;
      					}
      				}

      				else{
      					std::cout << "P & G Test...-> sorting the objects" << std::endl;

      					if(!JustinaTasks::sortCutleries(my_cutlery))
      						if(!JustinaTasks::sortCutleries(my_cutlery)) 

      					std::cout << "P & G Test...-> selecting one object" << std::endl;

      					for(int i=0; i < my_cutlery.recog_cubes.size(); i ++){
      						if(my_cutlery.recog_cubes[i].detected_cube == true){
      							std::cout << "P & G Test...-> detect the " << my_cutlery.recog_cubes[i].color << " object" << std::endl;
      							pose.position.x = my_cutlery.recog_cubes[i].cube_centroid.x;
                				pose.position.y = my_cutlery.recog_cubes[i].cube_centroid.y;
                				pose.position.z = my_cutlery.recog_cubes[i].cube_centroid.z;
                				id_cutlery = my_cutlery.recog_cubes[i].color;
                				JustinaHRI::say("I've found an object on the table");
        						ros::Duration(2.0).sleep();
                				nextState = SM_TakeObject;
                				break;
      						}
						} 
      				}
      			}

      		break;

      		case SM_TakeObject:
      			std::cout << "P & G Test...-> taking objects" << std::endl;

      			if(objTaken == 0){
      				if(pose.position.y > 0){
						withLeft = true;
						std::cout << "P & G Test...-> using  left arm" << std::endl;
						//JustinaHRI::say("I am going to take an object with my left arm");
        				//ros::Duration(2.0).sleep();
      				}
					else{
						withLeft = false;
						std::cout << "P & G Test...-> using  right arm" << std::endl;
						//JustinaHRI::say("I am going to take an object with my right arm");
        				//ros::Duration(2.0).sleep();
					}
      			}

      			else{
      				if(!withLeft){
      					//withLeft=true;
      					std::cout << "P & G Test...-> using  right arm" << std::endl;
						//JustinaHRI::say("I am going to take an object with my right arm");
        				//ros::Duration(2.0).sleep();
      				}
      				else{
      					//withLeft = false;
						std::cout << "P & G Test...-> using  left arm" << std::endl;
						//JustinaHRI::say("I am going to take an object with my left arm");
        				//ros::Duration(2.0).sleep();
      				}
      			}
      			

				if(!JustinaTasks::graspCutleryFeedback(pose.position.x, pose.position.y, pose.position.z, withLeft, id_cutlery, true)){
      				std::cout << "P & G Test...-> cannot take the object" << std::endl;
      				std::cout << "P & G Test...-> trying again" << std::endl;
				}
				else{
					if(!withLeft)
						withLeft=true;
					else
						withLeft=false;

					objTaken ++;
				}

      			if(objTaken==2){
      				nextState = SM_NAVIGATE_TO_THE_DISHWASHER;
      			}
      			else
      				nextState = SM_InspectTheObjetcs;
      		break;

      		case SM_NAVIGATE_TO_THE_DISHWASHER:
				std::cout << "P & G Test...->moving to the dish washer" << std::endl;
				if (!JustinaTasks::sayAndSyncNavigateToLoc("dishwasher", 120000)) {
					std::cout << "P & G Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("dishwasher", 120000)) {
						std::cout << "P & G Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("dishwasher", 120000)) {
							nextState = SM_DeliverObject;
						}
					} 
					else{
						nextState = SM_DeliverObject;
					}
				} 
				else {
					nextState = SM_DeliverObject;
				}
			break;

      		case SM_DeliverObject:
      			std::cout << "P & G Test...-> delivering the objects" << std::endl;

      			if(withLeft){
      				JustinaHRI::say("I am going to deliver an object with my left arm");
      				if(!JustinaTasks::placeCutleryOnDishWasher(withLeft, 0.22))
      					if(!JustinaTasks::placeCutleryOnDishWasher(withLeft, 0.22))
      						std::cout << "P & G Test...-> cannot deliver the object" << std::endl;
      				JustinaManip::laGoTo("home", 6000);
      				withLeft=false;
      				objTaken --;
      			}
      			else{
      				JustinaHRI::say("I am going to deliver an object with my right arm");
      				if(!JustinaTasks::placeCutleryOnDishWasher(withLeft, 0.22))
      					if(!JustinaTasks::placeCutleryOnDishWasher(withLeft, 0.22))
      						std::cout << "P & G Test...-> cannot deliver the object" << std::endl;
      				JustinaManip::raGoTo("home", 6000);
      				withLeft=true;
      				objTaken --;
      			}

      			chances++;

      			if(objTaken == 0 && chances==4)
      				nextState = SM_NAVIGATE_TO_THE_EXIT;

      			else if(objTaken==0 && chances ==2)
      				nextState = SM_NAVIGATE_TO_THE_TABLE;

      			else
      				nextState = SM_DeliverObject;
      			
      		break;

      		case SM_NAVIGATE_TO_THE_EXIT:
				std::cout << "P & G Test...->moving to the exit" << std::endl;
				if (!JustinaTasks::sayAndSyncNavigateToLoc("exit", 120000)) {
					std::cout << "P & G Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("exit", 120000)) {
						std::cout << "P & G Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("exit", 120000)) {
							nextState = SM_FinalState;
						}
					} 
					else{
						nextState = SM_FinalState;
					}
				} 
				else {
					nextState = SM_FinalState;
				}
			break;

			case SM_FinalState:
				std::cout <<"P & G Test...->finalState reached" << std::endl;
				JustinaHRI::say("I have finished the procter & Gamble challenge");
				ros::Duration(2.0).sleep();
				success=true;
			break;

    }
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
