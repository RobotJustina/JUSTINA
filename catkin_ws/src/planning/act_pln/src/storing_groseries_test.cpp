#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaKnowledge.h"
#include "std_msgs/Bool.h"
#include "string"

#define SM_INIT 0
#define SM_WAIT_FOR_START_COMMAND 10
#define SM_NAVIGATION_TO_TABLE 20
#define SM_FIND_OBJECTS_ON_TABLE 30
#define SM_SAVE_OBJECTS_PDF 40
#define SM_TAKE_OBJECT_RIGHT 50
#define SM_TAKE_OBJECT_LEFT 60
#define SM_GOTO_CUPBOARD 70
#define SM_FIND_OBJECTS_ON_CUPBOARD 80
#define SM_PUT_OBJECT_ON_TABLE_RIGHT 90
#define SM_PUT_OBJECT_ON_TABLE_LEFT 100
#define SM_FINISH_TEST 110




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
	JustinaTasks::setNodeHandle(&n);
	ros::Rate loop(10);


	std::string reco_sentence;

	std::vector<vision_msgs::VisionObject> recoObjList;

	std::vector<std::string> validItems;
	validItems.push_back("juice");
	validItems.push_back("milk");
	validItems.push_back("soup");
	validItems.push_back("sugar");

	int nextState = 0;
	bool fail = false;
	bool success = false;
	bool stop=false;

	bool leftArm;



	std::string lastRecoSpeech;
	std::string idObject_1 = "";
	std::string idObject_2 = "";

	geometry_msgs::Pose poseObj_1;
	geometry_msgs::Pose poseObj_2;

	std::vector<std::string> validCommands;
	validCommands.push_back("robot start");

	bool userConfirmation;


	while(ros::ok() && !fail && !success)
	{
		switch(nextState)
		{

			case SM_INIT:
			{
				std::cout << "----->  State machine: INIT" << std::endl;
				JustinaHRI::say("I'm waiting for the start command");
				//nextState = SM_WAIT_FOR_START_COMMAND;
				//nextState = SM_PUT_OBJECT_ON_TABLE_RIGHT;
				//nextState = SM_NAVIGATION_TO_TABLE;
				nextState = SM_FIND_OBJECTS_ON_TABLE;
			}
			break;

			case SM_WAIT_FOR_START_COMMAND:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: WAIT_FOR_START_COMMAND" << std::endl;
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
                	JustinaHRI::say("Please repeat the command");
            	else
            	{
                	if(lastRecoSpeech.find("robot start") != std::string::npos)
                		nextState = SM_NAVIGATION_TO_TABLE;
                	else
                		nextState = SM_WAIT_FOR_START_COMMAND;
                }
			}
			break;

			case SM_NAVIGATION_TO_TABLE:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: NAVIGATION_TO_TABLE" << std::endl;       
				if(!JustinaNavigation::getClose("kitchen_table",200000))
			    	if(!JustinaNavigation::getClose("kitchen_table",200000))
			    		JustinaNavigation::getClose("kitchen_table",200000);
				JustinaHRI::say("I arrived to kitchen table");
				nextState = SM_FIND_OBJECTS_ON_TABLE;
			}
			break;


			case SM_FIND_OBJECTS_ON_TABLE:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FIND_OBJECTS_ON_TABLE" << std::endl;
				JustinaHRI::say("I am going to search objects on the table");

				JustinaTasks::alignWithTable(0.35);
				if(!JustinaVision::detectAllObjects(recoObjList))
					std::cout << "I  can't detect anything" << std::endl;
				else
				{
					std::cout << "I have found " << recoObjList.size() << " objects on the table" << std::endl;
					for(int i = 0; i < recoObjList.size(); i++)
					{
						std::cout << recoObjList[i].id << std::endl;
						if(recoObjList[0].id != "unknown0" && recoObjList[0].id != "unknown1" )
							idObject_1 = recoObjList[0].id;
						if(recoObjList[1].id != "unknown0" && recoObjList[1].id != "unknown1")
							idObject_2 = recoObjList[1].id;
					}
					
				}
				nextState = SM_SAVE_OBJECTS_PDF;
			}
			break;

			case SM_SAVE_OBJECTS_PDF:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: SAVE_OBJECTS_PDF" << std::endl;
				nextState = SM_TAKE_OBJECT_RIGHT;
			}
			break;

			case SM_TAKE_OBJECT_RIGHT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: TAKE_OBJECT_RIGHT" << std::endl;
				if(!JustinaTasks::alignWithTable(0.35))
					std::cout << "I can´t align with table   :´(" << std::endl;
				else
				{
					if(JustinaTasks::findObject(idObject_1, poseObj_1, leftArm) )
						if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z, false, idObject_1) )
							if(recoObjList.size() > 1)
								nextState = SM_TAKE_OBJECT_LEFT;
							else
								nextState = SM_GOTO_CUPBOARD;
				}
			}
			break;

			case SM_TAKE_OBJECT_LEFT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: TAKE_OBJECT_LEFT" << std::endl;
				if(!JustinaTasks::alignWithTable(0.35))
					std::cout << "I can´t align with table   :´(" << std::endl;
				else
				{
					if(JustinaTasks::findObject(idObject_2, poseObj_2, leftArm) )
						if(JustinaTasks::moveActuatorToGrasp(poseObj_2.position.x, poseObj_2.position.y, poseObj_2.position.z, true, idObject_2) )
							nextState = SM_GOTO_CUPBOARD;
				}
			}
			break;

			case SM_GOTO_CUPBOARD:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: GOTO_CUPBOARD" << std::endl;
				if(!JustinaNavigation::getClose("cupboard",200000))
			    	if(!JustinaNavigation::getClose("cupboard",200000))
			    		JustinaNavigation::getClose("cupboard",200000);
				JustinaHRI::say("I arrived to cupboard");
				nextState = SM_FIND_OBJECTS_ON_CUPBOARD;
			}
			break;

			case SM_FIND_OBJECTS_ON_CUPBOARD:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FIND_OBJECTS_ON_CUPBOARD" << std::endl;
				nextState = SM_PUT_OBJECT_ON_TABLE_RIGHT;
			}
			break;

			case SM_PUT_OBJECT_ON_TABLE_RIGHT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: PUT_OBJECT_ON_TABLE_RIGHT" << std::endl;
				if(JustinaTasks::placeObject(false))
					nextState = SM_PUT_OBJECT_ON_TABLE_LEFT;
			}
			break;

			case SM_PUT_OBJECT_ON_TABLE_LEFT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: PUT_OBJECT_ON_TABLE_LEFT" << std::endl;
				if(JustinaTasks::placeObject(true))
					nextState = SM_FINISH_TEST;
			}
			break;

			case SM_FINISH_TEST:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FINISH_TEST" << std::endl;
				nextState = -1;
			}
			break;

			default:
			{
				fail = true;
				success = true;
			}
			break;


		}
		ros::spinOnce();
		loop.sleep();
	}

	return 0;
}