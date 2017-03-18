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
#define SM_WAIT_FOR_START_COMMAND 10
#define SM_NAVIGATION_TO_TABLE 20
#define SM_FIND_OBJECTS_ON_TABLE 30
#define SM_SAVE_OBJECTS_PDF 40
#define SM_TAKE_OBJECT_RIGHT 50
#define SM_TAKE_OBJECT_LEFT 60
#define SM_GOTO_CUPBOARD 70
#define SM_FIND_OBJECTS_ON_CUPBOARD 80
#define SM_PUT_OBJECTS_ON_TABLE 90
#define SM_FINISH_TEST 100




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


	int nextState = 0;
	bool fail = false;
	bool success = false;
	bool stop=false;



	std::string lastRecoSpeech;
	std::vector<std::string> validCommands;

	vision_msgs::FindPlane fp;
	fp.request.name="";

	//ros::ServiceClient client = n.serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/findPlane");
	ros::ServiceClient client = n.serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/freePlanes");

	float robot_y,robot_x,robot_a;

	validCommands.push_back("robot follow me");
	validCommands.push_back("stop");
	validCommands.push_back("continue");
	validCommands.push_back("this is the table one");
	validCommands.push_back("this is the table two");
	validCommands.push_back("this is the table three");
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
				std::cout << "State machine: INIT" << std::endl;
				//JustinaHRI::say("I'm ready for the restaurant test");
				//sleep(1);
				//JustinaHRI::say("I'm waiting for the Professional Waiter");
				client.call(fp);
				sleep(2);
				nextState = SM_WAIT_FOR_START_COMMAND;
			}
			break;

			case SM_WAIT_FOR_START_COMMAND:
			{
				std::cout << "State machine: WAIT_FOR_START_COMMAND" << std::endl;
				nextState = SM_NAVIGATION_TO_TABLE;
			}
			break;

			case SM_NAVIGATION_TO_TABLE:
			{
				std::cout << "State machine: NAVIGATION_TO_TABLE" << std::endl;
				nextState = SM_FIND_OBJECTS_ON_TABLE;
			}
			break;


			case SM_FIND_OBJECTS_ON_TABLE:
			{
				std::cout << "State machine: FIND_OBJECTS_ON_TABLE" << std::endl;
				nextState = SM_SAVE_OBJECTS_PDF;
			}
			break;

			case SM_SAVE_OBJECTS_PDF:
			{
				std::cout << "State machine: SAVE_OBJECTS_PDF" << std::endl;
				nextState = SM_TAKE_OBJECT_RIGHT;
			}
			break;

			case SM_TAKE_OBJECT_RIGHT:
			{
				std::cout << "State machine: TAKE_OBJECT_RIGHT" << std::endl;
				nextState = SM_TAKE_OBJECT_LEFT;
			}
			break;

			case SM_TAKE_OBJECT_LEFT:
			{
				std::cout << "State machine: TAKE_OBJECT_LEFT" << std::endl;
				nextState = SM_GOTO_CUPBOARD;
			}
			break;

			case SM_GOTO_CUPBOARD:
			{
				std::cout << "State machine: GOTO_CUPBOARD" << std::endl;
				nextState = SM_FIND_OBJECTS_ON_CUPBOARD;
			}
			break;

			case SM_FIND_OBJECTS_ON_CUPBOARD:
			{
				std::cout << "State machine: FIND_OBJECTS_ON_CUPBOARD" << std::endl;
				nextState = SM_PUT_OBJECTS_ON_TABLE;
			}
			break;

			case SM_PUT_OBJECTS_ON_TABLE:
			{
				std::cout << "State machine: PUT_OBJECTS_ON_TABLE" << std::endl;
				nextState = SM_FINISH_TEST;
			}
			break;

			case SM_FINISH_TEST:
			{
				std::cout << "State machine: FINISH_TEST" << std::endl;
				nextState = SM_INIT;
			}
			break;

		}
		ros::spinOnce();
		loop.sleep();
	}

	return 0;
}








