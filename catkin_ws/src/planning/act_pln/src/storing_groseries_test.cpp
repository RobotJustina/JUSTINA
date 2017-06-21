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
#include "justina_tools/JustinaRepresentation.h"
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
	std::cout << "INITIALIZING ACT_PLN STORING GROSERIES TEST by EDGAR-II    ..." << std::endl;
	ros::init(argc, argv, "act_pln");
	ros::NodeHandle n;
	JustinaHardware::setNodeHandle(&n);
	JustinaHRI::setNodeHandle(&n);
	JustinaManip::setNodeHandle(&n);
	JustinaNavigation::setNodeHandle(&n);
	JustinaTools::setNodeHandle(&n);
	JustinaVision::setNodeHandle(&n);
	JustinaTasks::setNodeHandle(&n);

	JustinaRepresentation::setNodeHandle(&n);
	JustinaRepresentation::initKDB("", true);
	ros::Rate loop(10);


	int nextState = 0;
	int maxAttempsGraspLeft = 0;
	int maxAttempsGraspRight = 0;
	int maxAttempsPlaceObj = 0;

	bool fail = false;
	bool success = false;
	bool stop=false;
	bool findObjCupboard = true;
	bool leftArm;

	std::vector<vision_msgs::VisionObject> recoObjForTake;
	std::vector<vision_msgs::VisionObject> recoObjList;
	std::vector<std::string> idObjectGrasp;

	std::string lastRecoSpeech;
	std::stringstream justinaSay;

	geometry_msgs::Pose poseObj_1;
	geometry_msgs::Pose poseObj_2;

	std::vector<std::string> validCommands;
	validCommands.push_back("robot start");


	while(ros::ok() && !fail && !success)
	{
		switch(nextState)
		{

			case SM_INIT:
			{
				std::cout << "----->  State machine: INIT" << std::endl;
				JustinaHRI::say("I'm ready for storing groseries test");
				boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
				JustinaHRI::say("I'm waiting for the start command");
				nextState = SM_WAIT_FOR_START_COMMAND;
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
				JustinaHRI::say("I am going to navigate to the side table");
				if(!JustinaNavigation::getClose("table",200000))
			    	if(!JustinaNavigation::getClose("table",200000))
			    		JustinaNavigation::getClose("table",200000);
				JustinaHRI::say("I arrived to kitchen table");
				nextState = SM_FIND_OBJECTS_ON_TABLE;
			}
			break;



			case SM_FIND_OBJECTS_ON_TABLE:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FIND_OBJECTS_ON_TABLE" << std::endl;
				JustinaHRI::say("I am going to search objects on the kitchen table");

				if(!JustinaTasks::alignWithTable(0.35))
				{
					JustinaNavigation::moveDist(0.10, 3000);
					if(!JustinaTasks::alignWithTable(0.35))
					{
						std::cout << "I can´t alignWithTable... :'(" << std::endl;
						JustinaNavigation::moveDist(-0.15, 3000);
						break;
					}
				}


				idObjectGrasp.clear();
				recoObjForTake.clear();
				for(int attempt = 0; attempt < 4; attempt++)
				{
					if(!JustinaVision::detectAllObjects(recoObjForTake, true))
						std::cout << "I  can't detect anything" << std::endl;
					else
					{
						std::cout << "I have found " << recoObjForTake.size() << " objects on the side table" << std::endl;
						justinaSay.str( std::string() );
						justinaSay << "I have found " << recoObjForTake.size() << " objects on the side table";
						JustinaHRI::say(justinaSay.str());

						for(int i = 0; i < recoObjForTake.size(); i++)
						{
							std::cout << recoObjForTake[i].id << "   ";
							std::cout << recoObjForTake[i].pose << std::endl;

							if(recoObjForTake[i].id.find("unknown") != std::string::npos)
								idObjectGrasp.push_back("");
							else
								idObjectGrasp.push_back(recoObjForTake[i].id);
						}

						if(idObjectGrasp.size() > 1)
						{
							poseObj_1 = recoObjForTake[0].pose;
							poseObj_2 = recoObjForTake[1].pose;
							nextState = SM_SAVE_OBJECTS_PDF;
						}
						else if( idObjectGrasp.size() > 0)
						{
							poseObj_1 = recoObjForTake[0].pose;
							nextState = SM_SAVE_OBJECTS_PDF;
						}

						break;
					}

				}

			}
			break;



			case SM_SAVE_OBJECTS_PDF:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: SAVE_OBJECTS_PDF" << std::endl;
				JustinaTools::pdfImageExport("StoringGroseriesTest","/home/$USER/objs/");
				if(idObjectGrasp.size() > 1)
						nextState = SM_TAKE_OBJECT_RIGHT;
				else if(idObjectGrasp.size() > 0)
						nextState = SM_TAKE_OBJECT_LEFT;
				else
					nextState = SM_FIND_OBJECTS_ON_TABLE;
			}
			break;



			case SM_TAKE_OBJECT_RIGHT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: TAKE_OBJECT_RIGHT" << std::endl;
				JustinaHRI::say("I am going to take object whit my right arm");
				boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

				if (maxAttempsGraspRight < 3)
				{
					if(!JustinaTasks::alignWithTable(0.35))
						std::cout << "I can´t align with table   :´(" << std::endl;
					else
					{
						if(idObjectGrasp[1] != "")
						{
								if(JustinaTasks::findObject(idObjectGrasp[1], poseObj_1, leftArm) )

									if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, false, idObjectGrasp[1]) )
									{
										if(recoObjForTake.size() > 1)
										{
											maxAttempsGraspRight = 0;
											nextState = SM_TAKE_OBJECT_LEFT;
										}
										else
										{
											maxAttempsGraspRight = 0;
											nextState = SM_GOTO_CUPBOARD;
										}
									}
									else
									{
										std::cout << "I can´t grasp objects in " << maxAttempsGraspRight << " attempt" << std::endl;
									}
						}
						else
						{
									//If the object is unknown, not find again....
									if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, false, idObjectGrasp[1]) )
									{
										if(recoObjForTake.size() > 1)
										{
											maxAttempsGraspRight = 0;
											nextState = SM_TAKE_OBJECT_LEFT;
										}
										else
										{
											maxAttempsGraspRight = 0;
											nextState = SM_GOTO_CUPBOARD;
										}
									}
									else
									{
										std::cout << "I can´t grasp objects in " << maxAttempsGraspRight << " attempts" << std::endl;
									}

						}

					}
					maxAttempsGraspRight++;
				}
				else
				{
					maxAttempsGraspRight = 0;
					nextState = SM_TAKE_OBJECT_LEFT;
				}

			}
			break;



			case SM_TAKE_OBJECT_LEFT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: TAKE_OBJECT_LEFT" << std::endl;

				JustinaHRI::say("I am going to take object whit my left arm");
				boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

				if(maxAttempsGraspLeft < 3)
				{
					if(!JustinaTasks::alignWithTable(0.35))
						std::cout << "I can´t align with table   :´(" << std::endl;
					else
					{
						if(idObjectGrasp[0] != "")
						{
							if(JustinaTasks::findObject(idObjectGrasp[0], poseObj_2, leftArm) )
								if(JustinaTasks::moveActuatorToGrasp(poseObj_2.position.x, poseObj_2.position.y, poseObj_2.position.z + 0.03, true, idObjectGrasp[0]) )
								{
									maxAttempsGraspLeft = 0;
									nextState = SM_GOTO_CUPBOARD;
								}
						}
						else
						{
							if(JustinaTasks::moveActuatorToGrasp(poseObj_2.position.x, poseObj_2.position.y, poseObj_2.position.z + 0.03, true, idObjectGrasp[0]) )
									maxAttempsGraspLeft = 0;
									nextState = SM_GOTO_CUPBOARD;
						}
					}

					maxAttempsGraspLeft++;
				}
				else
				{
					if(JustinaManip::objOnRightHand())
						nextState = SM_PUT_OBJECT_ON_TABLE_RIGHT;
					else
						nextState = SM_FIND_OBJECTS_ON_TABLE;
					recoObjForTake.clear();
					maxAttempsGraspLeft = 0;
					nextState = SM_GOTO_CUPBOARD;
				}

			}
			break;



			case SM_GOTO_CUPBOARD:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: GOTO_CUPBOARD" << std::endl;
				JustinaHRI::say("I am going to navigate to the shelf");
				if(!JustinaNavigation::getClose("cupboard",200000))
			    	if(!JustinaNavigation::getClose("cupboard",200000))
			    		JustinaNavigation::getClose("cupboard",200000);
				JustinaHRI::say("I arrived to the cupboard");
				if(!findObjCupboard)
					nextState = SM_FIND_OBJECTS_ON_CUPBOARD;
				else
				{
					if(JustinaManip::objOnRightHand())
						nextState = SM_PUT_OBJECT_ON_TABLE_RIGHT;
					else
					{
						if(JustinaManip::objOnLeftHand())
							nextState = SM_PUT_OBJECT_ON_TABLE_LEFT;
						else
							nextState = SM_NAVIGATION_TO_TABLE;
					}
				}
			}
			break;



			case SM_FIND_OBJECTS_ON_CUPBOARD:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FIND_OBJECTS_ON_CUPBOARD" << std::endl;
				JustinaHRI::say("I am going to search objects on the shelf");

				JustinaManip::hdGoTo(0, -0.5, 5000);
				if(!JustinaTasks::alignWithTable(0.40))
				{
					JustinaNavigation::moveDist(0.15, 3000);
					if(!JustinaTasks::alignWithTable(0.40))
						JustinaTasks::alignWithTable(0.40);
				}

				JustinaManip::hdGoTo(0.0, -0.2, 5000);
				boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
				if(!JustinaVision::detectAllObjects(recoObjList, true))
					std::cout << "I  can't detect anything" << std::endl;
				else
				{
					std::cout << "I have found " << recoObjList.size() << " objects on the cupboard" << std::endl;
				}

				JustinaManip::hdGoTo(0, -0.4, 5000);
				boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
				if(!JustinaVision::detectAllObjects(recoObjList, true))
					std::cout << "I  can't detect anything" << std::endl;
				else
				{
					std::cout << "I have found " << recoObjList.size() << " objects on the cupboard" << std::endl;
				}

				JustinaManip::hdGoTo(0, -0.6, 5000);
				boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
				if(!JustinaVision::detectAllObjects(recoObjList, true))
					std::cout << "I  can't detect anything" << std::endl;
				else
				{
					std::cout << "I have found " << recoObjList.size() << " objects on the cupboard" << std::endl;
				}

				JustinaManip::hdGoTo(0, -0.8, 5000);
				boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
				if(!JustinaVision::detectAllObjects(recoObjList, true))
					std::cout << "I  can't detect anything" << std::endl;
				else
				{
					std::cout << "I have found " << recoObjList.size() << " objects on the cupboard" << std::endl;
				}

				JustinaTools::pdfImageExport("StoringGroseriesTest","/home/$USER/objs/");
				findObjCupboard = true;
				nextState = SM_PUT_OBJECT_ON_TABLE_RIGHT;
			}
			break;



			case SM_PUT_OBJECT_ON_TABLE_RIGHT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: PUT_OBJECT_ON_TABLE_RIGHT" << std::endl;
				JustinaHRI::say("I will placed the object in my right arm in the cupboard");


				if(maxAttempsPlaceObj < 4)
				{
					if(!JustinaTasks::alignWithTable(0.30))
					{
						JustinaNavigation::moveDist(0.15, 3000);
						boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
						if(!JustinaTasks::alignWithTable(0.30))
						{
							JustinaNavigation::moveDist(0.15, 3000);
							boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
							JustinaTasks::alignWithTable(0.30);
						}
					}
					if(JustinaTasks::placeObject(false))
					{
						if(JustinaManip::objOnLeftHand())
							nextState = SM_PUT_OBJECT_ON_TABLE_LEFT;
						else
							nextState = SM_NAVIGATION_TO_TABLE;
						maxAttempsPlaceObj = 0;
					}
					maxAttempsPlaceObj++;
				}
				else
				{
					maxAttempsPlaceObj = 0;
					std::cout << "I can´t placed objects on cupboard whit right Arm" << std::endl;
					JustinaHRI::say("I can´t found a free place in the cupboard");
					if(JustinaManip::objOnLeftHand())
							nextState = SM_PUT_OBJECT_ON_TABLE_LEFT;
						else
							nextState = SM_NAVIGATION_TO_TABLE;
				}
			}
			break;



			case SM_PUT_OBJECT_ON_TABLE_LEFT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: PUT_OBJECT_ON_TABLE_LEFT" << std::endl;
				JustinaHRI::say("I will placed the object in my left arm in the cupboard");


				if(maxAttempsPlaceObj < 4)
				{
					if(!JustinaTasks::alignWithTable(0.33))
					{
						JustinaNavigation::moveDist(0.10, 3000);
						JustinaTasks::alignWithTable(0.33);
					}
					if(JustinaTasks::placeObject(true))
						nextState = SM_NAVIGATION_TO_TABLE;
					maxAttempsPlaceObj++;
				}
				else
				{
					std::cout << "I can´t placed objects on cupboard whit left Arm" << std::endl;
					JustinaHRI::say("I can´t found a free place in the cupboard");
					nextState = SM_INIT;
				}
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
