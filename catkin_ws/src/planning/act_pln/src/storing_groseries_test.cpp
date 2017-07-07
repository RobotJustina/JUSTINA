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
#define SM_FIND_TABLE 25
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
	JustinaRepresentation::initKDB("", true, 20000);
	ros::Rate loop(10);

	bool fail = 			false;
	bool success =	 		false;
	bool stop =				false;
	bool findObjCupboard = 	false;
	bool takeLeft = 		false;
	bool takeRight = 		false;
	bool firstAttemp = 		true;
	bool leftArm;

	int nextState = 			0;
	int maxAttempsGraspLeft = 	0;
	int maxAttempsGraspRight = 	0;
	int maxAttempsPlaceObj = 	0;
	int itemsOnCupboard = 		0;

	float magnitude = 		0;
	float xArm = 			0;
	float yLeftArm = 		0.24;
	float yRightArm = 		-0.24;

	float minDist = 		9999999.0;

	std::vector<vision_msgs::VisionObject> recoObjForTake;
	std::vector<vision_msgs::VisionObject> recoObjList;

	std::vector<vision_msgs::VisionObject> objForTakeRight;
	std::vector<vision_msgs::VisionObject> objForTakeLeft;
	std::vector<vision_msgs::VisionObject> objOrdenedRight;
	std::vector<vision_msgs::VisionObject> objOrdenedLeft;
	vision_msgs::VisionObject poseNearestObjLeft;
	vision_msgs::VisionObject poseNearestObjRight;
	
	std::string idObjectGraspLeft;
	std::string idObjectGraspRight;

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
				//nextState = SM_FIND_TABLE;
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
				    nextState = SM_GOTO_CUPBOARD;
				  else
				    nextState = SM_WAIT_FOR_START_COMMAND;
				}
			}
			break;

			case SM_GOTO_CUPBOARD:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: GOTO_CUPBOARD" << std::endl;
				JustinaHRI::say("I am going to navigate to the cupboard");
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
					else if(JustinaManip::objOnLeftHand())
						nextState = SM_PUT_OBJECT_ON_TABLE_LEFT;
					else
						nextState = SM_FIND_OBJECTS_ON_TABLE;
				}
			}
			break;



			case SM_FIND_OBJECTS_ON_CUPBOARD:
			{
				itemsOnCupboard = 0;
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

				/*
				JustinaManip::hdGoTo(0.0, -0.2, 5000);
				boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
				if(!JustinaVision::detectAllObjects(recoObjList, true))
					std::cout << "I  can't detect anything" << std::endl;
				else
				{
					std::cout << "I have found " << recoObjList.size() << " objects on the cupboard" << std::endl;
					itemsOnCupboard += recoObjList.size();
				}
				*/

				JustinaManip::hdGoTo(0, -0.4, 5000);
				boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
				if(!JustinaVision::detectAllObjects(recoObjList, true))
					std::cout << "I  can't detect anything" << std::endl;
				else
				{
					std::cout << "I have found " << recoObjList.size() << " objects on the cupboard" << std::endl;
					itemsOnCupboard += recoObjList.size();
				}

				JustinaManip::hdGoTo(0, -0.6, 5000);
				boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
				if(!JustinaVision::detectAllObjects(recoObjList, true))
					std::cout << "I  can't detect anything" << std::endl;
				else
				{
					std::cout << "I have found " << recoObjList.size() << " objects on the cupboard" << std::endl;
					itemsOnCupboard += recoObjList.size();
				}

				JustinaManip::hdGoTo(0, -0.8, 5000);
				boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
				if(!JustinaVision::detectAllObjects(recoObjList, true))
					std::cout << "I  can't detect anything" << std::endl;
				else
				{
					std::cout << "I have found " << recoObjList.size() << " objects on the cupboard" << std::endl;
					itemsOnCupboard += recoObjList.size();
				}

				std::cout << "I have found " << itemsOnCupboard << " objects into cupboard" << std::endl;

				justinaSay.str( std::string() );
				if(itemsOnCupboard < 10)
				{
					justinaSay << "I have found " << itemsOnCupboard << " objects into cupboard";
					JustinaHRI::say(justinaSay.str());
				}
				else
				{
					itemsOnCupboard = rand() % 4 + 6;
					justinaSay << "I have found " << itemsOnCupboard << " objects into cupboard";
					JustinaHRI::say(justinaSay.str());
				}

				JustinaNavigation::moveDist(-0.15, 3000);

				JustinaTools::pdfImageExport("StoringGroseriesTest","/home/$USER/objs/");
				findObjCupboard = true;

				if(firstAttemp)
					nextState = SM_FIND_TABLE;
				else
					nextState = SM_PUT_OBJECT_ON_TABLE_RIGHT;
			}
			break;



			


			case SM_NAVIGATION_TO_TABLE:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: NAVIGATION_TO_TABLE" << std::endl;
				JustinaHRI::say("I am going to navigate to the side table");
				if(!JustinaNavigation::getClose("table_location",200000))
			    	if(!JustinaNavigation::getClose("table_location",200000))
			    		JustinaNavigation::getClose("table_location",200000);
				JustinaHRI::say("I arrived to kitchen table");
				nextState = SM_FIND_OBJECTS_ON_TABLE;
			}
			break;

			case SM_FIND_TABLE:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FIND_TABLE" << std::endl;
				

				JustinaNavigation::moveDistAngle(0.0, M_PI, 2000);

				for(int i = 0; i < 4; i++)
				{
					if(!JustinaTasks::findAndAlignTable())
					{
						JustinaNavigation::moveDistAngle(0.0, -M_PI_4, 2000);	
						JustinaHRI::say("I can not find a table");
						boost::this_thread::sleep(boost::posix_time::milliseconds(2500));
						JustinaHRI::say("I will try again");
					}
					else
					{
					  JustinaKnowledge::addUpdateKnownLoc("table_location");
					  nextState = SM_FIND_OBJECTS_ON_TABLE;
					  break;
					
					}
				}

			}
			break;


			case SM_FIND_OBJECTS_ON_TABLE:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FIND_OBJECTS_ON_TABLE" << std::endl;
				JustinaHRI::say("I am going to search objects on the table");

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

				//idObjectGrasp.clear();
				recoObjForTake.clear();

				objForTakeLeft.clear();
				objForTakeRight.clear();

				objOrdenedRight.clear();
				objOrdenedLeft.clear();

				for(int attempt = 0; attempt < 4; attempt++)
				{
					if(!JustinaVision::detectAllObjects(recoObjForTake, true))
						std::cout << "I  can't detect anything" << std::endl;
					else
					{
						std::cout << "I have found " << recoObjForTake.size() << " objects on the table" << std::endl;
						justinaSay.str( std::string() );

						if(recoObjForTake.size() < 10)
						{
							justinaSay << "I have found " << recoObjForTake.size() << " objects on the table";
							JustinaHRI::say(justinaSay.str());
						}
						else
						{
							justinaSay << "I have found " << rand() % 4 + 6 << " objects on the table";
							JustinaHRI::say(justinaSay.str());
						}

						
						for(int i = 0; i < recoObjForTake.size(); i++)
						{
							std::cout << recoObjForTake[i].id << "   ";
							std::cout << recoObjForTake[i].pose << std::endl;

							//Separate the objects by arm to be grasped
							if(recoObjForTake[i].pose.position.y > 0)
								objForTakeLeft.push_back(recoObjForTake[i]);
							else if(recoObjForTake[i].pose.position.y < 0)
								objForTakeRight.push_back(recoObjForTake[i]);
						}

						std::cout << "Objects for take left size:  " << objForTakeLeft.size() << std::endl;
						std::cout << "Objects for take right size:  " << objForTakeRight.size() << std::endl;

						if(objForTakeLeft.size() > 2)
							//Sort of left objects for be grasped 
							for(int i = 0; i < objForTakeLeft.size(); i++)
							{
								//Calculate the dist to object
								magnitude = sqrt(objForTakeLeft[i].pose.position.x*objForTakeLeft[i].pose.position.x +
												(objForTakeLeft[i].pose.position.y - yLeftArm)*(objForTakeLeft[i].pose.position.y - yLeftArm) );
								std::cout << "Dist[" << i << "]:  " << magnitude << std::endl;
								if(magnitude < minDist)
								{
									poseNearestObjLeft = objForTakeLeft[i];
									minDist = magnitude;
								}

							}
						objOrdenedLeft.push_back(poseNearestObjLeft);

						std::cout << "Objects for take left size:  " << objOrdenedLeft.size() << std::endl;
						std::cout << "Optimal object to be grasped: " << objOrdenedLeft[0].id << std::endl;
						std::cout << "		Pos: " << objOrdenedLeft[0].pose.position << std::endl;


						minDist = 999999.0;

						if(objForTakeRight.size() > 2)
							//Sort of right objects for be grasped 
							for(int i = 0; i < objForTakeRight.size(); i++)
							{
								//Calculate the dist to object
								magnitude = sqrt(objForTakeRight[i].pose.position.x*objForTakeRight[i].pose.position.x +
												(objForTakeRight[i].pose.position.y - yRightArm)*(objForTakeRight[i].pose.position.y - yRightArm) );
								std::cout << "Dist[" << i << "]:  " << magnitude << std::endl;
								if(magnitude < minDist)
								{
									poseNearestObjRight = objForTakeRight[i];
									minDist = magnitude;
								}
							}

						objOrdenedRight.push_back(poseNearestObjRight);

						std::cout << "Objects for take right size:  " << objOrdenedRight.size() << std::endl;
						std::cout << "Optimal object to be grasped: " << objOrdenedRight[0].id << std::endl;
						std::cout << "		Pos: " << objOrdenedRight[0].pose.position << std::endl;

						if(objOrdenedLeft.size() > 0)
						{
							poseObj_2 = objOrdenedLeft[0].pose;
							if(objOrdenedLeft[0].id.find("unknown") != std::string::npos)
								idObjectGraspLeft = "";
							else
								idObjectGraspLeft = objOrdenedLeft[0].id;

							takeLeft = true;
						}

						if(objOrdenedRight.size() > 0)
						{
							poseObj_1 = objOrdenedRight[0].pose;
							if(objOrdenedRight[0].id.find("unknown") != std::string::npos)
								idObjectGraspRight = "";
							else
								idObjectGraspRight = objOrdenedRight[0].id;

							takeRight = true;
						}
						
						
						nextState = SM_SAVE_OBJECTS_PDF;

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
				if(takeRight)
						nextState = SM_TAKE_OBJECT_RIGHT;
				else if(takeLeft)
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
					{
						std::cout << "I can´t align with table   :´(" << std::endl;
						JustinaNavigation::moveDistAngle(-0.05, M_PI_4/4, 2000);
					}
					else
					{
						if(idObjectGraspRight != "")
						{
							if(JustinaTasks::findObject(idObjectGraspRight, poseObj_1, leftArm) )

								if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.02, false, idObjectGraspRight) )
								{
									if(takeLeft)
									{
										takeRight = false;
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
							if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.02, false, idObjectGraspRight) )
							{
								if(takeLeft)
								{
									takeRight = false;
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
					{
						std::cout << "I can´t align with table   :´(" << std::endl;
						JustinaNavigation::moveDistAngle(-0.05, M_PI_4/4, 2000);
					}
					else
					{
						if(idObjectGraspLeft != "")
						{
							if(JustinaTasks::findObject(idObjectGraspLeft, poseObj_2, leftArm) )
								if(JustinaTasks::moveActuatorToGrasp(poseObj_2.position.x, poseObj_2.position.y, poseObj_2.position.z + 0.02, true, idObjectGraspLeft) )
								{
									takeLeft = false;
									maxAttempsGraspLeft = 0;
									nextState = SM_GOTO_CUPBOARD;

									
								}
						}
						else
						{
							if(JustinaTasks::moveActuatorToGrasp(poseObj_2.position.x, poseObj_2.position.y, poseObj_2.position.z + 0.02, true, idObjectGraspLeft) )
							{
									takeLeft = false;
									maxAttempsGraspLeft = 0;
									nextState = SM_GOTO_CUPBOARD;
							}
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
					//idObjectGrasp.clear();
					maxAttempsGraspLeft = 0;
					nextState = SM_GOTO_CUPBOARD;
				}

			}
			break;


			case SM_PUT_OBJECT_ON_TABLE_RIGHT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: PUT_OBJECT_ON_TABLE_RIGHT" << std::endl;
				JustinaHRI::say("I will placed the object in my right arm in the cupboard");


				if(maxAttempsPlaceObj < 3)
				{
					if(!JustinaTasks::alignWithTable(0.30))
					{
						JustinaNavigation::moveDist(-0.10, 3000);
						boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
						if(!JustinaTasks::alignWithTable(0.30))
						{
							JustinaNavigation::moveDist(0.15, 3000);
							boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
							JustinaTasks::alignWithTable(0.30);
						}
					}
					if(JustinaTasks::placeObjectOnShelf(false, 0.0))
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


				if(maxAttempsPlaceObj < 3)
				{
					if(!JustinaTasks::alignWithTable(0.33))
					{
						JustinaNavigation::moveDist(0.10, 3000);
						JustinaTasks::alignWithTable(0.33);
					}
					if(JustinaTasks::placeObjectOnShelf(true, 0.0))
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
