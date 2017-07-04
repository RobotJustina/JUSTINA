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

enum task  
{   
    SM_INIT, 
    SM_WAIT_FOR_DOOR,
    SM_WAIT_FOR_START_COMMAND, 
    SM_NAVIGATION_TO_TABLE, 
    SM_NAVIGATION_TO_RACK,  
    SM_NAVIGATION_TO_CUPBOARD,  
    SM_FIND_OBJECTS_ON_TABLE, 
    SM_FIND_OBJECTS_ON_RACK, 
    SM_FIND_OBJECTS_ON_CUPBOARD, 
    SM_SAVE_OBJECTS_PDF, 
    SM_TAKE_OBJECT_RIGHT, 
    SM_TAKE_OBJECT_LEFT, 
    SM_PUT_OBJECT_ON_TABLE_RIGHT, 
    SM_PUT_OBJECT_ON_TABLE_LEFT, 
    SM_FINISH_TEST,
    SM_WAIT_FOR_COMMAND
};

enum food 
{
   //choose the best easy to grasp food objects
    JUICE,
    PRINGLES,
    WEBO_LATE 
};

enum cutlery 
{
   //choose the best easy to grasp food objects
    CUP,
    PLATE,
    KINFE 
};

int main(int argc, char** argv)
{
	std::cout << "INITIALIZING ACT_PLN SET UP TABLE AND CLEAN IT UP TEST by EL URUGUAYO..." << std::endl;
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
	JustinaRepresentation::initKDB("", true, 20000);   //what this parameters means?
	ros::Rate loop(10);                                //what this line do?


	task nextState               = SM_INIT;
	int maxAttempsGraspLeft     = 0;
	int maxAttempsGraspRight    = 0;
	int maxAttempsPlaceObj      = 0;

	bool fail               = false;
	bool success            = false;
	bool stop               = false;
	bool findObjCupboard    = false;
	bool leftArm;

	std::vector<vision_msgs::VisionObject> recoObjForTake;
	std::vector<vision_msgs::VisionObject> recoObjList;
	std::vector<std::string> idObjectGrasp;

	std::stringstream justinaSay;

	geometry_msgs::Pose poseObj_1;
	geometry_msgs::Pose poseObj_2;

	std::string lastRecoSpeech;
	std::vector<std::string> validCommands;
	validCommands.push_back("robot yes");
	validCommands.push_back("robot no");
	validCommands.push_back("continue");


	while(ros::ok() && !fail && !success)
	{
		switch(nextState)
		{

			case SM_INIT:
			{
				std::cout << "----->  State machine: INIT" << std::endl;
				JustinaHRI::waitAfterSay("I'm ready for set up table and clean it up test", 4000);
				nextState = SM_WAIT_FOR_DOOR;
                break;
			}

            case SM_WAIT_FOR_DOOR:
            {
                if(!JustinaNavigation::obstacleInFront())
                {
                    nextState = SM_NAVIGATION_TO_TABLE;
                }else{
                    JustinaHRI::waitAfterSay("Please, can you open de door for me?", 4000);
                    nextState = SM_WAIT_FOR_DOOR;
                }
                break;
            }

            case SM_NAVIGATION_TO_TABLE:
            {
                JustinaHRI::waitAfterSay("I can see that the door is open, I am navigating to the table", 4000);
                if(!JustinaNavigation::getClose("table", 180000))
                    if(!JustinaNavigation::getClose("table", 180000))
                        if(!JustinaNavigation::getClose("table", 180000))
                JustinaHRI::waitAfterSay("I have arrived to the table", 4000);
                JustinaHRI::waitAfterSay("Do you want me to set up the table for you?. Please anwser robot yes or robot no", 4000);
                nextState = SM_WAIT_FOR_START_COMMAND;
                break;

            }

			case SM_WAIT_FOR_START_COMMAND:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: WAIT_FOR_START_COMMAND" << std::endl;
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))   //what are this parameters?
                    JustinaHRI::waitAfterSay("Please repeat the command", 4000);
				else
				{
				  if(lastRecoSpeech.find("robot yes") != std::string::npos)
				    nextState = SM_FIND_OBJECTS_ON_TABLE;                      //in the table is the client - search for the face?
                    // nextState = SM_FINISH_TEST;
				  else
				    nextState = SM_WAIT_FOR_START_COMMAND;
				}
                break;
			}
//ask someone to open de door
//got to de table and wait for de instruction "robot please set up a table"
//o: could you serve the table, please
//r: yes, madame. Would you preffer tea and cookies or yogurt and pringles?
//o: I prefer tea and cookies
//r: Please confirm tea and cookies?
//o: Robot yes.
//r: I will set up the table for you.


//    O: Robot, set the table.
//    R: You want me to set up the table. Is that correct?
//    O: Robot, yes.
//    R: Understood. I will set up the table. If you want me to place the default setup, say: robot, default setup.
//    R: If you want me to serve frosties with milk, say: robot, serve the frosties option.
//    R: If you want me to serve choco-flakes with milk, say: robot, serve the choco-flakes option.
//    R: If you want me to serve apple and orange juice, say: robot, serve the apple option.
//    R: Please tell me, which option do you want me to serve?
//    O: Robot, please serve the choco-flakes option.
//    R: You said: Robot, serve the choco-flakes option. Is that correct?
//    O: Robot, yes.
//    R: Ok. I will set the table for serving choco-flakes. Please wait.

            //This is to know which objects are missing in the table
            case SM_FIND_OBJECTS_ON_TABLE:
            {
                std::cout << "" << std::endl;
                std::cout << "" << std::endl;
                std::cout << "----->  State machine: FIND_OBJECTS_ON_TABLE" << std::endl;
                JustinaHRI::waitAfterSay("I am going to check for objects on the table", 4000);

                if(!JustinaTasks::alignWithTable(0.35))
                {
                    JustinaNavigation::moveDist(0.10, 3000);
                    if(!JustinaTasks::alignWithTable(0.35))
                    {
                        std::cout << "I can´t alignWithTable... :'(" << std::endl;
                        JustinaNavigation::moveDist(-0.15, 3000);
                        //nextState = SM_NAVIGATION_TO_RACK;
                        JustinaHRI::waitAfterSay("I cant align myself with the table", 4000);
                        nextState = SM_FINISH_TEST;
                        break;
                    }
                }

                idObjectGrasp.clear();
                recoObjForTake.clear();
                for(int attempt = 0; attempt < 4; attempt++)
                {
                    if(!JustinaVision::detectAllObjects(recoObjForTake, true))
                    {
                        std::cout << "I  can't detect anything" << std::endl;
                        if (attempt == 3) nextState = SM_FINISH_TEST;
                    }else
                    {
                        std::cout << "I have found " << recoObjForTake.size() << " objects on the side table" << std::endl;
                        justinaSay.str( std::string() );
                        justinaSay << "I have found " << recoObjForTake.size() << " objects on the side table";
                        JustinaHRI::waitAfterSay(justinaSay.str(), 4000);

                        for(int i = 0; i < recoObjForTake.size(); i++)
                        {
                            //Here Justina has to take count about which objects are in the table and only pick the missing ones.
                            std::cout << recoObjForTake[i].id << "   ";
                            std::cout << recoObjForTake[i].pose << std::endl;

                            if(recoObjForTake[i].id.find("unknown") != std::string::npos)
                                idObjectGrasp.push_back("");
                            else
                                idObjectGrasp.push_back(recoObjForTake[i].id);
                        }
                        break;
                    }

                }
                nextState = SM_NAVIGATION_TO_RACK;
                break;
            }


			case SM_NAVIGATION_TO_RACK:
			{
                //FIXME::where is set the initial pose?
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: NAVIGATION_TO_RACK" << std::endl;
                JustinaHRI::waitAfterSay("I am going to navigate to the rack and bring the food", 4000);
				if(!JustinaNavigation::getClose("rack",200000))   
			    	if(!JustinaNavigation::getClose("rack",200000))  
			    		JustinaNavigation::getClose("rack",200000);  
                JustinaHRI::waitAfterSay("I arrived to kitchen rack", 4000);
				nextState = SM_FIND_OBJECTS_ON_RACK;
                break;
			}


			case SM_FIND_OBJECTS_ON_RACK:   //FIXME:check objects or check food/?
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FIND_OBJECTS_ON_RACK" << std::endl;
                JustinaHRI::waitAfterSay("I am going to search for food on the rack", 4000);

				if(!JustinaTasks::alignWithTable(0.35))
				{
					JustinaNavigation::moveDist(0.10, 3000);
					if(!JustinaTasks::alignWithTable(0.35))
					{
						std::cout << "I can´t alignWithTable... :'(" << std::endl;
						JustinaNavigation::moveDist(-0.15, 3000);
                        nextState = SM_FINISH_TEST;
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

					}

				}
            nextState = SM_FINISH_TEST;
			break;
			}



            /*  
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
					nextState = SM_FIND_OBJECTS_ON_RACK;
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
											nextState = SM_NAVIGATION_TO_CUPBOARD;
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
											nextState = SM_NAVIGATION_TO_CUPBOARD;
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
									nextState = SM_NAVIGATION_TO_CUPBOARD;
								}
						}
						else
						{
							if(JustinaTasks::moveActuatorToGrasp(poseObj_2.position.x, poseObj_2.position.y, poseObj_2.position.z + 0.03, true, idObjectGrasp[0]) )
									maxAttempsGraspLeft = 0;
									nextState = SM_NAVIGATION_TO_CUPBOARD;
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
					nextState = SM_NAVIGATION_TO_CUPBOARD;
				}

			}
			break;



			case SM_NAVIGATION_TO_CUPBOARD:
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
            */


			case SM_FINISH_TEST:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FINISH_TEST" << std::endl;
				boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
                JustinaHRI::say("I have finish the test.");
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
