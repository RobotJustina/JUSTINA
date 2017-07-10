#include <iostream>
#include <algorithm>
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

#define MAX_OBJ_SATTU     3
#define MENU_1_drink      "juice"
#define MENU_1_food       "pringles"
#define MENU_2_drink      "milk"
#define MENU_2_food       "peanuts"
#define DELAY_SPEAK       7000
#define DELAY_AFTER_SPEAK 2000

enum task  
{   
    SM_INIT, 
    SM_WAIT_FOR_DOOR,
    SM_WAIT_FOR_START_COMMAND, 
    SM_WAIT_FOR_CHOOSE_COMMAND,
    SM_OFFER_MENUS,
    SM_NAVIGATION_TO_TABLE, 
    SM_INIT_COMMAND,
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



struct elemState
{
	std::vector<std::string> name;
    bool    inTable[MAX_OBJ_SATTU];
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


	//task nextState               = SM_INIT;
	task nextState               = SM_INIT_COMMAND;

    std::map<std::string, bool> obj_localiz;
    obj_localiz.insert( std::pair<std::string, bool>("milk", false));
    obj_localiz.insert( std::pair<std::string, bool>("cup", false));
    obj_localiz.insert( std::pair<std::string, bool>("juice", false));
    obj_localiz.insert( std::pair<std::string, bool>("peanuts", false));
	//std::vector<std::string> obj_on_table;
    std::set<std::string> obj_on_table;
    
	int maxAttempsGraspLeft     = 0;
	int maxAttempsGraspRight    = 0;
	int maxAttempsPlaceObj      = 0;

	bool fail               = false;
	bool success            = false;
	bool stop               = false;
	bool findObjCupboard    = false;
	bool leftArm;
    bool rackVisited        = false;
    bool cupboardVisited    = false;
    int  menu_selected      = 1;

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
	validCommands.push_back("menu one");
	validCommands.push_back("menu two");
	validCommands.push_back("continue");

	while(ros::ok() && !fail && !success)
	{
		switch(nextState)
		{

			case SM_INIT:
			{
				std::cout << "----->  State machine: INIT" << std::endl;
				JustinaHRI::waitAfterSay("I'm ready for set up table and clean it up test", DELAY_SPEAK);
				nextState = SM_WAIT_FOR_DOOR;
                break;
			}

            case SM_WAIT_FOR_DOOR:
            {
                if(!JustinaNavigation::obstacleInFront())
                {
                    JustinaHRI::waitAfterSay("I can see that the door is open, I am navigating to the table", DELAY_SPEAK);
                    nextState = SM_NAVIGATION_TO_TABLE;
                }
                else
                {
                    JustinaHRI::waitAfterSay("Please, can you open de door for me?", DELAY_SPEAK);
                    nextState = SM_WAIT_FOR_DOOR;
                }
                break;
            }
            
            case SM_NAVIGATION_TO_TABLE:
            {
                if(!JustinaNavigation::getClose("table", 180000))
                    if(!JustinaNavigation::getClose("table", 180000))
                        if(!JustinaNavigation::getClose("table", 180000))
                JustinaHRI::waitAfterSay("I have arrived to the table", 4000);
                nextState = SM_INIT_COMMAND;
                break;
            }    

            case SM_INIT_COMMAND:
            {
                if (!rackVisited && !cupboardVisited)
                {
                    JustinaHRI::waitAfterSay("Do you want me to set up the table for you?. Please anwser robot yes or robot no", DELAY_SPEAK);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(DELAY_AFTER_SPEAK));
                    nextState = SM_WAIT_FOR_START_COMMAND;
                    lastRecoSpeech.clear();
                }
                else if (rackVisited && !cupboardVisited)
                {
                    nextState = SM_NAVIGATION_TO_CUPBOARD;
                }
                else if (rackVisited && cupboardVisited)
                {
                    JustinaHRI::waitAfterSay("Enjoy your meal. Let me know when you finish it by saying i have finish.", DELAY_SPEAK);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(DELAY_AFTER_SPEAK));
                    nextState = SM_FINISH_TEST;
                }       
                break;
            }

			case SM_WAIT_FOR_START_COMMAND:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: WAIT_FOR_START_COMMAND" << std::endl;
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))   //what are this parameters?
                {
                    JustinaHRI::waitAfterSay("Please repeat the command", DELAY_SPEAK);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(DELAY_AFTER_SPEAK));
                }
                else
				{
				  if(lastRecoSpeech.find("robot yes") != std::string::npos)
                  {
				    nextState = SM_OFFER_MENUS;
				  }
                    else if(lastRecoSpeech.find("robot no") != std::string::npos)
                  {
                    nextState = SM_FINISH_TEST;
                    //nextState = SM_WAIT_FOR_START_COMMAND;
				  }
                  else
				  {
                    nextState = SM_WAIT_FOR_START_COMMAND;
				  }
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
            case SM_OFFER_MENUS:
            {
                
                justinaSay.str( std::string() );
                justinaSay << "If you prefer " << MENU_1_drink << " and " << MENU_1_food <<  " please say menu one, else If you prefer " << MENU_2_drink << " and " << MENU_2_food << " please say menu two";
                JustinaHRI::waitAfterSay(justinaSay.str(), DELAY_SPEAK);
                boost::this_thread::sleep(boost::posix_time::milliseconds(DELAY_AFTER_SPEAK));
                nextState = SM_WAIT_FOR_CHOOSE_COMMAND;
                lastRecoSpeech.clear();
                break;
            }

			case SM_WAIT_FOR_CHOOSE_COMMAND:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: WAIT_FOR_CHOOSE_COMMAND" << std::endl;
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))   //what are this parameters?
                { 
                    JustinaHRI::waitAfterSay("Please repeat menu one or meno two", DELAY_SPEAK);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(DELAY_AFTER_SPEAK));
                }
                else
				{
				  if(lastRecoSpeech.find("menu one") != std::string::npos)
                  {
                    menu_selected = 1;
                    justinaSay.str( std::string() );
                    justinaSay << "You asked for " << MENU_1_drink << " and " << MENU_1_food << ", I am going to set up your order.";
                    JustinaHRI::waitAfterSay(justinaSay.str(), DELAY_SPEAK);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(DELAY_AFTER_SPEAK));
				    nextState = SM_FIND_OBJECTS_ON_TABLE;                      //FIXME:save info about menu one anywhere
				  }
                  else if(lastRecoSpeech.find("menu two") != std::string::npos)
                  {
                    menu_selected = 2;
                    justinaSay.str( std::string() );
                    justinaSay << "You asked for " << MENU_2_drink << " and " << MENU_2_food << ", I am going to set up your order.";
                    JustinaHRI::waitAfterSay(justinaSay.str(), DELAY_SPEAK);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(DELAY_AFTER_SPEAK));
				    nextState = SM_FIND_OBJECTS_ON_TABLE;                      //FIXME:save info about menu one anywhere
				  }
                  else
				  {
                    nextState = SM_WAIT_FOR_CHOOSE_COMMAND;
                    //should i have to do a lastrecospeech.clear()?
				  }
                }
                break;
			}
            

            case SM_FIND_OBJECTS_ON_TABLE:
            {
                std::cout << "" << std::endl;
                std::cout << "" << std::endl;
                std::cout << "----->  State machine: FIND_OBJECTS_ON_TABLE" << std::endl;
                JustinaHRI::waitAfterSay("I am going to check which objects are already on the table", DELAY_SPEAK);

                if(!JustinaTasks::alignWithTable(0.35))
                {
                    JustinaNavigation::moveDist(0.10, 3000);
                    if(!JustinaTasks::alignWithTable(0.35))
                    {
                        std::cout << "I can´t alignWithTable... :'(" << std::endl;
                        JustinaNavigation::moveDist(-0.15, 3000);
                        JustinaHRI::waitAfterSay("I cant align myself with the table", DELAY_SPEAK);
                        nextState = SM_NAVIGATION_TO_RACK;
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
                        if (attempt == 3) 
                        {    
                            JustinaHRI::waitAfterSay("There are no objects on the table", DELAY_SPEAK);
                        }
                    }
                    else
                    {
                        std::cout << "I have found " << recoObjForTake.size() << " objects on the table" << std::endl;
                        justinaSay.str( std::string() );
                        justinaSay << "I have found " << recoObjForTake.size() << " objects on the table";
                        JustinaHRI::waitAfterSay(justinaSay.str(), 4000);

                        for(int i = 0; i < recoObjForTake.size(); i++)
                        {
                              obj_on_table.insert (recoObjForTake[i].id);
                        }
                        nextState = SM_NAVIGATION_TO_RACK;
                        break;
                    }

                }
                nextState = SM_NAVIGATION_TO_RACK;
                break;
            }


			case SM_NAVIGATION_TO_RACK:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: NAVIGATION_TO_RACK" << std::endl;
                JustinaHRI::waitAfterSay("I am going to navigate to the rack and bring the food", DELAY_SPEAK);
                //specify which food they are going to pick
				if(!JustinaNavigation::getClose("rack",200000))   
			    	if(!JustinaNavigation::getClose("rack",200000))  
			    		JustinaNavigation::getClose("rack",200000);  
                JustinaHRI::waitAfterSay("I arrived to rack", 4000);
                rackVisited = true;
				nextState = SM_FIND_OBJECTS_ON_RACK;
                break;
			}


			case SM_FIND_OBJECTS_ON_RACK:   //FIXME:check objects or check food/?
			{
                bool grab = false;
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FIND_OBJECTS_ON_RACK" << std::endl;
                justinaSay.str( std::string() );
                if (menu_selected == 1)
                {
                    if (obj_on_table.find (MENU_1_drink) != obj_on_table.end() && obj_on_table.find (MENU_1_food) != obj_on_table.end() )
                        justinaSay << "All the food is in the table, I need to go to the cupboard";
                    else if (obj_on_table.find (MENU_1_food) != obj_on_table.end() )
                        justinaSay << "I am going to search only for " << MENU_1_drink << ", because " << MENU_1_food << " is already on the table.";
                    else if (obj_on_table.find (MENU_1_drink) != obj_on_table.end() )
                        justinaSay << "I am going to search only for " << MENU_1_food << ", because " << MENU_1_drink << " is already on the table.";
                    else
                        justinaSay << "I am going to search for " << MENU_1_drink << " and " << MENU_1_food << " on the rack.";
                }
                else
                { 
                    if (obj_on_table.find (MENU_2_drink) != obj_on_table.end() && obj_on_table.find (MENU_2_food)!= obj_on_table.end() )
                        justinaSay << "All the food is in the table, I need to go to the cupboard";
                    else if (obj_on_table.find (MENU_2_food)!= obj_on_table.end() )
                        justinaSay << "I am going to search only for " << MENU_2_drink << ", because " << MENU_2_food << " is already on the table.";
                    else if (obj_on_table.find (MENU_2_drink)!= obj_on_table.end() )
                        justinaSay << "I am going to search only for " << MENU_2_food << ", because " << MENU_2_drink << " is already on the table.";
                    else
                        justinaSay << "I am going to search for " << MENU_2_drink << " and " << MENU_2_food << " on the rack.";
                }
                JustinaHRI::waitAfterSay(justinaSay.str(), DELAY_SPEAK);

				if(!JustinaTasks::alignWithTable(0.35))
				{
					JustinaNavigation::moveDist(0.10, 3000);
					if(!JustinaTasks::alignWithTable(0.35))
					{
						std::cout << "I can´t alignWithTable... :'(" << std::endl;
						JustinaNavigation::moveDist(-0.15, 3000);
                        JustinaHRI::waitAfterSay("I can not align myself with the rack. I will navigate to de cupboard", 4000);
                        nextState = SM_NAVIGATION_TO_CUPBOARD;
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
                        if (attempt == 3) 
                        {    
                            //nextState = SM_FINISH_TEST;
                            nextState = SM_NAVIGATION_TO_CUPBOARD;
                            JustinaHRI::waitAfterSay("I could not find objects on the rack", 4000);
                        }
					}
                    else
					{
						std::cout << "I have found " << recoObjForTake.size() << " objects on the rack" << std::endl;
						justinaSay.str( std::string() );
						justinaSay << "I have found " << recoObjForTake.size() << " objects on the rack";
						JustinaHRI::say(justinaSay.str());

						for(int i = 0; i < recoObjForTake.size(); i++)
						{
							std::cout << recoObjForTake[i].id << "   ";
							std::cout << recoObjForTake[i].pose << std::endl;

							if(recoObjForTake[i].id.find("unknown") != std::string::npos)
								idObjectGrasp.push_back("");
							else
                                //don put the object if it is already in the table
								idObjectGrasp.push_back(recoObjForTake[i].id);
						}
                        grab = true;            //a posibility is to grab only if the object is recognized
                        //JustinaHRI::waitAfterSay("Imagine that I have grab this object", 4000);
                        //JustinaHRI::waitAfterSay("I will come back to the table with this object", 4000);
					}

				}
                if (!grab)
                {
                    nextState = SM_NAVIGATION_TO_TABLE;
                    JustinaHRI::waitAfterSay("I could not find objects, so I am going find objects on the cupboard", 4000);
                }
                else
                {
                    nextState = SM_TAKE_OBJECT_RIGHT;
                }
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
*/


			case SM_TAKE_OBJECT_RIGHT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: TAKE_OBJECT_RIGHT" << std::endl;
                JustinaHRI::waitAfterSay("I am going to take object with my right arm", 4000);
				if (maxAttempsGraspRight < 3)
				{
					if(!JustinaTasks::alignWithTable(0.35))
                    {
						std::cout << "I can´t align with table   :´(" << std::endl;
                        JustinaNavigation::moveDistAngle(-0.05, M_PI_4/4, 2000);
                        JustinaTasks::alignWithTable(0.35);
                        JustinaTasks::alignWithTable(0.35);
                        JustinaTasks::alignWithTable(0.35);
                    }
					else
					{
						if(idObjectGrasp[1] != "")
						{
								if(JustinaTasks::findObject(idObjectGrasp[1], poseObj_1, leftArm) )
                                {

									if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, false, idObjectGrasp[1]) )
									{
                                        //FIXME: check if I need a variable to info that i have grasp an object
										if(recoObjForTake.size() > 1)
										{
											maxAttempsGraspRight = 0;
											nextState = SM_NAVIGATION_TO_TABLE;
											//nextState = SM_TAKE_OBJECT_LEFT;
										}
										else
										{
											maxAttempsGraspRight = 0;
											nextState = SM_NAVIGATION_TO_TABLE;
										}
									}
									else
									{
										std::cout << "I can´t grasp objects in " << maxAttempsGraspRight << " attempt" << std::endl;
									}
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
											nextState = SM_NAVIGATION_TO_TABLE;
											//nextState = SM_TAKE_OBJECT_LEFT;
										}
										else
										{
											maxAttempsGraspRight = 0;
											nextState = SM_NAVIGATION_TO_TABLE;
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
                    nextState = SM_NAVIGATION_TO_TABLE;
					//nextState = SM_TAKE_OBJECT_LEFT;
				}
                /*
                if(JustinaManip::objOnRightHand())
                    nextState = SM_PUT_OBJECT_ON_TABLE_RIGHT;
                else
                    nextState = SM_FIND_OBJECTS_ON_TABLE;
                recoObjForTake.clear();
                //idObjectGrasp.clear();
                maxAttempsGraspLeft = 0;
                nextState = SM_GOTO_CUPBOARD;
*/
                break;
			}

/*
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

*/

			case SM_NAVIGATION_TO_CUPBOARD:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: GOTO_CUPBOARD" << std::endl;
				JustinaHRI::say("I am going to navigate to the cupboard to find the cup");
				if(!JustinaNavigation::getClose("cupboard",200000))
			    	if(!JustinaNavigation::getClose("cupboard",200000))
			    		JustinaNavigation::getClose("cupboard",200000);
				JustinaHRI::say("I arrived to the cupboard");
                cupboardVisited = true;
                nextState = SM_FIND_OBJECTS_ON_CUPBOARD;
                break;
			}


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
					if(!JustinaTasks::alignWithTable(0.40))
                    {
                        JustinaHRI::waitAfterSay("I can not align myself with the cupboard. I will return to the table", 4000);
                        nextState = SM_NAVIGATION_TO_TABLE;
                    }    
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

				//JustinaTools::pdfImageExport("StoringGroseriesTest","/home/$USER/objs/");
				findObjCupboard = true;
				nextState = SM_NAVIGATION_TO_TABLE;
			}
			break;


/*

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
