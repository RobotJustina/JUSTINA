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
#define CLUTERY_1         "beer"
#define CLUTERY_2         "soap"
#define CLUTERY_3         "beer"
#define CLUTERY_4         "soap"
#define DELAY_SPEAK       7000
#define DELAY_AFTER_SPEAK 1000
#define YES_CMD           "robot yes"
#define NO_CMD            "robot no"
#define MENU1_CMD         "menu one"
#define MENU2_CMD         "menu two"
#define CLEAN_CMD         "please clean"

enum task  
{   
    SM_DUMMY,
    SM_INIT, 
    SM_WAIT_FOR_DOOR,
    SM_WAIT_FOR_COMMAND,
    SM_WAIT_FOR_START_COMMAND, 
    SM_WAIT_FOR_CHOOSE_COMMAND,
    SM_WAIT_FOR_CLEAN_COMMAND,    
    SM_OFFER_MENUS,
    SM_INIT_COMMAND,
    SM_NAVIGATION_TO_RACK,  
    SM_NAVIGATION_TO_TABLE, 
    SM_NAVIGATION_TO_CUPBOARD,  
    SM_FIND_OBJECTS_ON_TABLE, 
    SM_FIND_OBJECTS_ON_RACK, 
    SM_FIND_OBJECTS_ON_CUPBOARD, 
    SM_FIND_SPONGE_ON_CUPBOARD,
    SM_TAKE_OBJECT_RIGHT, 
    SM_TAKE_OBJECT_LEFT, 
    SM_TAKE_OBJECT_RIGHT_CUPBOARD, 
    SM_TAKE_OBJECT_LEFT_CUPBOARD,
    SM_TAKE_OBJECT_RIGHT_TABLE,
    SM_PUT_OBJECT_ON_TABLE_RIGHT, 
    SM_PUT_OBJECT_ON_TABLE_LEFT, 
    SM_CLEAN_TABLE_SAY,
    SM_CLEAN_TABLE,
    SM_GIVE_SPACE_TO_USER,
    SM_FINISH_TEST,
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
	task nextState               = SM_WAIT_FOR_CLEAN_COMMAND;//SM_INIT_COMMAND;
	//task nextState               = SM_NAVIGATION_TO_TABLE;   //SM_INIT_COMMAND;
	task lastState               = SM_DUMMY;   //SM_INIT_COMMAND;

    std::map<std::string, bool> obj_localiz;
    obj_localiz.insert( std::pair<std::string, bool>(MENU_1_drink, false));
    obj_localiz.insert( std::pair<std::string, bool>(MENU_1_food , false));
    obj_localiz.insert( std::pair<std::string, bool>(MENU_2_drink, false));
    obj_localiz.insert( std::pair<std::string, bool>(MENU_2_food , false));
    obj_localiz.insert( std::pair<std::string, bool>(CLUTERY_1   , false));
    obj_localiz.insert( std::pair<std::string, bool>(CLUTERY_2   , false));

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
    bool cleanTable         = false;
    int  menu_selected      = 1;
    bool beter_arm;
    
//////////// BORRARRRRRRRRRRR   !!!!!!!!!!!!!!!!!!!!!!!!11  /////////////////////
	nextState               = SM_FIND_OBJECTS_ON_TABLE;
    rackVisited  = true;
    cupboardVisited = true;
    cleanTable = true;

	std::vector<vision_msgs::VisionObject> recoObjForTake;
	std::vector<vision_msgs::VisionObject> recoObjList;
	std::vector<vision_msgs::VisionObject> recoObjForGrasp;
	std::vector<vision_msgs::VisionObject> objForTakeRight;
	std::vector<vision_msgs::VisionObject> objForTakeLeft;
	std::vector<std::string> idObjectGrasp;

	std::stringstream justinaSay;

	geometry_msgs::Pose poseObj_1;
	geometry_msgs::Pose poseObj_2;

	std::string lastRecoSpeech;
	std::vector<std::string> validCommands;
	validCommands.push_back(YES_CMD);
	validCommands.push_back(NO_CMD);
	validCommands.push_back(MENU1_CMD);
	validCommands.push_back(MENU2_CMD);
	validCommands.push_back(CLEAN_CMD);

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
                if (JustinaManip::objOnRightHand())
                {              
                    nextState = SM_PUT_OBJECT_ON_TABLE_RIGHT;
                }
                else if (JustinaManip::objOnLeftHand())
                {
                    nextState = SM_PUT_OBJECT_ON_TABLE_LEFT;
                }
                else if (cleanTable)
                {
                    nextState = SM_FIND_OBJECTS_ON_TABLE;
                }
                else
                {
                    nextState = SM_INIT_COMMAND;
                }
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
                    JustinaManip::startLaGoTo("navigation");
                    JustinaManip::startRaGoTo("navigation");
                }
                else if (rackVisited && !cupboardVisited)
                {
                    nextState = SM_NAVIGATION_TO_CUPBOARD;
                }
                else if (rackVisited && cupboardVisited)
                {
                    JustinaHRI::waitAfterSay("Human, bon appetite. Let me know when you finish.", DELAY_SPEAK);
                    JustinaHRI::waitAfterSay("Say please clean for ask me to clean the table, i will be waiting. Excuse me.", DELAY_SPEAK);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(DELAY_AFTER_SPEAK));
                    lastRecoSpeech.clear();
                    nextState = SM_GIVE_SPACE_TO_USER;
                    
                    //FIXME only for test
                    /*
                    std::cout << "" << std::endl;
                    std::cout << "" << std::endl;
                    std::cout << "----->  State machine: going to nav" << std::endl;
                    JustinaManip::startLaGoTo("navigation");
                    //JustinaManip::startRaGoTo("navigation");
                    //JustinaTasks::dropObjectInBox("", true);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
                    std::cout << "" << std::endl;
                    std::cout << "" << std::endl;
                    std::cout << "----->  State machine: going to box" << std::endl;
                    JustinaManip::startLaGoTo("box");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
                    JustinaManip::startLaGoTo("navigation");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
                    std::cout << "" << std::endl;
                    std::cout << "" << std::endl;
                    std::cout << "----->  State machine: going to nav" << std::endl;
                    JustinaManip::startRaGoTo("navigation");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
                    std::cout << "" << std::endl;
                    std::cout << "" << std::endl;
                    std::cout << "----->  State machine: going to box" << std::endl;
                    JustinaManip::startRaGoTo("box");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
                    JustinaManip::startRaGoTo("navigation");
                    //JustinaManip::startRaGoTo("box");
                    nextState = SM_FINISH_TEST;
                    */
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
				  if(lastRecoSpeech.find(YES_CMD) != std::string::npos)
                  {
				    nextState = SM_OFFER_MENUS;
				  }
                    else if(lastRecoSpeech.find(NO_CMD) != std::string::npos)
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

			case SM_GIVE_SPACE_TO_USER:
			{
				if(!JustinaNavigation::getClose("space",200000))
			    	if(!JustinaNavigation::getClose("space",200000))
			    		JustinaNavigation::getClose("space",200000);
                nextState = SM_WAIT_FOR_CLEAN_COMMAND;
                break;
			}

			case SM_WAIT_FOR_CLEAN_COMMAND:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: WAIT_FOR_CLEAN_COMMAND" << std::endl;
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))   //what are this parameters?
                {
                    //JustinaHRI::waitAfterSay("Please repeat the command", DELAY_SPEAK);
                    //boost::this_thread::sleep(boost::posix_time::milliseconds(DELAY_AFTER_SPEAK));
                }
                else
				{
				  if(lastRecoSpeech.find(CLEAN_CMD) != std::string::npos)
                  {
				    nextState = SM_CLEAN_TABLE_SAY;
				  }
                }
                break;
			}
            
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
				  if(lastRecoSpeech.find(MENU1_CMD) != std::string::npos)
                  {
                    menu_selected = 1;
                    justinaSay.str( std::string() );
                    justinaSay << "You have asked for " << MENU_1_drink << " and " << MENU_1_food << ", I am going to set up your order.";
                    JustinaHRI::waitAfterSay(justinaSay.str(), DELAY_SPEAK);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(DELAY_AFTER_SPEAK));
				    nextState = SM_FIND_OBJECTS_ON_TABLE;                      
				  }
                  else if(lastRecoSpeech.find(MENU2_CMD) != std::string::npos)
                  {
                    menu_selected = 2;
                    justinaSay.str( std::string() );
                    justinaSay << "You have asked for " << MENU_2_drink << " and " << MENU_2_food << ", I am going to set up your order.";
                    JustinaHRI::waitAfterSay(justinaSay.str(), DELAY_SPEAK);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(DELAY_AFTER_SPEAK));
				    nextState = SM_FIND_OBJECTS_ON_TABLE;                      
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
                if (!cleanTable) JustinaHRI::waitAfterSay("I am going to check which objects are already on the table", DELAY_SPEAK);
                else             JustinaHRI::waitAfterSay("I am going to take find from table to clean it", DELAY_SPEAK);

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
                int cant_obj_detected = 0;
                bool obj_detected     = false;
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
                        cant_obj_detected = recoObjForTake.size(); 
                        std::cout << "I have found " << cant_obj_detected << " objects on the table" << std::endl;
                        justinaSay.str( std::string() );
                        justinaSay << "I have found " << cant_obj_detected << " objects on the table";
                        JustinaHRI::waitAfterSay(justinaSay.str(), DELAY_SPEAK);
                        for(int i = 0; i < cant_obj_detected; i++)
                        {
                              obj_on_table.insert (recoObjForTake[i].id);
                        }
                        if (cant_obj_detected > 0) obj_detected = true;
                        break;
                    }
                }
                if (obj_detected)
                {

                    std::cout << "" << std::endl;
                    std::cout << "" << std::endl;
                    std::cout << "----->  State machine: SAVE_OBJECTS_PDF" << std::endl;
                    //JustinaTools::pdfImageExport("SetUpTableTest","/home/$USER/objs/");
                }
                if (!cleanTable){
                    nextState = SM_NAVIGATION_TO_RACK;
                }else{
                    //clasificar los objetos a tomar en comida y no comida
                    if (obj_detected)    
                        nextState = SM_TAKE_OBJECT_RIGHT_TABLE;
                    else
                        nextState = SM_NAVIGATION_TO_CUPBOARD;
                }
                break;
            }


			case SM_TAKE_OBJECT_RIGHT_TABLE:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: TAKE_OBJECT_RIGHT_TABLE" << std::endl;
				if (maxAttempsGraspRight < 3)
				{
                    JustinaHRI::waitAfterSay("I am going to take object with my right arm", DELAY_SPEAK);
                    leftArm = false;
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
						if(recoObjForTake[0].id != "")
						{
								if(JustinaTasks::findObject(recoObjForTake[0].id, poseObj_1, beter_arm) )
                                {
									if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, leftArm, recoObjForTake[0].id, true) )
									{
                                        maxAttempsGraspRight = 3;
                                        JustinaTasks::dropObjectInBox("");
                                        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
                                        JustinaManip::startRaGoTo("navigation");
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
                                if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, leftArm, recoObjForTake[0].id, true) )
                                {
                                    maxAttempsGraspRight = 3;
                                    JustinaTasks::dropObjectInBox("");
                                    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
                                    JustinaManip::startRaGoTo("navigation");
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
                    lastState = SM_TAKE_OBJECT_RIGHT_TABLE; 
					maxAttempsGraspRight = 0;
                    nextState = SM_FIND_OBJECTS_ON_TABLE;
                }
                break;
			}
			case SM_NAVIGATION_TO_RACK:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: NAVIGATION_TO_RACK" << std::endl;
                JustinaHRI::waitAfterSay("I am going to navigate to the rack and bring the missing food", DELAY_SPEAK);
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
                        JustinaHRI::waitAfterSay("I can not align myself with the rack. I will navigate to the cupboard", DELAY_SPEAK);
                        nextState = SM_NAVIGATION_TO_CUPBOARD;
						break;
					}
				}


				//idObjectGrasp.clear();
				recoObjForTake.clear();
                recoObjForGrasp.clear();
                objForTakeRight.clear();
                objForTakeLeft.clear();
				for(int attempt = 0; attempt < 4; attempt++)
				{
					if(!JustinaVision::detectAllObjects(recoObjForTake, true))
                    {
						std::cout << "I  can't detect anything" << std::endl;
                        if (attempt == 3 && recoObjForTake.size() == 0) 
                        {    
                            nextState = SM_NAVIGATION_TO_CUPBOARD;
                            JustinaHRI::waitAfterSay("I could not find objects on the rack", 4000);
                        }
					}
                    else
					{
						std::cout << "I have found " << recoObjForTake.size() << " objects on the rack" << std::endl;
						for(int i = 0; i < recoObjForTake.size(); i++)
						{
							std::cout << recoObjForTake[i].id << "   ";
							std::cout << recoObjForTake[i].pose << std::endl;

							if(recoObjForTake[i].id.find("unknown") != std::string::npos)
                            {   
                                //for now i am not adding unkow objects, buy may be i should
								//idObjectGrasp.push_back("");
                            }
							else
                            {
                                if ( (menu_selected == 1 && (recoObjForTake[i].id == MENU_1_food || recoObjForTake[i].id == MENU_1_drink)) ||
                                     (menu_selected == 2 && (recoObjForTake[i].id == MENU_2_food || recoObjForTake[i].id == MENU_2_drink))  ) 
                                {
                                    //idObjectGrasp.push_back(recoObjForTake[i].id);
                                    int cant_obj_grasp = recoObjForGrasp.size();
                                    bool notAdded = true;
                                    for (int j = 0; j < cant_obj_grasp; j++)
                                    {
                                        if(recoObjForGrasp[j].id.find(recoObjForTake[i].id) != std::string::npos) notAdded = false;   
                                    }
                                    if (notAdded) recoObjForGrasp.push_back(recoObjForTake[i]);
                                    grab = true;            //a posibility is to grab only if the object is recognized
                                }
                            }
						}
					}

				}
                if (!grab)
                {
                    JustinaHRI::waitAfterSay("I could not find food on the rack, so I am going take clutery from the cupboard", DELAY_SPEAK);
                    nextState = SM_NAVIGATION_TO_CUPBOARD;
                }
                else
                {
                    justinaSay.str( std::string() );
                    justinaSay << "I have found " << recoObjForGrasp.size() << " of the objects that i need";
                    JustinaHRI::say(justinaSay.str());
                    if (recoObjForGrasp.size() > 1)
                    {
                        if (recoObjForGrasp[0].pose.position.y > recoObjForGrasp[1].pose.position.y)
                        {
                            objForTakeLeft.push_back ( recoObjForGrasp[0] );
                            objForTakeRight.push_back( recoObjForGrasp[1] );
                        }
                        else
                        {
                            objForTakeLeft.push_back ( recoObjForGrasp[1] );
                            objForTakeRight.push_back( recoObjForGrasp[0] );
                        }
                    }
                    else
                    {
                        objForTakeRight.push_back ( recoObjForGrasp[0] );
                    }
                    nextState = SM_TAKE_OBJECT_RIGHT;
                    lastState = SM_FIND_OBJECTS_ON_RACK;
                    std::cout << "" << std::endl;
                    std::cout << "" << std::endl;
                    std::cout << "----->  State machine: SAVE_OBJECTS_PDF" << std::endl;
                    //JustinaTools::pdfImageExport("SetUpTableTest","/home/$USER/objs/");
                }
                break;
			}

			case SM_TAKE_OBJECT_RIGHT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: TAKE_OBJECT_RIGHT" << std::endl;
				if (maxAttempsGraspRight < 3)
				{
                    JustinaHRI::waitAfterSay("I am going to take object with my right arm", DELAY_SPEAK);
                    leftArm = false;
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
						if(objForTakeRight[0].id != "")
						{
								if(JustinaTasks::findObject(objForTakeRight[0].id, poseObj_1, beter_arm) )
                                {
									if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, leftArm, objForTakeRight[0].id, true) )
									{
                                        maxAttempsGraspRight = 3;
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
                                if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, leftArm, objForTakeRight[0].id, true) )
                                {
                                    maxAttempsGraspRight = 3;
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
                    lastState = SM_TAKE_OBJECT_RIGHT; 
					maxAttempsGraspRight = 0;
                    if (objForTakeLeft.size() == 0)
                    { 
                        nextState = SM_NAVIGATION_TO_TABLE;
                    }
                    else
                    {
                        nextState = SM_TAKE_OBJECT_LEFT;
                        maxAttempsGraspLeft = 0;
                    }
                }
                break;
			}

			case SM_TAKE_OBJECT_LEFT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: TAKE_OBJECT_LEFT" << std::endl;
				if (maxAttempsGraspLeft < 3)
				{
                    JustinaHRI::waitAfterSay("I am going to take object with my left arm", DELAY_SPEAK);
                    leftArm = true;
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
						if(objForTakeLeft[0].id != "")
						{
								if(JustinaTasks::findObject(objForTakeLeft[0].id, poseObj_1, beter_arm) )
                                {
									if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, leftArm, objForTakeLeft[0].id, true) )
									{
                                        maxAttempsGraspLeft = 3;
									}
									else
									{
										std::cout << "I can´t grasp objects in " << maxAttempsGraspLeft << " attempt" << std::endl;
									}
                                }
						}
						else
						{
                                if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, leftArm, objForTakeLeft[0].id, true) )
                                {
                                    maxAttempsGraspLeft = 3;
                                }
                                else
                                {
                                    std::cout << "I can´t grasp objects in " << maxAttempsGraspLeft << " attempts" << std::endl;
                                }
						}

					}
					maxAttempsGraspLeft++;
				}
				else
				{
                    lastState = SM_TAKE_OBJECT_LEFT; 
					maxAttempsGraspLeft = 0;
                    if (JustinaManip::objOnRightHand() || JustinaManip::objOnLeftHand())
                    { 
                        nextState = SM_NAVIGATION_TO_TABLE;
                    }
                    else
                    {
                        nextState = SM_NAVIGATION_TO_CUPBOARD;
                        JustinaHRI::waitAfterSay("I am going to navigate to the cupboard because I could not grasp any object", DELAY_SPEAK);
                    }
                    recoObjForTake.clear();
                    recoObjForGrasp.clear();
                    objForTakeRight.clear();
                    objForTakeLeft.clear();
                }
                break;
			}

			case SM_PUT_OBJECT_ON_TABLE_RIGHT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: PUT_OBJECT_ON_TABLE_RIGHT" << std::endl;
				if(maxAttempsPlaceObj < 4)
				{
                    JustinaHRI::say("I will placed the object in my right arm on the table");
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
						maxAttempsPlaceObj = 4;
					}
					maxAttempsPlaceObj++;
				}
				else
				{
					maxAttempsPlaceObj = 0;
                    if (JustinaManip::objOnRightHand())
                    {
                        std::cout << "I can´t placed objects on table with my right arm" << std::endl;
                        JustinaHRI::say("I can´t found a free place on the table");
                    }
                    if (JustinaManip::objOnLeftHand())
                    {   
                        nextState = SM_PUT_OBJECT_ON_TABLE_LEFT;
                    }else if (lastState == SM_TAKE_OBJECT_RIGHT_CUPBOARD || lastState == SM_TAKE_OBJECT_LEFT_CUPBOARD)
                        nextState = SM_INIT_COMMAND;
                    else
                        nextState = SM_NAVIGATION_TO_CUPBOARD;
				}
                break;
			}

			case SM_PUT_OBJECT_ON_TABLE_LEFT:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: PUT_OBJECT_ON_TABLE_LEFT" << std::endl;
				if(maxAttempsPlaceObj < 4)
				{
                    JustinaHRI::say("I will placed the object in my left arm on the table");
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
					if(JustinaTasks::placeObject(true)) 
					{
						maxAttempsPlaceObj = 4;
					}
					maxAttempsPlaceObj++;
				}
				else
				{
					maxAttempsPlaceObj = 0;
                    if (JustinaManip::objOnLeftHand())
                    {
                        std::cout << "I can´t placed objects on table with my left arm" << std::endl;
                        JustinaHRI::say("I can´t found a free place on the table");
                    }
                    if (lastState == SM_TAKE_OBJECT_RIGHT_CUPBOARD || lastState == SM_TAKE_OBJECT_LEFT_CUPBOARD)
                        nextState = SM_INIT_COMMAND;
                    else
                        nextState = SM_NAVIGATION_TO_CUPBOARD;
				}
                break;
			}

			case SM_NAVIGATION_TO_CUPBOARD:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: GOTO_CUPBOARD" << std::endl;
				JustinaHRI::say("I am going to navigate to the cupboard to to take the clutery");
				if(!JustinaNavigation::getClose("cupboard",200000))
			    	if(!JustinaNavigation::getClose("cupboard",200000))
			    		JustinaNavigation::getClose("cupboard",200000);
				JustinaHRI::say("I arrived to the cupboard");
                cupboardVisited = true;
                if (lastState == SM_CLEAN_TABLE || cleanTable)
                {
                    nextState = SM_FIND_SPONGE_ON_CUPBOARD;
                }
                else
                {
                    nextState = SM_FIND_OBJECTS_ON_CUPBOARD;
                }    
                break;
			}

			case SM_FIND_OBJECTS_ON_CUPBOARD:   
			{
                bool grab = false;
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FIND_OBJECTS_ON_CUPBOARD" << std::endl;
                justinaSay.str( std::string() );
                if (menu_selected == 1)
                {
                    if (obj_on_table.find (CLUTERY_1) != obj_on_table.end() && obj_on_table.find (CLUTERY_2) != obj_on_table.end() )
                        justinaSay << "All the clutery is in the table, I need to go back to the table";
                    else if (obj_on_table.find (CLUTERY_1) != obj_on_table.end() )
                        justinaSay << "I am going to search only for " << CLUTERY_2 << ", because " << CLUTERY_1 << " is already on the table.";
                    else if (obj_on_table.find (CLUTERY_2) != obj_on_table.end() )
                        justinaSay << "I am going to search only for " << CLUTERY_1 << ", because " << CLUTERY_2 << " is already on the table.";
                    else
                        justinaSay << "I am going to search for " << CLUTERY_1 << " and " << CLUTERY_2 << " on the cupboard.";
                }
                else
                { 
                    if (obj_on_table.find (CLUTERY_3) != obj_on_table.end() && obj_on_table.find (CLUTERY_4) != obj_on_table.end() )
                        justinaSay << "All the clutery is in the table, I need to go back to the table";
                    else if (obj_on_table.find (CLUTERY_3) != obj_on_table.end() )
                        justinaSay << "I am going to search only for " << CLUTERY_4 << ", because " << CLUTERY_3 << " is already on the table.";
                    else if (obj_on_table.find (CLUTERY_4) != obj_on_table.end() )
                        justinaSay << "I am going to search only for " << CLUTERY_3 << ", because " << CLUTERY_4 << " is already on the table.";
                    else
                        justinaSay << "I am going to search for " << CLUTERY_3 << " and " << CLUTERY_4 << " on the cupboard.";
                }
                JustinaHRI::waitAfterSay(justinaSay.str(), DELAY_SPEAK);
				if(!JustinaTasks::alignWithTable(0.35))
				{
					JustinaNavigation::moveDist(0.10, 3000);
					if(!JustinaTasks::alignWithTable(0.35))
					{
						std::cout << "I can´t alignWithTable... :'(" << std::endl;
						JustinaNavigation::moveDist(-0.15, 3000);
                        JustinaHRI::waitAfterSay("I can not align myself with the cupboard. I will navigate to de table", DELAY_SPEAK);
                        nextState = SM_NAVIGATION_TO_TABLE;
                        lastState = SM_FIND_OBJECTS_ON_CUPBOARD;
						break;
					}
				}

                recoObjForTake.clear();
                recoObjForGrasp.clear();
                objForTakeRight.clear();
                objForTakeLeft.clear();
                maxAttempsGraspRight = 0;
                maxAttempsGraspLeft = 0;
                for(int attempt = 0; attempt < 4; attempt++)
                {
                    if(!JustinaVision::detectAllObjects(recoObjForTake, true))
                    {
                        std::cout << "I  can't detect anything" << std::endl;
                        if (attempt == 3 && recoObjForTake.size() == 0) 
                        {    
                            nextState = SM_NAVIGATION_TO_TABLE;
                            JustinaHRI::waitAfterSay("I could not find objects on the cupboard", DELAY_SPEAK);
                        }
                    }
                    else
                    {
                        std::cout << "I have found " << recoObjForTake.size() << " objects on the cupboard" << std::endl;
                        for(int i = 0; i < recoObjForTake.size(); i++)
                        {
                            std::cout << recoObjForTake[i].id << "   ";
                            std::cout << recoObjForTake[i].pose << std::endl;

                            if(recoObjForTake[i].id.find("unknown") != std::string::npos)
                            {   
                                //for now i am not adding unkow objects, buy may be i should
                                //idObjectGrasp.push_back("");
                            }
                            else
                            {
                                if ( (menu_selected == 1 && (recoObjForTake[i].id == CLUTERY_1 || recoObjForTake[i].id == CLUTERY_2)) ||
                                     (menu_selected == 2 && (recoObjForTake[i].id == CLUTERY_3 || recoObjForTake[i].id == CLUTERY_4))  ) 
                                {
                                    //idObjectGrasp.push_back(recoObjForTake[i].id);
                                    int cant_obj_grasp = recoObjForGrasp.size();
                                    bool notAdded = true;
                                    for (int j = 0; j < cant_obj_grasp; j++)
                                    {
                                        if(recoObjForGrasp[j].id.find(recoObjForTake[i].id) != std::string::npos) notAdded = false;   
                                    }
                                    if (notAdded) recoObjForGrasp.push_back(recoObjForTake[i]);
                                    grab = true;            //a posibility is to grab only if the object is recognized
                                }
                            }
                        }
                    }

                }
                if (!grab)
                {
                    JustinaHRI::waitAfterSay("I could not find food on the cupboard, so I am going return to the table", DELAY_SPEAK);
                    nextState = SM_NAVIGATION_TO_TABLE;
                }
                else
                {
                    justinaSay.str( std::string() );
                    justinaSay << "I have found " << recoObjForGrasp.size() << " objects on the cupboard";
                    JustinaHRI::say(justinaSay.str());
                    if (recoObjForGrasp.size() > 1)
                    {
                        if (recoObjForGrasp[0].pose.position.y > recoObjForGrasp[1].pose.position.y)
                        {
                            objForTakeLeft.push_back ( recoObjForGrasp[0] );
                            objForTakeRight.push_back( recoObjForGrasp[1] );
                        }
                        else
                        {
                            objForTakeLeft.push_back ( recoObjForGrasp[1] );
                            objForTakeRight.push_back( recoObjForGrasp[0] );
                        }
                    }
                    else
                    {
                        objForTakeRight.push_back ( recoObjForGrasp[0] );
                    }
                    nextState = SM_TAKE_OBJECT_RIGHT_CUPBOARD;
                    std::cout << "" << std::endl;
                    std::cout << "----->  State machine: SAVE_OBJECTS_PDF" << std::endl;
                    //JustinaTools::pdfImageExport("SetUpTableTest","/home/$USER/objs/");
                }
                break;				
			}

			case SM_TAKE_OBJECT_RIGHT_CUPBOARD:
			{
                std::cout << "" << std::endl;
                std::cout << "" << std::endl;
                std::cout << "----->  State machine: SM_TAKE_OBJECT_RIGHT_CUPBOARD" << std::endl;
                if (maxAttempsGraspRight < 3)
                {
                    JustinaHRI::waitAfterSay("I am going to take object with my right arm", 4000);
                    leftArm = false;
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
                        if(objForTakeRight[0].id != "")
                        {
                                if(JustinaTasks::findObject(objForTakeRight[0].id, poseObj_1, beter_arm) )
                                {
                                    if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, leftArm, objForTakeRight[0].id, true) )
                                    {
                                                maxAttempsGraspRight = 3;
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
                                if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, leftArm, objForTakeRight[0].id, true) )
                                {
                                            maxAttempsGraspRight = 3;
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
                    lastState = SM_TAKE_OBJECT_RIGHT_CUPBOARD; 
                    maxAttempsGraspRight = 0;
                    if (objForTakeLeft.size() == 0)
                    { 
                        nextState = SM_NAVIGATION_TO_TABLE;
                    }
                    else
                    {
                        nextState = SM_TAKE_OBJECT_LEFT_CUPBOARD;
                    }
                }
                break;
			}

            case SM_TAKE_OBJECT_LEFT_CUPBOARD:
            {
                std::cout << "" << std::endl;
                std::cout << "" << std::endl;
                std::cout << "----->  State machine: TAKE_OBJECT_LEFT" << std::endl;
                if (maxAttempsGraspLeft < 3)
                {
                    JustinaHRI::waitAfterSay("I am going to take object with my left arm", DELAY_SPEAK);
                    leftArm = true;
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
                        if(objForTakeLeft[0].id != "")
                        {
                                if(JustinaTasks::findObject(objForTakeLeft[0].id, poseObj_1, beter_arm) )
                                {
                                    if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, leftArm, objForTakeLeft[0].id, true) )
                                    {
                                                maxAttempsGraspLeft = 3;
                                    }
                                    else
                                    {
                                        std::cout << "I can´t grasp objects in " << maxAttempsGraspLeft << " attempt" << std::endl;
                                    }
                                }
                        }
                        else
                        {
                                if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z + 0.04, leftArm, objForTakeLeft[0].id, true) )
                                {
                                            maxAttempsGraspLeft = 3;
                                }
                                else
                                {
                                    std::cout << "I can´t grasp objects in " << maxAttempsGraspLeft << " attempts" << std::endl;
                                }
                        }

                    }
                    maxAttempsGraspLeft++;
                }
                else
                {
                    lastState = SM_TAKE_OBJECT_LEFT_CUPBOARD; 
                    maxAttempsGraspLeft = 0;
                    JustinaHRI::waitAfterSay("I am going to navigate to the table", DELAY_SPEAK);
                    nextState = SM_NAVIGATION_TO_TABLE;
                    recoObjForTake.clear();
                    recoObjForGrasp.clear();
                    objForTakeRight.clear();
                    objForTakeLeft.clear();
                    
                }
                break;
            }

            case SM_CLEAN_TABLE_SAY:
            {
                std::cout << "" << std::endl;
                std::cout << "" << std::endl;
                std::cout << "----->  State machine: CLEAN TABLE" << std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
                JustinaHRI::say("I am going to clean the table now.");
                cleanTable = true;
                lastState = SM_CLEAN_TABLE;
                nextState = SM_NAVIGATION_TO_TABLE;
                //align to table
                //find objects on table
                break;
            }


			case SM_FIND_SPONGE_ON_CUPBOARD:   
			{
                bool grab = false;
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FIND_SPONGE_ON_CUPBOARD" << std::endl;
                justinaSay.str( std::string() );
                justinaSay << "I am going to search for the sponge.";
                JustinaHRI::waitAfterSay(justinaSay.str(), DELAY_SPEAK);

				if(!JustinaTasks::alignWithTable(0.35))
				{
					JustinaNavigation::moveDist(0.10, 3000);
					if(!JustinaTasks::alignWithTable(0.35))
					{
						std::cout << "I can´t alignWithTable... :'(" << std::endl;
						JustinaNavigation::moveDist(-0.15, 3000);
                        JustinaHRI::waitAfterSay("I can not align myself with the cupboard. I will finish the test", 4000);
                        nextState = SM_FINISH_TEST;
                        lastState = SM_FIND_OBJECTS_ON_CUPBOARD;
						break;
					}
				}


				idObjectGrasp.clear();
				recoObjForTake.clear();
                objForTakeRight.clear();
				for(int attempt = 0; attempt < 4; attempt++)
				{
					if(!JustinaVision::detectAllObjects(recoObjForTake, true))
                    {
						std::cout << "I  can't detect anything" << std::endl;
                        if (attempt == 3 && recoObjForTake.size() == 0) 
                        {    
                            //nextState = SM_FINISH_TEST;
                            nextState = SM_NAVIGATION_TO_TABLE;
                            JustinaHRI::waitAfterSay("I could not find objects on the cupboard", 4000);
                        }
					}
                    else
					{
						std::cout << "I have found " << recoObjForTake.size() << " objects on the cupboard" << std::endl;

						for(int i = 0; i < recoObjForTake.size(); i++)
						{
							std::cout << recoObjForTake[i].id << "   ";
							std::cout << recoObjForTake[i].pose << std::endl;

							if(recoObjForTake[i].id.find("sponge") != std::string::npos)
								objForTakeRight.push_back(recoObjForTake[i]);
						}
                        grab = true;            //a posibility is to grab only if the object is recognized
                        //what happend if see only first time and then cant see anything? maybe grab should be false again
                        //JustinaHRI::waitAfterSay("Imagine that I have grab this object", 4000);
                        //JustinaHRI::waitAfterSay("I will come back to the table with this object", 4000);
					}

				}
                if (!grab)
                {
                    JustinaHRI::waitAfterSay("I could not find the sponge on the cupboard, so I am going to come back to the table.", 4000);
                    nextState = SM_FINISH_TEST;
                }
                else
                {
                    justinaSay.str( std::string() );
                    justinaSay << "I have found " << recoObjForTake.size() << " objects on the cupboard";
                    JustinaHRI::say(justinaSay.str());
                    lastState = SM_FIND_SPONGE_ON_CUPBOARD;
                    nextState = SM_TAKE_OBJECT_RIGHT_CUPBOARD;
                    maxAttempsGraspRight = 0;
                }
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: SAVE_OBJECTS_PDF" << std::endl;
				//JustinaTools::pdfImageExport("SetUpTableTest","/home/$USER/objs/");
                break;
			}
            
            case SM_CLEAN_TABLE:
            {
                std::cout << "" << std::endl;
                std::cout << "" << std::endl;
                std::cout << "----->  State machine: CLEAN TABLE" << std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
                nextState  = SM_NAVIGATION_TO_TABLE;
                lastState  = SM_CLEAN_TABLE;
                cleanTable = true;
                //align to table
                //find objects on table
                break;
            }

			case SM_FINISH_TEST:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FINISH_TEST" << std::endl;
				boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
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
