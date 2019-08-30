//Bibliotecas por default

#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaRepresentation.h"
//Estados para la máquina

enum SMState {
	SM_INIT,
	SM_INIT_TABLE,
	SM_WAIT_FOR_COMMAND,
	SM_REPEAT_COMMAND,
	SM_PARSE_SPOKEN_COMMAND,
    SM_WAIT_COMMAND,
    SM_RESET_OBJ,
    SM_GET_ORDER,
    SM_WHERE_IS,
	SM_NAVIGATE_TO_INSPECTION,
	SM_ALIGN_TABLE,
	SM_DETECT_OBJECT,
	SM_HANDLER,
	SM_GRASP_OBJECT,
	SM_DELIVER_OBJECT,
	SM_REPEAT_TASK,
	SM_FINAL_STATE
};

SMState state = SM_INIT_TABLE;
/*
#define SM_INIT 0
//#define SM_WAIT_FOR_DOOR 10
#define SM_NAVIGATE_TO_INSPECTION 20
#define SM_WAITING_FOR_KITCHEN 30
#define SM_WAIT_FOR_COMMAND 40 
#define SM_REPEAT_COMMAND 50 
#define SM_PARSE_SPOKEN_COMMAND 60
#define SM_FINAL_STATE 70 
#define SM_FINAL_STATE_2 55
#define SM_WAIT_FOR_CONFIRMATION 80 
#define SM_PARSE_SPOKEN_CONFIRMATION 90
#define SM_WAIT_FOR_INSPECTION 25 
//#define SM_ROBOT_STOP 35 
#define SM_MOVE_HEAD 45 
*/

//Params
int numberAttemps = 0;
int cantidad = 0;
int women;
int men;
int sitting;
int standing;
int lying;
ros::Time beginPlan;
bool fplan = false;
double maxTime = 180;
//std::string cat_grammar= "eegpsr_montreal.xml";
std::string microsoft_grammars[3];
std::string sphinx_grammars[3];
bool alternative_drink = true;
bool poket_reco = false;
std::string no_drink;
std::string prev_drink = "no_prev";
JustinaTasks::POSE poseRecog;
std::vector<Eigen::Vector3d> centroids;
float robot_x, robot_y, robot_a;
float gx_w, gy_w, gz_w;
std::stringstream ss_loc; 
float goalx, goaly, goala;
float theta = 0, thetaToGoal = 0, angleHead = 0;
float dist_to_head;
std::vector<std::string> centroids_loc;
float torsoSpine, torsoWaist, torsoShoulders;


//Main
int main(int argc, char** argv){
	std::cout << "INITIALIZING ACT_PLN BY MARCOSOFT..." << std::endl; //cout
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    //Setting nodeHandle
    JustinaHRI::setNodeHandle(&n);
    JustinaHardware::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaRepresentation::setNodeHandle(&n);
    ros::Rate loop(10);

    int nextState = 0;
    bool fail = false;
    bool success = false;

    //Valid voice commands
    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    validCommands.push_back("continue");
    validCommands.push_back("robot yes");
    validCommands.push_back("robot no");
    validCommands.push_back("robot stop");
    validCommands.push_back("justina start");
    validCommands.push_back("justina yes");
    validCommands.push_back("justina no");

    JustinaHRI::setInputDevice(JustinaHRI::USB);
    JustinaHRI::setOutputDevice(JustinaHRI::USB);
    JustinaHRI::setVolumenInputDevice(JustinaHRI::USB, 65000);
    JustinaHRI::setVolumenOutputDevice(JustinaHRI::USB, 50000);
    JustinaManip::startTorsoGoTo(0.1, 0, 0);

    //Params to getOrder
    JustinaRepresentation::initKDB("/cia_2019/cia.dat", false, 20000);



    std::vector<std::string> tokens;
    std::vector<std::string> tokens1;
    std::stringstream ss;
    int cont = 0;
    int contdrink = 0;
    int attemps = 0;
    int confirm = 0;
    bool la = false;
    bool ra = false;
    bool drop = false;
    bool objectDetected = false;
    std::vector<vision_msgs::VisionObject> recoObj;
    sensor_msgs::Image image;
    int index;
    std::string lastReco;
	recoObj = std::vector<vision_msgs::VisionObject>();
    
    std::string drink;
    std::string query;
	std::string str;

    while(ros::ok() && !fail && !success){
        switch(state){

        	case SM_INIT:
        		//Init case
        		std::cout << "State machine: SM_INIT" << std::endl;	
        		JustinaManip::startHdGoTo(0.0, 0.0);
        		JustinaHRI::waitAfterSay("I am ready for the robot test", 4000);
            	JustinaHRI::waitAfterSay("Please, tell me justina start to go to the kitchen table",4000);
                
                JustinaHRI::enableSpeechRecognized(false);
                JustinaHRI::loadGrammarSpeechRecognized("restaurant_commands.xml");
            	JustinaHRI::waitAfterSay("Please, tell me justina start to go to the kitchen table",4000);
                JustinaHRI::enableSpeechRecognized(true);
        		state = SM_WAIT_FOR_COMMAND;
        		break;

    		case SM_INIT_TABLE:
        		//Init case
        		std::cout << "State machine: SM_INIT_TABLE" << std::endl;	
        		JustinaManip::startHdGoTo(0.0, 0.0);
        		JustinaHRI::waitAfterSay("Hello my name is Justina", 4000);
            	JustinaHRI::waitAfterSay("I am ready for recieve a command",4000);
        		//state = SM_ALIGN_TABLE;
        		state = SM_WAIT_COMMAND;
        		break;

    		case SM_WAIT_FOR_COMMAND:
    			//Waiting for the command
    			std::cout << "State machine: SM_WAIT_FOR_COMMAND" << std::endl;	
    			if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 12000))
                {
                    state = SM_WAIT_FOR_COMMAND;
                }
                else
                {
                    std::cout << "Parsing word..." << std::endl;
                    state = SM_PARSE_SPOKEN_COMMAND;
                }
    			break;

			case SM_REPEAT_COMMAND:
				//Repeat the command
				std::cout << "State machine: SM_REPEAT_COMMAND" << std::endl;
                JustinaHRI::waitAfterSay("Please repeat the command", 4000);
                	state = SM_WAIT_FOR_COMMAND;
                break;

            case SM_PARSE_SPOKEN_COMMAND:
            	//Parse the command
            	std::cout << "State machine: SM_PARSE_SPOKEN_COMMAND" << std::endl;
            	if (confirm == 0){
		            if(lastRecoSpeech.find("justina start") != std::string::npos)
		            {
		                state = SM_NAVIGATE_TO_INSPECTION;
		            }
		            else
		            {
		            	JustinaHRI::waitAfterSay("I can't recognize this command", 4000);
		                state = SM_REPEAT_COMMAND;
		            }
		        }
		        else if (confirm == 1) {
		        	if(lastRecoSpeech.find("justina yes") != std::string::npos || lastRecoSpeech.find("robot yes") != std::string::npos)
		        	{
		            	JustinaHRI::waitAfterSay("Ok, I am ready for recieve a command", 4000);
		            	//JustinaNavigation::moveDistAngle(0, 3.141592, 5000);
		                //state = SM_ALIGN_TABLE;
                        state = (contdrink == 3) ? SM_RESET_OBJ : SM_WAIT_COMMAND;
		                cont = 0;
		            }
		            else
		            {
		            	cont++;
                        state = (cont >= 3) ? SM_FINAL_STATE : SM_REPEAT_COMMAND;

		            }

		            

		        }
                break;
            case SM_RESET_OBJ:
        		std::cout << "State machine: SM_RESET_OBJ" << std::endl;	
                contdrink = 0;
                ss.str("");
                ss << "(assert (reset_objs 1))";
                JustinaRepresentation::sendAndRunCLIPS(ss.str());

                JustinaHRI::waitAfterSay("There are no more objects on the table", 2000);
                JustinaHRI::waitAfterSay("please put all the objects I give you on top of the table", 4000);
                JustinaHRI::waitAfterSay("thank you", 2000);
                state = SM_WAIT_COMMAND;
                break;
            case SM_WAIT_COMMAND:
        		std::cout << "State machine: SM_WAIT_COMMAND" << std::endl;	
                JustinaHRI::enableSpeechRecognized(false);
                JustinaHRI::loadGrammarSpeechRecognized("cia.xml");
                JustinaHRI::enableSpeechRecognized(true);
                JustinaHRI::waitForSpeechRecognized(lastReco,400);
                if(JustinaHRI::waitForSpeechRecognized(lastReco,10000)){
                    if(JustinaRepresentation::stringInterpretation(lastReco, drink))
                        std::cout << "last int: " << drink << std::endl;
                    str = drink;
	                split(tokens, str, boost::is_any_of(" "));

                    if(tokens.size()>1)
                        state = SM_GET_ORDER;
                    else
                        state = SM_WHERE_IS;
                }
                break;
            case SM_WHERE_IS:
        		    std::cout << "State machine: SM_WHERE_IS" << std::endl;	
                    ss.str("");
                    ss << "(assert (get_obj_default_loc " << drink << " 1))";    
                    JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);
                    if (query != "table"){
                        ss.str("");
                        ss << "you have the " << drink;
                        JustinaHRI::waitAfterSay(ss.str(), 2000);
                        alternative_drink = false;
                    }
                    else{
                        ss.str("");
                        ss << "The " << drink << " is on the table";
                        JustinaHRI::waitAfterSay(ss.str(), 2000);
                    }
                    state = SM_WAIT_COMMAND;
        		    JustinaHRI::waitAfterSay("do you want something else",4000);
                break;    
            case SM_GET_ORDER:
        		    std::cout << "State machine: SM_GET_ORDER" << std::endl;	
                    ss.str("");
                    ss << "Do you want " << tokens[1] << ", say justina yes or justina no";
                    drink = tokens[1];
                    
                    JustinaHRI::enableSpeechRecognized(false);
                    JustinaHRI::loadGrammarSpeechRecognized("commands.xml");
        		    JustinaHRI::waitAfterSay(ss.str(),4000);
                    JustinaHRI::enableSpeechRecognized(true);
                    JustinaHRI::waitForSpeechRecognized(lastReco,400);

                    JustinaHRI::waitForSpeechRecognized(lastReco,10000);
                    if(lastReco == "robot yes" || lastReco == "justina yes"){
                            ss.str("");
                            ss << "(assert (get_obj_default_loc " << drink << " 1))";    
                            JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);
                            if (query != "table"){
                                ss.str("");
                                ss << "I am sorry, the " << drink << " is out of stock , please choose another one";
                                JustinaHRI::waitAfterSay(ss.str(), 2000);
                                alternative_drink = false;
                                state = SM_WAIT_COMMAND;
                            }
                            else{
                                ss.str("");
                                ss << "Ok i will give you the " << drink;
                                JustinaHRI::waitAfterSay(ss.str(), 2000);
		            	        JustinaNavigation::moveDistAngle(0, 3.141592, 5000);
                                state = SM_ALIGN_TABLE;
                            }
                    }
                    else{
                        JustinaHRI::waitAfterSay("Do you want something else, repeat the command please", 4000);
                        state = SM_WAIT_COMMAND;
                    }
                    //count++;

                break;

        	case SM_NAVIGATE_TO_INSPECTION:
        		//Go to kitchen table
        		std::cout << "State machine: SM_NAVIGATE_TO_INSPECTION" << std::endl;
        		JustinaHRI::waitAfterSay("I am going to the kitchen table",4000);
        		sleep(2);
        		if(!JustinaNavigation::getClose("kitchen_table", 120000)){
        			std::cout << "Cannot move to kitchen_table" << std::endl;
        		}
        		JustinaHRI::say("I have arrived to the kitchen table");
        		state = SM_ALIGN_TABLE;
        		break;	

    		case SM_ALIGN_TABLE:
                JustinaHRI::enableSpeechRecognized(false);
    			std::cout << "State machine: SM_ALIGN_TABLE" << std::endl;
                ss.str("");
                ss << "Barman I need a " << drink  << " please, put the " << drink << "in front of me, on the table"; 
                JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
    			JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 6000);
        		objectDetected = JustinaTasks::alignWithTable(0.35);
        		//objectDetected = true;
        		state = SM_DETECT_OBJECT;
    			break;

			case SM_DETECT_OBJECT:
    			std::cout << "State machine: SM_DETECT_OBJECT" << std::endl;
    			if(objectDetected){
                    ss.str("");
                    ss << "I am looking for the " << drink << " on the table";
		            JustinaHRI::waitAfterSay(ss.str(), 5000);
		            //Obtiene la lista de objetos a detectar
		            //recoObj = std::vector<vision_msgs::VisionObject>();

		            objectDetected = false;
		            //Detecta los objetos en la mesa
		            if(JustinaVision::detectAllObjectsVot(recoObj, image, 5)){
		                for(int j = 0; j < recoObj.size() && !objectDetected; j++){
		                	// id.compare es la lista de objetos a leer, en este caso es cocacola
		                    if (recoObj[j].id.compare(drink) == 0){
		                        index = j;
		                        objectDetected = true;
		                    }
		                }
		            } 
		        }   
	        	state = (objectDetected) ? SM_GRASP_OBJECT : SM_HANDLER;
	        	//state = SM_HANDLER;
    			break;


			case SM_GRASP_OBJECT:
				std::cout << "State machine: SM_GRASP_OBJECT" << std::endl;
                if(objectDetected && recoObj.size() > 0){
                    ss.str("");
                    ss << "I have found the " << drink;
                    JustinaHRI::waitAfterSay(ss.str(), 5000);
                    //JustinaTasks::alignWithTable(0.35);
                    ss.str("");
                    ss << "I am going to take the " << drink;
                    JustinaHRI::waitAfterSay(ss.str(), 5000);
                    // This is for grasp with two frames //false for right true for left, "", true torso 
                    //std::cout << "Index: " << index << std::endl;
                    //std::cout << "recoObj: " << recoObj.size() << std::endl;

                    if(recoObj[index].pose.position.y > 0)
                        ra = false;
                    else
                        ra = true;

                    if (ra){
                    	JustinaTasks::graspObject(recoObj[index].pose.position.x, recoObj[index].pose.position.y, recoObj[index].pose.position.z, false, "", true);
                    	drop = true;
                	}
                	else{
						JustinaTasks::graspObject(recoObj[index].pose.position.x, recoObj[index].pose.position.y, recoObj[index].pose.position.z, true, "", true);
						drop = false;                		
                	}

                }
               
				state = SM_DELIVER_OBJECT;		        
				break;

			case SM_HANDLER:
                JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 5000);
                JustinaNavigation::startMoveDist(-0.15);
				std::cout << "State machine: SM_HANDLER" << std::endl;
                ss.str("");
                ss << "Sorry i could not grasp the " << drink << ", please put the " << drink << " in my gripper";
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                if(drop){
                	JustinaManip::raGoTo("navigation", 3000);
                	JustinaTasks::detectObjectInGripper(drink, false, 7000);
                }
                else{
                	JustinaManip::laGoTo("navigation", 3000);
                	JustinaTasks::detectObjectInGripper(drink, true, 7000);
                }
                state = SM_DELIVER_OBJECT;
				break;

			case SM_DELIVER_OBJECT:
				std::cout << "State machine: SM_DELIVER_OBJECT" << std::endl;
				JustinaNavigation::moveDistAngle(0, 3.141592, 5000);
				JustinaHRI::waitAfterSay("Guest, i try find you", 4000);
				JustinaManip::hdGoTo(0.9, 0.0, 6000);
				JustinaManip::hdGoTo(0.6, 0.0, 6000);
				JustinaManip::hdGoTo(0.3, 0.0, 6000);
				JustinaManip::hdGoTo(0.0, 0.0, 6000);
				JustinaManip::hdGoTo(-0.3, 0.0, 6000);
				JustinaManip::hdGoTo(-0.6, 0.0, 6000);
				JustinaManip::hdGoTo(-0.9, 0.0, 6000);
				JustinaManip::hdGoTo(-0.6, 0.0, 6000);
				JustinaManip::hdGoTo(-0.3, 0.0, 6000);
				JustinaManip::hdGoTo(0.0, 0.0, 6000);
				JustinaHRI::waitAfterSay("Guest, i find you", 6000);
                ss.str("");
                ss << "Please take the " << drink << " from my gripper"; 
				JustinaHRI::waitAfterSay(ss.str(), 5000);
                if(drop){
                	JustinaManip::raGoTo("take", 3000);
                	JustinaTasks::dropObject("", false, 10000);
            	}
            	else{
            		JustinaManip::laGoTo("take", 3000);
            		JustinaTasks::dropObject("", true, 10000);
            	}
                ss.str("");
                JustinaManip::hdGoTo(0.0, 0.0, 6000);
                ss << "Enjoy the " << drink;
            	JustinaHRI::waitAfterSay(ss.str(), 5000);
                ss.str("");
                ss << "(assert (set_obj_default_loc " << drink <<" no_on_table 1))";
                JustinaRepresentation::sendAndRunCLIPS(ss.str());
                contdrink++;
                state = SM_REPEAT_TASK;
				break;

			case SM_REPEAT_TASK:
        		//Final state
        		std::cout << "State machine: SM_REPEAT_TASK" << std::endl;	
        		confirm = 1;
        		JustinaHRI::waitAfterSay("Do you want something else",4000);
                JustinaHRI::enableSpeechRecognized(false);
                JustinaHRI::loadGrammarSpeechRecognized("commands.xml");
        		JustinaHRI::waitAfterSay("Please, tell me justina yes o justina no",4000);
                JustinaHRI::enableSpeechRecognized(true);
    			state = SM_WAIT_FOR_COMMAND;
                break;


        	
        	case SM_FINAL_STATE:
        		//Final state
        		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	
        		sleep(3);
                JustinaHRI::say("I have finished test");
        		success = true;
        		fail = true;
                break;


        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;

}
