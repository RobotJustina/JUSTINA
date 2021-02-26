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
    SM_SELECT_OBJECT_GRASP,
	SM_NAVIGATE_TO_INSPECTION,
	SM_ALIGN_TABLE,
	SM_DETECT_OBJECT,
    SM_NO_DETECT_OBJECT,
	SM_HANDLER,
	SM_GRASP_OBJECT,
	SM_DELIVER_OBJECT,
	SM_REPEAT_TASK,
	SM_FINAL_STATE
};

SMState state = SM_INIT;
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
    std::vector<std::string> validCommandsTake;
    validCommandsTake.push_back("bring me a coke");
    validCommandsTake.push_back("bring me an apple");
    validCommandsTake.push_back("bring me a pringles");

    JustinaHRI::setInputDevice(JustinaHRI::USB);
    JustinaHRI::setOutputDevice(JustinaHRI::USB);
    JustinaHRI::setVolumenInputDevice(JustinaHRI::USB, 65000);
    JustinaHRI::setVolumenOutputDevice(JustinaHRI::USB, 50000);
    JustinaManip::startTorsoGoTo(0.1, 0, 0);

    //Params to getOrder
    //JustinaRepresentation::initKDB("/cia_2019/cia.dat", false, 20000);



    std::vector<std::string> tokens;
    std::vector<std::string> tokens1;
    std::stringstream ss;
    int cont = 0;
    int change_loc=0;
    int d_location=0;
    int countOp = 0;
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
    std::string location;

    while(ros::ok() && !fail && !success){
        switch(state){
        	case SM_INIT:
        		//Init case
        		std::cout << "State machine: SM_INIT" << std::endl;	
        		JustinaManip::startHdGoTo(0.0, 0.0);
        		JustinaHRI::waitAfterSay("I am ready for the gpsr test", 4000);
                JustinaHRI::enableSpeechRecognized(false);
                JustinaHRI::loadGrammarSpeechRecognized("restaurant_commands.xml");
                JustinaHRI::enableSpeechRecognized(true);
        		state = SM_SELECT_OBJECT_GRASP;
        		break;
		
		    case SM_WAIT_FOR_COMMAND:
    			//Waiting for the command
    			std::cout << "State machine: SM_WAIT_FOR_COMMAND" << std::endl;	
                ss.str("");
                ss << "Do you want " << drink << ", say justina yes or justina no";
                JustinaHRI::waitAfterSay(ss.str(), 5000);
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

		    case SM_PARSE_SPOKEN_COMMAND:
            	//Parse the command
            	std::cout << "State machine: SM_PARSE_SPOKEN_COMMAND" << std::endl;
            	if(confirm == 0){
		            if(lastRecoSpeech.find("justina yes") != std::string::npos){
		                state = SM_NAVIGATE_TO_INSPECTION;
		            }
		            else if(lastRecoSpeech.find("justina no") != std::string::npos){
		              JustinaHRI::waitAfterSay("Repeat the command, please", 4000);
		              state = SM_SELECT_OBJECT_GRASP;
		            }
                }    
                else if(confirm == 1){
                    if(lastRecoSpeech.find("justina yes") != std::string::npos){
                        state = SM_NAVIGATE_TO_INSPECTION;
                    }
                    else{
                      std::cout << "Finally test" << std::endl;
                      state = SM_FINAL_STATE;
                    }    
                }    
                break;

		
		    case SM_SELECT_OBJECT_GRASP:
        		//Select object to grasp
        		std::cout << "State machine: SM_SELECT_OBJECT_GRASP" << std::endl;
                JustinaHRI::waitAfterSay("I am ready for recieve a command",4000);
                std::cout << "I am ready for recieve a command" << std::endl; 
                std::cout << "The sentences is bring me a/bring me an" << std::endl;
			    std::cout << "coke" << std::endl;
			    std::cout << "apple" << std::endl;
			    std::cout << "pringles" << std::endl;
			    if(JustinaHRI::waitForSpecificSentence(validCommandsTake, lastRecoSpeech, 7000)){
                    		//attemptsRecogLoc++;
 
    				if(lastRecoSpeech.find("bring me a coke") != std::string::npos){
    					drink = "coke";
                        state = SM_WAIT_FOR_COMMAND;	
    				}
    				else if(lastRecoSpeech.find("bring me an apple") != std::string::npos){
                        drink = "apple";
                        state = SM_WAIT_FOR_COMMAND;		
                    }
                    else if(lastRecoSpeech.find("bring me a pringles") != std::string::npos){
                        drink = "pringles";
                        state = SM_WAIT_FOR_COMMAND;                    		
                    }
			    }

        	    break;	

                
        	case SM_NAVIGATE_TO_INSPECTION:
        		//Go to location
        		std::cout << "State machine: SM_NAVIGATE_TO_INSPECTION" << std::endl;
			    if(change_loc==0){
                    ss.str("");
                    ss << "I am going to the kitchen table";
                    JustinaHRI::waitAfterSay(ss.str(), 5000);
        			if(!JustinaNavigation::getClose("kitchen_table", 120000)){
        				std::cout << "Cannot move to kitchen_table" << std::endl;
        			}
                    ss.str("");
                    ss << "I have arrived to the kitchen table";
                    JustinaHRI::waitAfterSay(ss.str(), 5000);
        			state = SM_ALIGN_TABLE;
			    }
			    if(change_loc==1){
                    ss.str("");
                    ss << "I am going to the bed";
                    JustinaHRI::waitAfterSay(ss.str(), 5000);
        			if(!JustinaNavigation::getClose("bed", 120000)){
        				std::cout << "Cannot move to bed" << std::endl;
        			}
                    ss.str("");
                    ss << "I have arrived to the bed";
                    JustinaHRI::waitAfterSay(ss.str(), 5000);
        			state = SM_DELIVER_OBJECT;
			    }
        	    break;	


    		case SM_ALIGN_TABLE:
    			std::cout << "State machine: SM_ALIGN_TABLE" << std::endl;
                JustinaHRI::waitAfterSay("I aling with the kitchen table", 4000);
                std::cout << "I aling with the kitchen table" << std::endl;
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
	        	state = (objectDetected) ? SM_GRASP_OBJECT : SM_NO_DETECT_OBJECT;
    		    break;

            case SM_NO_DETECT_OBJECT:
                std::cout << "State machine: SM_DETECT_OBJECT" << std::endl;
                if(!JustinaNavigation::getClose("bed", 120000)){
                    std::cout << "Cannot move to bed" << std::endl;
                }
                ss.str("");
                ss << "I have arrived to the bed";
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                JustinaTasks::findPerson("", -1, JustinaTasks::NONE, false, "bedroom");
                ss.str("");
                ss << "Sorry, i cannot find the " << drink << std::endl;
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                state = SM_FINAL_STATE;
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

                    if(ra){
                    	JustinaTasks::graspObject(recoObj[index].pose.position.x, recoObj[index].pose.position.y, recoObj[index].pose.position.z, false, "", true);
                    	drop = true;
                	}
                	else{
						JustinaTasks::graspObject(recoObj[index].pose.position.x, recoObj[index].pose.position.y, recoObj[index].pose.position.z, true, "", true);
						drop = false;                		
                	}

                }
               	change_loc=1;
				state = SM_NAVIGATE_TO_INSPECTION;		        
			    break;

		

			case SM_DELIVER_OBJECT:
				std::cout << "State machine: SM_DELIVER_OBJECT" << std::endl;
				//JustinaNavigation::moveDistAngle(0, 3.141592, 5000);
                //std::cout << "Guest, i try to find you" << std::endl;
				
				//face recognigtion
                JustinaTasks::findPerson("", -1, JustinaTasks::NONE, false, "bedroom");
                ss.str("");
                ss << "Guest, i find you";
                JustinaHRI::waitAfterSay(ss.str(), 5000); 
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
     
                JustinaManip::hdGoTo(0.0, 0.0, 6000);
                contdrink++;
                state = SM_REPEAT_TASK;
		        break;

            case SM_REPEAT_TASK:
                std::cout << "State machine: SM_REPEAT_TASK" << std::endl;
                ss.str("");
                ss << "Do you want something else, say justina yes or justina no";
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                confirm = 1;
                state = SM_PARSE_SPOKEN_COMMAND;
                break;

        	
        	case SM_FINAL_STATE:
        		//Final state
        		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	
                JustinaHRI::waitAfterSay("I have finished test",4000);
			    std::cout << "I have finished test" << std::endl;	
        		success = true;
        		fail = true;
                break;

            
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
