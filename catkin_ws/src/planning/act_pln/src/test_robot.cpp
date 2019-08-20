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
	SM_WAIT_FOR_COMMAND,
	SM_REPEAT_COMMAND,
	SM_PARSE_SPOKEN_COMMAND,
	SM_NAVIGATE_TO_INSPECTION,
	SM_ALIGN_TABLE,
	SM_DETECT_OBJECT,
	SM_HANDLER,
	SM_GRASP_OBJECT,
	SM_DELIVER_OBJECT,
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
    //validCommands.push_back("justina no");

    JustinaHRI::setInputDevice(JustinaHRI::USB);
    JustinaHRI::setOutputDevice(JustinaHRI::USB);
    JustinaHRI::setVolumenInputDevice(JustinaHRI::USB, 65000);
    JustinaHRI::setVolumenOutputDevice(JustinaHRI::USB, 50000);
    JustinaManip::startTorsoGoTo(0.1, 0, 0);

    //Params to getOrder



    std::vector<std::string> tokens;
    std::vector<std::string> tokens1;
    std::stringstream ss;
    int attemps = 0;
    bool la = false;
    bool ra = false;
    bool objectDetected = false;
    std::vector<vision_msgs::VisionObject> recoObj;
    sensor_msgs::Image image;
    int index;
    std::string lastReco;
	recoObj = std::vector<vision_msgs::VisionObject>();

    while(ros::ok() && !fail && !success){
        switch(state){

        	case SM_INIT:
        		//Init case
        		std::cout << "State machine: SM_INIT" << std::endl;	
        		JustinaManip::startHdGoTo(0.0, 0.0);
        		JustinaHRI::waitAfterSay("I am ready for the robot test", 4000);
            	JustinaHRI::waitAfterSay("Please, tell me justina start to go to the kitchen table",4000);
        		state = SM_WAIT_FOR_COMMAND;
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
            	std::cout << "State machine: SM_REPEAT_COMMAND" << std::endl;
                if(lastRecoSpeech.find("justina start") != std::string::npos)
                {
                    state = SM_NAVIGATE_TO_INSPECTION;
                }
                else
                {
                	JustinaHRI::waitAfterSay("I can't recognize this command", 4000);
                    state = SM_REPEAT_COMMAND;
                }
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
    			std::cout << "State machine: SM_ALIGN_TABLE" << std::endl;
    			JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 6000);
        		objectDetected = JustinaTasks::alignWithTable(0.35);
        		objectDetected = true;
        		state = SM_DETECT_OBJECT;
    			break;

			case SM_DETECT_OBJECT:
    			std::cout << "State machine: SM_DETECT_OBJECT" << std::endl;
    			if(objectDetected){
		            JustinaHRI::waitAfterSay("I am looking for the object on the kitchen table", 5000);
		            //Obtiene la lista de objetos a detectar
		            //recoObj = std::vector<vision_msgs::VisionObject>();

		            objectDetected = false;
		            //Detecta los objetos en la mesa
		            if(JustinaVision::detectAllObjectsVot(recoObj, image, 5)){
		                for(int j = 0; j < recoObj.size() && !objectDetected; j++){
		                	// id.compare es la lista de objetos a leer, en este caso es cocacola
		                    if (recoObj[j].id.compare("coke") == 0){
		                        index = j;
		                        objectDetected = true;
		                    }
		                }
		            } 
		        }
                std::cout << "recoObj: " << recoObj.size() << std::endl;
                std::cout << "Index: " << index << std::endl;
                std::cout << "ObjectDetected: " << objectDetected << std::endl;   
	        	state = (objectDetected) ? SM_GRASP_OBJECT : SM_HANDLER;
	        	//state = SM_HANDLER;
    			break;


			case SM_GRASP_OBJECT:
				std::cout << "State machine: SM_GRASP_OBJECT" << std::endl;
                if(objectDetected && recoObj.size() > 0){
                    JustinaHRI::waitAfterSay("I have found the object", 5000);
                    //JustinaTasks::alignWithTable(0.35);
                    JustinaHRI::waitAfterSay("I am going to take the object", 5000);
                    // This is for grasp with two frames //false for right true for left, "", true torso 
                    std::cout << "Index: " << index << std::endl;
                    std::cout << "recoObj: " << recoObj.size() << std::endl;

                    JustinaTasks::graspObject(recoObj[index].pose.position.x, recoObj[index].pose.position.y, recoObj[index].pose.position.z, false, "", true);


                }
               
		            /*if(!objectDetected){
			            JustinaNavigation::startMoveDist(-0.15);
			            JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 5000);
			            JustinaNavigation::waitForGoalReached(2000);
			            JustinaHRI::waitAfterSay("Sorry i could not grasp the object by myself", 5000);
			            JustinaHRI::waitAfterSay("Please put the object in my gripper", 5000);
			            if(!ra){
			                JustinaManip::raGoTo("navigation", 3000);
			                JustinaTasks::detectObjectInGripper(tokens[i], false, 7000);
			                ra = true;
			                JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 6000);
			                objectDetected = JustinaTasks::alignWithTable(0.35); 
			                if(objectDetected){
			                    objectDetected = false;
			                    if(JustinaVision::detectAllObjectsVot(recoObj, image, 5)){
			                        for(int j = 0; j < recoObj.size() && !objectDetected; j++){
			                            if (recoObj[j].id.compare(tokens[i]) == 0){
			                                index =j;
			                                objectDetected = true;
			                             }
			                         }

			                         if(objectDetected){
			                             ss.str("");
			                             ss << "I have found the " << tokens[i] << " on the table, please give me the correct object" << std::endl; 
			                             
			                             JustinaNavigation::startMoveDist(-0.15);
			                             JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 5000);
			                             JustinaNavigation::waitForGoalReached(2000);
			                             
			                             JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
			                             JustinaTasks::dropObject("", false, 10000);

			                             ss.str("");
			                             ss << "give me the " << tokens[i] << std::endl; 
			                             JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
			                             JustinaTasks::detectObjectInGripper(tokens[i], false, 7000);

			                             ra = true;
			                             ss.str("");
			                            ss << "(assert (set_object_arm " << tokens[i] << " false))";
			                            JustinaRepresentation::sendAndRunCLIPS(ss.str());
			                            JustinaHRI::waitAfterSay("thank you barman", 5000, 0);
			                        }
			                    }
			                }
			                if(!objectDetected){
			                    ra = true;
			                    ss.str("");
			                    ss << "(assert (set_object_arm " << tokens[i] << " false))";
			                    JustinaRepresentation::sendAndRunCLIPS(ss.str());
			                    JustinaHRI::waitAfterSay("thank you barman", 5000, 0);
			                }   

			                
			            }
			            if(!la && tokens.size() > 3 && i >1){
			                JustinaManip::laGoTo("navigation", 3000);
			                JustinaTasks::detectObjectInGripper(tokens[i], true, 7000);
			                la = true;
			                JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 6000);
			                objectDetected = JustinaTasks::alignWithTable(0.35);
			                if(objectDetected){
			                    objectDetected = false;
			                     if(JustinaVision::detectAllObjectsVot(recoObj, image, 5)){
			                        for(int j = 0; j < recoObj.size() && !objectDetected; j++){
			                            if (recoObj[j].id.compare(tokens[i]) == 0){
			                                index =j;
			                                objectDetected = true;
			                            }
			                        }

			                        if(objectDetected){
			                            ss.str("");
			                            ss << "I have found the " << tokens[i] << " on the table, please give me the correct object" << std::endl; 
			                             
			                            JustinaNavigation::startMoveDist(-0.15);
			                            JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 5000);
			                            JustinaNavigation::waitForGoalReached(2000);
			                            
			                            JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
			                            JustinaTasks::dropObject("", true, 10000);
			                            
			                            ss.str("");
			                            ss << "give me the " << tokens[i] << std::endl; 
			                            JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
			                            JustinaTasks::detectObjectInGripper(tokens[i], true, 7000);
			                            
			                            la = true;
			                            ss.str("");
			                            ss << "(assert (set_object_arm " << tokens[i] << " true))";
			                            JustinaRepresentation::sendAndRunCLIPS(ss.str());
			                            JustinaHRI::waitAfterSay("thank you barman", 5000, 0);
			                        }
			                    }
			                }
			                if(!objectDetected){
			                    la = true;
			                    ss.str("");
			                    ss << "(assert (set_object_arm " << tokens[i] << " true))";
			                    JustinaRepresentation::sendAndRunCLIPS(ss.str());
			                    JustinaHRI::waitAfterSay("thank you barman", 5000, 0);
			                }   
			            }
			        }*/
				state = SM_DELIVER_OBJECT;		        
				break;

			case SM_HANDLER:
				std::cout << "State machine: SM_HANDLER" << std::endl;
				JustinaHRI::waitAfterSay("Sorry i could not grasp the object", 5000);
                JustinaHRI::waitAfterSay("Please put the object in my gripper", 5000);
                JustinaManip::raGoTo("navigation", 3000);
                JustinaTasks::detectObjectInGripper("coke", false, 7000);
                state = SM_DELIVER_OBJECT;
				break;

			case SM_DELIVER_OBJECT:
				std::cout << "State machine: SM_DELIVER_OBJECT" << std::endl;
				JustinaNavigation::moveDistAngle(0, 3.141592, 5000);
				JustinaHRI::waitAfterSay("Human, please take the object from my gripper", 5000);
                JustinaManip::raGoTo("take", 3000);
                JustinaTasks::dropObject("", false, 10000);
                state = SM_FINAL_STATE;
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
