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
    SM_WAIT_COMMAND_WORD,
    SM_REPEAT_COMMAND_WORD,

	SM_NAVIGATE_TO_INSPECTION,
	SM_ALIGN_TABLE,
	SM_DETECT_OBJECT,
	SM_HANDLER,
	SM_GRASP_OBJECT,
	SM_DELIVER_OBJECT,
	SM_REPEAT_TASK,
	SM_FINAL_STATE
};

SMState state = SM_NAVIGATE_TO_INSPECTION;
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
    std::string location;

    while(ros::ok() && !fail && !success){
        switch(state){
                
        	case SM_NAVIGATE_TO_INSPECTION:
        		//Go to location
        		std::cout << "State machine: SM_NAVIGATE_TO_INSPECTION" << std::endl;
                std::cout << "I am going to the location table" << std::endl;
        		if(!JustinaNavigation::getClose("kitchen_table", 120000)){
        			std::cout << "Cannot move to location table" << std::endl;
        		}
                std::cout << "I have arrived to the table location" << std::endl;
        		state = SM_ALIGN_TABLE;
        		break;	

    		case SM_ALIGN_TABLE:
    			std::cout << "State machine: SM_ALIGN_TABLE" << std::endl;
                std::cout << "I aling with the table location" << std::endl;
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
	        	state = (objectDetected) ? SM_GRASP_OBJECT : SM_HANDLER;
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
                std::cout << "Guest, i try find you" << std::endl;
				
				//face recognigtion

                std::cout <<"Guest, i find you" << std::endl;
                ss.str("");
                ss << "Please take the " << drink << " from my gripper"; 
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
                ss.str("");
                
                contdrink++;
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
