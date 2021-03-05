//Bibliotecas por default

#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaNavigation.h"
//Estados para la máquina

enum SMState {
    SM_INIT,
    SM_MOV_DIST,
    SM_MOV_ANGLE,
    SM_MOV_LAT,
	SM_NAV_LOC
};

SMState state = SM_INIT;



//Main
int main(int argc, char** argv){
	std::cout << "INITIALIZING Navigation Test..." << std::endl; //cout
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    //Setting nodeHandle
    JustinaNavigation::setNodeHandle(&n);
    ros::Rate loop(10);
    bool success = false;


    while(ros::ok() && !success){
        switch(state){
        	case SM_INIT:
        		//Init case
        		std::cout << "State machine: SM_INIT" << std::endl;	
                state = SM_MOV_DIST;
        		break;
            case SM_MOV_DIST:
                std::cout << "State machine: SM_MOV_DIST" << std::endl;
                JustinaNavigation::moveDist(1.0, 5000);
                state = SM_MOV_ANGLE;
                break;
            case SM_MOV_ANGLE:
                std::cout << "State machine: SM_MOV_ANGLE" << std::endl;
                JustinaNavigation::moveDistAngle(0.0, 1.5708, 5000);
                state = SM_MOV_LAT;
                break;
            case SM_MOV_LAT:
                std::cout << "State machine: SM_MOV_LATERAL" << std::endl;
                JustinaNavigation::moveLateral(1.0, 5000);
                state = SM_NAV_LOC;
                break;
		
		    case SM_NAV_LOC:
    			std::cout << "State machine: SM_WAIT_FOR_COMMAND" << std::endl;	
        	    JustinaNavigation::getClose("kitchen_table", 120000);
                success = true;
    			break;

        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
