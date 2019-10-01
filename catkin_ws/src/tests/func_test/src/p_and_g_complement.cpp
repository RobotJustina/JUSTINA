#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaAudio.h"
#include "justina_tools/JustinaRepresentation.h"
#include "justina_tools/JustinaTasks.h"
#include "std_msgs/Bool.h"
#include "string"

#define SM_GRASP_OBJECT_R 101
#define SM_WAIT_OBJECT_R    105
#define SM_DeliverObject_R  110
#define	SM_FinalState 80

int main(int argc, char** argv)
{
    std::cout << "Initializing P & G Test..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaAudio::setNodeHandle(&n);
    JustinaRepresentation::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
    JustinaKnowledge::setNodeHandle(&n);//knowledge

    bool fail = false;
    bool success = false;

    int nextState = 0;

    //set the KINECT as the input device 
    JustinaHRI::setInputDevice(JustinaHRI::RODE);

    // This is for the State machine to take a normal objects
    bool alignWithTable = false;
    int attempsGrasp = 1;
    int maxAttempsGrasp = 2;
    std::vector<std::string> objsToGrasp;
    std::vector<std::string> objsToTake;
    objsToGrasp.push_back("bowl");
    objsToGrasp.push_back("glass");
    objsToTake.push_back("bowl");
    objsToTake.push_back("glass");
    std::string idObject;
    bool withLeftOrRightArm;
    geometry_msgs::Pose pose;
    bool armsFree[2] = {true, true};
    std::string objsToDeliv[2] = {"", ""};
    int contObj = 0;
    std::stringstream ss;
	bool openDWFlag=false; //set this flag as false due to the dishwasher will be open by default

    while(ros::ok() && !fail && !success)
    {
        ros::Rate loop(10);
        switch(nextState)
        {
            case SM_GRASP_OBJECT_R:
                std::cout << "State machine: SM_GRASP_OBJECT" << std::endl;
                if(objsToGrasp.size() > 0){
                    idObject = objsToGrasp[0];
                    if(!alignWithTable && !JustinaTasks::alignWithTable(0.4)){
                        std::cout << "I can´t align with table   :´(" << std::endl;
                        JustinaNavigation::moveDistAngle(-0.05, M_PI_4/4, 2000);
                        JustinaTasks::alignWithTable(0.4);
                        JustinaTasks::alignWithTable(0.4);
                    }
                    alignWithTable = true;
                    if(attempsGrasp <= maxAttempsGrasp){
                        attempsGrasp++;
                        if(JustinaTasks::findObject(idObject, pose, withLeftOrRightArm)){
                            // index 0 is right arm index 1 is left arm
                            /*if(!(withLeftOrRightArm && armsFree[1]))
                              withLeftOrRightArm = false;
                              else if(!(!withLeftOrRightArm && armsFree[0]))
                              withLeftOrRightArm = true;*/
                            if(withLeftOrRightArm){
                                if(!armsFree[1])
                                    withLeftOrRightArm = false;
                            }
                            else{
                                if(!armsFree[0])
                                    withLeftOrRightArm = true;
                            }
                            //if(JustinaTasks::moveActuatorToGrasp(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, idObject)){
                            // If we want to use another frame we need to pass de id how not empty
                            if(JustinaTasks::graspObject(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, "", true)){
                                if(withLeftOrRightArm){
                                    objsToDeliv[1] = idObject;
                                    armsFree[1] = false;
                                }else{
                                    objsToDeliv[0] = idObject;
                                    armsFree[0] = false;
                                }
                                objsToGrasp.erase(objsToGrasp.begin());
                                objsToTake.erase(objsToTake.begin());
                            }
                        }
                    }
                    else{
                        alignWithTable = false;
                        attempsGrasp = 1;
                        nextState = SM_WAIT_OBJECT_R;
                    }
                }
                else{
                    alignWithTable = false;
                    attempsGrasp = 1;
                    nextState = SM_WAIT_OBJECT_R;
                }
                break;

            case SM_WAIT_OBJECT_R:
                std::cout << "State machine: SM_WAIT_OBJECT" << std::endl;
                if(objsToTake.size() > 0){
                    idObject = objsToTake[0];
                    ss.str("");
                    ss << "I can not take the " << idObject << ", but i will take the " << idObject << " if you put it in my gripper";
                    JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
                    if(armsFree[0]){
                        JustinaManip::raGoTo("navigation", 3000);
                        JustinaTasks::detectObjectInGripper(idObject, false, 7000);
                        objsToDeliv[0] = idObject;
                        armsFree[0] = false;
                        contObj++;
                    }else if(armsFree[1]){
                        JustinaManip::laGoTo("navigation", 3000);
                        objsToDeliv[1] = idObject;
                        JustinaTasks::detectObjectInGripper(idObject, true, 7000);
                        armsFree[1] = false;
                        contObj++;
                    }
                    objsToTake.erase(objsToTake.begin());
                    std::vector<std::string>::iterator it = std::find(objsToGrasp.begin(), objsToGrasp.end(), idObject);
                    if(it != objsToGrasp.end())
                        objsToGrasp.erase(it);
                    nextState = SM_WAIT_OBJECT_R;
                    if(objsToGrasp.size() > 0)
                        nextState = SM_GRASP_OBJECT_R;
                }
                else{
                    // THIS IS FOR NAVIGATION TO THE DISH WASHER
                }
                break;

      		case SM_DeliverObject_R:
                std::cout << "State machine: SM_DeliverObject_r" << std::endl;
				if(openDWFlag){
					JustinaHRI::say("Human, please, open the dishwasher just until the half");
					ros::Duration(0.5).sleep();
					JustinaHRI::say("for example just open it 45 degrees");
					ros::Duration(0.5).sleep();
					JustinaHRI::say("Human, please, pull off the rack");
					ros::Duration(5.0).sleep();
					JustinaHRI::say("thank you");
					ros::Duration(0.5).sleep();
				}
      			
				if(!JustinaTasks::placeObjectDishWasher(0.275, 0.2, 0.075)){
      				std::cout << "P & G Test...-> cannot deliver the object" << std::endl;
					JustinaNavigation::moveDist(-0.25, 3000);  
					nextState = SM_DeliverObject_R;
					break;
				}
				
      			if(contObj == 5){
      				//nextState = SM_NAVIGATE_TO_THE_EXIT;
					JustinaHRI::say("human close the diswasher, please");
					ros::Duration(0.5).sleep();
					JustinaManip::startTorsoGoTo(0.1, 0, 0);
					JustinaManip::waitForTorsoGoalReached(0.5);
				}
				else{
					//objTaken = 0;
					//nextState = SM_NAVIGATE_TO_THE_TABLE;
					JustinaHRI::say("human, please, keep the diswasher open for me");
					ros::Duration(0.5).sleep();
					//attempts = 0;
					JustinaManip::startTorsoGoTo(0.1, 0, 0);
					JustinaManip::waitForTorsoGoalReached(0.5);
				}
                break;

            case SM_FinalState:
                std::cout <<"P & G Test...->finalState reached" << std::endl;
                JustinaHRI::say("I have finished the procter & Gamble challenge");
                ros::Duration(2.0).sleep();
                success=true;
                break;

            }
            ros::spinOnce();
            loop.sleep();
    }
    return 0;
}
