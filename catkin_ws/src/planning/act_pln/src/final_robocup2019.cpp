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
//#include "justina_tools/JustinaIROS.h"
#include "std_msgs/Bool.h"
#include "string"

enum State{
    SM_WatingPrepare, SM_InitialState, SM_WaitingDoorBell, SM_NAVIGATE_TO_BAR, SM_FinalState, SM_WAIT_OBJECT_R, SM_GRASP_OBJECT_R, SM_FINAL_STATE, SM_FIND_TO_HOST, SM_WAIT_FOR_ORDER, SM_DELIVER_OBJECT
};

std::vector<std::string> tokens;

bool grantedPerson = false;
ros::Subscriber subVisitorGranted;
ros::Publisher pubWhoPerson;
ros::Publisher pubWhatAppendPerson;

void getPersonCallback(const std_msgs::Empty::ConstPtr& msg){
    std::cout << "Reciving that attend person " << std::endl;
    grantedPerson = true;
}

void sendWhoPersonName(std::string name){
    std_msgs::String msg;
    msg.data = name;
    pubWhoPerson.publish(msg);
}

void sendWhatHappend(std::string message){
    std_msgs::String msg;
    msg.data = message;
    pubWhatAppendPerson.publish(msg);
}

int main(int argc, char** argv)
{
	std::cout << "Initializing Speech and Person Recognition Test..." << std::endl;
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
    //JustinaIROS::setNodeHandle(&n);

	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
    //JustinaRepresentation::initKDB("", true, 20000);

  	//ros::Rate loop(10);

    bool fail = false;
	bool success = false;

  	int nextState = SM_InitialState;
    
    // This is for the State machine to take a normal objects
    bool alignWithTable = false;
    int attempsGrasp = 1;
    int maxAttempsGrasp = 2;
    std::vector<std::string> objsToGrasp;
    std::vector<std::string> objsToTake;
    std::string idObject;
    bool withLeftOrRightArm;
    geometry_msgs::Pose pose;
    bool armsFree[2] = {true, true};
    std::string objsToDeliv[2] = {"", ""};
    int contObj = 0;
    std::stringstream ss;
	bool openDWFlag=false; //set this flag as false due to the dishwasher will be open by default
    bool findPerson;
    float gx_w, gy_w, gz_w;
    
    std::vector<Eigen::Vector3d> faceCentroids;
    int findPersonCount;
    int findPersonAttemps;
    int findPersonRestart;
    float robot_x, robot_y, robot_a;
    float torsoSpine, torsoShoulders, torsoWaist;
    float dist_to_head;
    float angleHead;
    std::string lastReco;
    std::string lastInt;
    std::string drink;
    std::string name;
    
    std::string test("finalrobocup_2019");

    int genderRecog;

    ros::Rate loop(10);

    subVisitorGranted = n.subscribe("/alexa/visitor_granted", 1, getPersonCallback);
    pubWhoPerson = n.advertise<std_msgs::String>("/alexa/who_person", 1);
    pubWhatAppendPerson = n.advertise<std_msgs::String>("/alexa/what_happend_person", 1);

    //ros::Publisher pubstartExecuting = n.advertise<std_msgs::Empty>("/planning/start_executing", 1);

  	while(ros::ok() && !fail && !success)
  	{
        /*if(JustinaTasks::tasksStop())
        {
            JustinaTasks::sayAndSyncNavigateToLoc("exitdoor", 240000, true);
            break;
        }*/
  		switch(nextState)
    	{
           
    		case SM_InitialState:
      			std::cout << "SM_InitialState...->start Initial state" << std::endl;
               
                JustinaManip::startHdGoTo(0.0, 0.0);
                ros::Duration(1.0).sleep();
                //JustinaNavigation::moveDist(1.0, 4000);
                //JustinaHRI::say("I'm waiting for the door bell");
                //ros::Duration(1.0).sleep();
                nextState = SM_WAIT_FOR_ORDER;
                
      		break;

            case SM_WaitingDoorBell:
      			std::cout << "SM_WaitingDoorBell...->start Initial state" << std::endl;
                sendWhoPersonName("unknown");
                sendWhatHappend("default");
                    
                JustinaHRI::enableSpeechRecognized(false);//enable recognized speech
                std::cout << "Welcoming visitor Test...->waiting door bell.." << std::endl;
                //JustinaHRI::waitAfterSay("Tell me, justina start, in order to attend the door bell", 12000, maxDelayAfterSay);
                //*JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                std::cout << "Welcoming visitor Test...-> SM_WAIT_FOR_COMMAND" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("justina start", 15000)){
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    JustinaNavigation::getClose("living_room", 4000);
                    nextState = SM_WAIT_FOR_ORDER;
                }

            break; 
            
            case SM_WAIT_FOR_ORDER:
                std::cout << "SM_WAIT_FOR_ORDER...->wating for order.." << std::endl;
                JustinaHRI::enableSpeechRecognized(false);//enable recognized speech
                if(JustinaHRI::waitForSpeechRecognized(lastReco,10000)){
                    if(lastReco.compare("place cutlery") == 0){
                        // To place the cutlery
                    }
                    else{
                        boost::algorithm::split(tokens, lastReco, boost::algorithm::is_any_of(" "));
                        if(JustinaRepresentation::stringInterpretation(tokens[1], drink))
                            std::cout << "last int: " << drink << std::endl;
                        if(JustinaRepresentation::stringInterpretation(tokens[0], name))
                            std::cout << "last int: " << name << std::endl;
                        objsToGrasp = std::vector<std::string>();
                        objsToTake = std::vector<std::string>();
                        objsToGrasp.push_back(drink);
                        objsToTake.push_back(drink);
                        nextState = SM_NAVIGATE_TO_BAR;
                    }
                }
                break;

            case SM_NAVIGATE_TO_BAR:
                std::cout << "SM_NAVIGATE_TO_THE_BAR...->navigate to the bar.." << std::endl;
                ss.str("");
                ss << "I notice that " << name << " is arriving to the party, please serve the " << drink << " in the bar";
                JustinaHRI::say(ss.str());
                if (!JustinaTasks::sayAndSyncNavigateToLoc("bar", 120000)) {
					std::cout << "Final Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("bar", 120000)) {
						std::cout << "Final Test...->Third attempt to move" << std::endl;
						JustinaTasks::sayAndSyncNavigateToLoc("bar", 120000);
					} 
				}
                nextState = SM_GRASP_OBJECT_R;
            break;

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
                    JustinaNavigation::getClose("living_room", 40000);
                    ss.str("");
                    ss << name << " i am going  to find you to deliver the " << drink;
                    JustinaHRI::say(ss.str());
                    nextState = SM_FIND_TO_HOST;
                }
                break;

            case SM_FIND_TO_HOST:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to John." << std::endl;
                faceCentroids = std::vector<Eigen::Vector3d>();
                // TODO Change the name and favorite drink
                findPerson = JustinaTasks::turnAndRecognizeFace(name, -1, -1, JustinaTasks::NONE, -M_PI_4, M_PI_4 / 2.0, M_PI_4, 0, -M_PI_4 / 2.0, -M_PI_4 / 2.0, 1.0f, 1.0f, faceCentroids, genderRecog, "living_room");
                if(findPerson){
                    ss.str("");
                    ss << name << ", I found you" << std::endl;
                    JustinaHRI::waitAfterSay(ss.str(), 3000);
                    JustinaHRI::waitAfterSay("i am getting close to you", 3000);
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    JustinaTools::transformPoint("/base_link", faceCentroids[0](0, 0), faceCentroids[0](1, 0) , faceCentroids[0](2, 0), "/map", gx_w, gy_w, gz_w);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    JustinaKnowledge::addUpdateKnownLoc("charly", gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                    JustinaTasks::closeToLoclWithDistanceTHR("charly", 1.2, 40000);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
                    dist_to_head = sqrt(pow(gx_w - robot_x, 2) + pow(gy_w - robot_y, 2));
                    //JustinaManip::hdGoTo(atan2(worldFaceCentroid.y() - curry, worldFaceCentroid.x() - currx) - currtheta, atan2(worldFaceCentroid.z() - (1.45 + torsoSpine), dist_to_head), 5000);
                    angleHead = atan2(gy_w - robot_y, gx_w - robot_x) - robot_a;
                    if (angleHead < -M_PI)
                        angleHead = 2 * M_PI + angleHead;
                    if (angleHead > M_PI)
                        angleHead = 2 * M_PI - angleHead;
                    JustinaManip::hdGoTo(angleHead, atan2(gz_w - (1.45 + torsoSpine), dist_to_head), 5000);
                    nextState = SM_DELIVER_OBJECT;
                }
                else{
                    if(findPersonAttemps > 3){
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        nextState = SM_DELIVER_OBJECT;
                    }
                    else
                        findPersonAttemps++;
                    ss.str("");
                    ss << name << " i am going  to find you again to deliver the " << drink;
                    JustinaHRI::waitAfterSay(ss.str(), 5000);
                    //JustinaHRI::insertAsyncSpeech("John, I'm going to find you again", 5000, ros::Time::now().sec, 10);
                }
                break;
            
            case SM_DELIVER_OBJECT:
                std::cout << "State machine: SM_DELIVER_OBJECT" << std::endl;
                if(armsFree[0] && armsFree[1]){
                    ss.str("");
                    ss << name << " enjoy the party" << std::endl;
                    JustinaHRI::waitAfterSay(ss.str(), 4000);
                    JustinaNavigation::getClose("living_room", 60000);
                    nextState = SM_WAIT_FOR_ORDER; 
                }
                else{
                    if(!armsFree[0]){
                        JustinaManip::raGoTo("navigation", 3000);
                        JustinaTasks::dropObject(objsToDeliv[0], false, 10000);
                        armsFree[0] = true;
                    }
                    else if(!armsFree[1]){
                        JustinaManip::laGoTo("navigation", 3000);
                        JustinaTasks::dropObject(objsToDeliv[1], true, 10000);
                        armsFree[1] = true;
                    }
                }
                break;

            case SM_FINAL_STATE:
				std::cout <<"Welcoming visitor Test...->finalState reached" << std::endl;
				JustinaHRI::say("I have finished the test");
                //JustinaIROS::end_execute();
				ros::Duration(2.0).sleep();
				success=true;
			break;

        }
        ros::spinOnce();
        loop.sleep();
  }
  //JustinaTools::stopGlobalRecordRosbag();
  //JustinaTools::stopTestRecordRosbag();
  return 0;
}
