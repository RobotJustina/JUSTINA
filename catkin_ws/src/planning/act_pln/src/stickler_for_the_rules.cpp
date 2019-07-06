#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "std_msgs/Bool.h"
//#include "string"
#include <sensor_msgs/LaserScan.h>
std::vector<std::string> idsPerson;
enum STATE{
    SM_INIT,
    SM_SAY_WAIT_FOR_DOOR,
    SM_WAIT_FOR_DOOR,
    SM_NAVIGATE_TO_THE_ARENA,
    SM_INSPECT_BLACKROOM,
    SM_Take_Out_Invader,
    SM_Search_Drinks,
    SM_Look_Person,
    SM_ask_help_Person,
    SM_SearchPeopleForDrink,
    SM_IntroducePeople,
    SM_CLOSE_TO_GUEST,
    SM_GRASP_OBJECT_R,
    SM_WAIT_OBJECT_R,
    SM_DELIVER_OBJECT,
    SM_REPETE_RULE,
    SM_WAIT_TO_REPETE_RULE,
    SM_FINAL_STATE
};

void switchSpeechReco(bool pocket, std::string grammarPocket, std::string grammarMicrosoft, std::string speech){
    if (pocket){
        //use pocket sphinx
        JustinaHRI::usePocketSphinx = true;
        JustinaHRI::enableGrammarSpeechRecognized(grammarPocket, 0);
	    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
        JustinaHRI::enableSpeechRecognized(false);
	    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
        JustinaHRI::waitAfterSay(speech,5000);
        JustinaHRI::enableSpeechRecognized(true);
    }

    else{
        //use speech recognition of microsoft
        JustinaHRI::usePocketSphinx = false;
        JustinaHRI::loadGrammarSpeechRecognized(grammarMicrosoft);
        JustinaHRI::waitAfterSay(speech,5000);
        JustinaHRI::enableSpeechRecognized(true);
    }
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING HELP ME CARRY TEST..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    boost::posix_time::ptime prev;
    boost::posix_time::ptime curr;

    JustinaTasks::POSE poseRecog;
    idsPerson.push_back("person");
    std::string blackroom = "bedroom";
    std::stringstream ss;

    std::vector<Eigen::Vector3d> centroidGestures;
    std::vector<std::string> centroids_loc;
    float gx_w, gy_w, gz_w; 
    float goalx, goaly, goala, angleError;
    float robot_y, robot_x, robot_a;
     
    int minDelayAfterSay = 0;
    int maxDelayAfterSay = 300;

    float dist_to_head;

    int nextState = SM_INIT;
    bool fail = false;
    bool success = false;
    bool findInvader = false;
    bool findGesture = false;
    bool reachedGoal = false;
    
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
	bool openDWFlag=false; //set this flag as false due to the dishwasher will be open by default
    int countRepetOrder = 0;
    int maxCountRepetOrder = 3;
    std::string grammarCommands = "restaurant_commands.xml";

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {  

            case SM_INIT:
                std::cout << "State machine: SM_INIT" << std::endl;	
                JustinaManip::startHdGoTo(0.0, 0.0);
                JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
                JustinaHRI::waitAfterSay("I am ready for RoboCop Test", 2000, minDelayAfterSay);
                //JustinaHRI::loadGrammarSpeechRecognized("CarryMyLuggage.xml");//load the grammar
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                nextState = SM_SAY_WAIT_FOR_DOOR;
                break;
            
            case SM_SAY_WAIT_FOR_DOOR:
                std::cout << "State machine: SM_Say_wait_door" << std::endl;	
				JustinaHRI::waitAfterSay("I am waiting for the door to be open", 4000);
				nextState = SM_WAIT_FOR_DOOR;
			break;

            case SM_WAIT_FOR_DOOR:
                std::cout << "State machine: SM_wait_door" << std::endl;
				if (!JustinaNavigation::obstacleInFront())
					nextState = SM_NAVIGATE_TO_THE_ARENA;
			break;


            case SM_NAVIGATE_TO_THE_ARENA:
				JustinaHRI::waitAfterSay("Now I can see that the door is open",4000);
				std::cout << "RoboCop...->First attempt to move" << std::endl;
            	JustinaNavigation::moveDist(1.0, 4000);

				JustinaHRI::waitAfterSay("I will inspect the black room",1000);
				
				if (!JustinaTasks::sayAndSyncNavigateToLoc(blackroom, 120000)) {
					std::cout << "P & G Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc(blackroom, 120000)) {
						std::cout << "P & G Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc(blackroom, 120000)) {
							std::cout << "RoboCop Test...->moving to the voice command point" << std::endl;
							nextState = SM_INSPECT_BLACKROOM;
						}
					} 
					else{
						std::cout << "Robocop...->moving to the voice command point" << std::endl;
						nextState = SM_INSPECT_BLACKROOM;
					}
				} 
				else {
					std::cout << "Robocop...->moving to the voice command point" << std::endl;
					nextState = SM_INSPECT_BLACKROOM;
				}
            	std::cout << "Robocop...->moving to the voice command point" << std::endl;
				nextState = SM_INSPECT_BLACKROOM;

			break;

            case SM_INSPECT_BLACKROOM:
                std::cout << "State machine: SM_inspect_blackroom" << std::endl;
                JustinaHRI::waitAfterSay("I am looking for an invader",6000);
                findInvader = JustinaTasks::findYolo(idsPerson, poseRecog, JustinaTasks::NONE, blackroom);
                if(findInvader){
                    countRepetOrder = 0;
                    JustinaHRI::waitAfterSay("Hello, my name is Justina, you are not allowed to stay in this room",3000);
                    nextState = SM_REPETE_RULE;
                }else
                    nextState = SM_Search_Drinks;

                break;

            case SM_Take_Out_Invader:
                std::cout << "State machine: SM_take_out_invader" << std::endl;
                //JustinaHRI::waitAfterSay("Hello, my name is Justina, you are not allowed to stay in this room, let me guide you to another room",3000);
                if(JustinaTasks::guideAPerson("living_room", 90000, 1.5)){
                    JustinaNavigation::getClose("living_room", 30000);
                    nextState= SM_Look_Person;
                }else{
                    JustinaHRI::waitAfterSay("I cannot guide you out of the blackroom, i will look for help in the living room",2000);
                    nextState=SM_ask_help_Person;
                }
                break;
            
            case SM_REPETE_RULE:
                std::cout << "State machine: SM_REPETE_RULE" << std::endl;
                JustinaHRI::enableSpeechRecognized(false);
                JustinaHRI::waitAfterSay("Please tell me, robot yes or robot no, if you understand the rule", 5000, 400);
                JustinaHRI::loadGrammarSpeechRecognized(grammarCommands);//load the grammar
                JustinaHRI::enableSpeechRecognized(true);
                /* CHANGE **************** */ 
                /* ---------------- */
                // JustinaHRI::waitAfterSay("you understood the order, tell me robot yes", 5000, maxDelayAfterSay);
                // JustinaHRI::enableSpeechRecognized(true);
                countRepetOrder++;
                nextState = SM_WAIT_TO_REPETE_RULE;
                break;
            
            case SM_WAIT_TO_REPETE_RULE:
                std::cout << "State machine: SM_WAIT_FOR_REPETE_ORDER" << std::endl;
                if(countRepetOrder <= maxCountRepetOrder){
                    if(JustinaHRI::waitForSpecificSentence("robot yes", 4000)){
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay("Let me guide you to another room",3000);
                        //JustinaHRI::enableSpeechRecognized(true);
                        nextState = SM_Take_Out_Invader;
                    }else
                        nextState = SM_REPETE_RULE;
                }
                else{
                    JustinaHRI::enableSpeechRecognized(false);
                    JustinaHRI::waitAfterSay("Let me guide you to another room",3000);
                    nextState = SM_Take_Out_Invader;
                }
                break;
            
            case SM_ask_help_Person:
                JustinaTasks::sayAndSyncNavigateToLoc("living_room", 120000);
                JustinaTasks::findYolo(idsPerson, poseRecog, JustinaTasks::NONE, "arena");
                JustinaHRI::waitAfterSay("Hello Human, I need your help, there an invader in the blackroom, please go to the blackroom and take him out",5000);
                JustinaHRI::waitAfterSay("Tanks in advance, and enjoy the party", 1500);
                nextState = SM_SearchPeopleForDrink;
                break;
            
            case SM_Look_Person:
                std::cout << "State machine: SM_take_out_invader" << std::endl;
                JustinaHRI::waitAfterSay("Let my find another guest to introduce you",1500);
                JustinaHRI::waitAfterSay("when i have found him, please follow me", 3000);
                JustinaTasks::findYolo(idsPerson, poseRecog, JustinaTasks::NONE, "arena");
                nextState = SM_IntroducePeople;
                break;
            
            case SM_IntroducePeople:
                JustinaVision::startSkeletonFinding();
                JustinaHRI::waitAfterSay("Hello, my name is Justina, and I came here with a friend, please talk with him for a while",2500);
                JustinaVision::startSkeletonFinding();
                JustinaHRI::waitAfterSay("Enjoy the party", 500);
                JustinaVision::startSkeletonFinding();
                nextState = SM_SearchPeopleForDrink;
                break;

            case SM_SearchPeopleForDrink:
                JustinaTasks::sayAndSyncNavigateToLoc("living_room", 120000);
                JustinaHRI::waitAfterSay("Hello, my name is Justina, all the people who does not have a drink please waving , to offer you a drink",5000);
                centroidGestures = std::vector<Eigen::Vector3d>();
                findGesture = JustinaTasks::turnAndRecognizeGesture("waving", -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2, -0.2, -0.2, M_PI_2, 2 * M_PI, 9.0, centroidGestures, "arena", true);
                if(findGesture){
                    JustinaVision::stopSkeletonFinding();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                    ros::spinOnce();

                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    for(int i = 0; i < centroidGestures.size(); i++)
                    {
                    	ss.str("");
                        Eigen::Vector3d centroid = centroidGestures[i];
                        JustinaTools::transformPoint("/base_link", centroid(0, 0), centroid(1, 0) , centroid(2, 0), "/map", gx_w, gy_w, gz_w);
                        ss << "person_" << i;
                        centroids_loc.push_back(ss.str());
                        JustinaKnowledge::addUpdateKnownLoc(ss.str(), gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                    }

                    JustinaManip::hdGoTo(0.0, 0.0, 1000);
                    ss.str("");
                    ss << "I noticed that somebody are calling me " << std::endl;

                    JustinaHRI::waitAfterSay(ss.str(), 5000, minDelayAfterSay);
                    JustinaHRI::waitAfterSay("I am going to approach to you", 5000, maxDelayAfterSay);
                    nextState = SM_CLOSE_TO_GUEST;
                }else
                    nextState = SM_SearchPeopleForDrink;
                break;
            
            case SM_CLOSE_TO_GUEST:
                std::cout << "Stickler Test...-> SM_CLOSE_TO_GUEST" << std::endl;
                JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                JustinaKnowledge::getKnownLocation(centroids_loc[0], goalx, goaly, goala);
                std::cout << "Farewell Test...->Centroid gesture:" << goalx << "," << goaly << "," << goala << std::endl;
                //reachedGoal = JustinaTasks::closeToLoclWithDistanceTHR(ss.str(), 0.9, 120000);
                reachedGoal = JustinaTasks::closeToLoclWithDistanceTHR(centroids_loc[0], 1.5, 30000);
                JustinaTasks::closeToGoalWithDistanceTHR(goalx, goaly, 1.5, 30000);
                reachedGoal = true;
                
                JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly- robot_y, 2));

                if(reachedGoal)
                    JustinaKnowledge::addUpdateKnownLoc(centroids_loc[0], robot_a);

                float torsoSpine, torsoWaist, torsoShoulders;
                JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
                float angleHead;
                angleHead = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
                if(angleHead < -M_PI)
                    angleHead = 2 * M_PI + angleHead;
                if(angleHead > M_PI)
                    angleHead = 2 * M_PI - angleHead;
                JustinaManip::startHdGoTo(angleHead, atan2(gz_w - (1.53 + torsoSpine), dist_to_head));
                
                JustinaHRI::waitAfterSay("Hello, my name is Justina, let me guide you to the bar",1500);
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            
                JustinaTasks::guideAPerson("bar", 120000, 1.5);

                objsToGrasp = std::vector<std::string>();
                objsToGrasp.push_back("juice");

                //TODO Preguntar que bebida quiere, cargar gramatica de bebidas, pedir que le pongan la bebida en el griper y dar bebida al guest
                nextState = SM_GRASP_OBJECT_R;
                break;

            case SM_GRASP_OBJECT_R:
                std::cout << "State machine: SM_GRASP_OBJECT" << std::endl;
                JustinaHRI::say("Hey barman, i need a juice for the guest, please put the juice in front of me");
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                JustinaHRI::say("Thank you");
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
                    JustinaNavigation::moveDistAngle(0.0, M_PI, 3000);
                    JustinaNavigation::moveDist(0.1, 3000);
                    JustinaHRI::say("Human please, go back one step");
                    nextState = SM_DELIVER_OBJECT;
                }
                break;
            
            case SM_DELIVER_OBJECT:
                std::cout << "State machine: SM_DELIVER_OBJECT" << std::endl;
                if(armsFree[0] && armsFree[1]){
                    JustinaHRI::waitAfterSay("Human enjoy the party", 4000, minDelayAfterSay);
                    nextState = SM_FINAL_STATE; 
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
                std::cout << "State machine: SM_FINAL_STATE" << std::endl;
                JustinaHRI::say("I have finished the test");
                success = true;
                break;
        }
        ros::spinOnce();
        loop.sleep();
    }
    return -1;
}
