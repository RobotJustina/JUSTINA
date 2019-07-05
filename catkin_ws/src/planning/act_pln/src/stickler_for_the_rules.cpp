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

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {  

            case SM_INIT:
                std::cout << "State machine: SM_INIT" << std::endl;	
                JustinaManip::startHdGoTo(0.0, 0.0);
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

            SM_INSPECT_BLACKROOM:
                std::cout << "State machine: SM_inspect_blackroom" << std::endl;
                JustinaHRI::waitAfterSay("I am looking for an invader",6000);
                findInvader = JustinaTasks::findYolo(idsPerson, poseRecog, JustinaTasks::NONE, blackroom);
                if(findInvader)
                    nextState = SM_Take_Out_Invader;
                else
                    nextState = SM_Search_Drinks;

                break;

            SM_Take_Out_Invader:
                std::cout << "State machine: SM_take_out_invader" << std::endl;
                JustinaHRI::waitAfterSay("Hello, my name is Justina, you are not allowed to stay in this room, let me guide you to another room",3000);
                if(JustinaTasks::guideAPerson("living_room", 120000, 1.5))
                    nextState= SM_Look_Person;
                else{
                    JustinaHRI::waitAfterSay("I cannot guide you out of the blackroom, i will look for help in the living room",2000);
                    nextState=SM_ask_help_Person;
                }
                break;

            SM_ask_help_Person:
                JustinaTasks::sayAndSyncNavigateToLoc("living_room", 120000);
                JustinaTasks::findYolo(idsPerson, poseRecog, JustinaTasks::NONE, "arena");
                JustinaHRI::waitAfterSay("Hello Human, I need your help, there an invader in the blackroom, please go to the blackroom and take him out",5000);
                JustinaHRI::waitAfterSay("Tanks in advance, and enjoy the party", 1500);
                SM_SearchPeopleForDrink;
                break;
            
            SM_Look_Person:
                std::cout << "State machine: SM_take_out_invader" << std::endl;
                JustinaHRI::waitAfterSay("Let my find another guest to introduce you",1500);
                JustinaHRI::waitAfterSay("when i have found him, please follow me", 3000);
                JustinaTasks::findYolo(idsPerson, poseRecog, JustinaTasks::NONE, "arena");
                SM_IntroducePeople;
                break;
            
            SM_IntroducePeople:
                JustinaHRI::waitAfterSay("Hello, my name is Justina, and I came here with a friend, please talk with him for a while",2500);
                JustinaHRI::waitAfterSay("Enjoy the party", 500);
                SM_SearchPeopleForDrink;
                break;

            SM_SearchPeopleForDrink:
                JustinaTasks::sayAndSyncNavigateToLoc("living_room", 120000);
                JustinaHRI::waitAfterSay("Hello, my name is Justina, all the people who does not have a drink please waving, to offer you a drink",5000);
                centroidGestures = std::vector<Eigen::Vector3d>();
                findGesture = JustinaTasks::turnAndRecognizeGesture("waving", -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2, -0.2, -0.2, 0.1, 0.1f, 9.0, centroidGestures, "", true, 0, 0.7);
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
                reachedGoal = JustinaTasks::closeToLoclWithDistanceTHR(centroids_loc[0], 1.2, 30000);
                JustinaTasks::closeToGoalWithDistanceTHR(goalx, goaly, 1.2, 30000);
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
                JustinaManip::startHdGoTo(angleHead, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
                
                JustinaHRI::waitAfterSay("Hello, my name is Justina, let me guide you to the bar",1500);
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            
                JustinaTasks::guideAPerson("bar", 120000, 1.5);

                //TODO Preguntar que bebida quiere, cargar gramatica de bebidas, pedir que le pongan la bebida en el griper y dar bebida al guest
                nextState = SM_FINAL_STATE;
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