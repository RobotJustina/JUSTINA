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

#define SM_INIT 0
#define SM_WAIT_FOR_INIT_COMMAND 10
#define SM_SEARCH_WAVING 20
#define SM_CLOSE_TO_GUEST 30
#define SM_RecognizeGuest 40
#define SM_ReturnSearchWaving 50
#define SM_GoCoatRack 60
#define SM_SearchTaxiDriver 70
#define SM_CLOSE_TO_TAXI_DRIVER 80
#define SM_RETURN_INITIAL_POINT 90
#define SM_FINAL_STATE 100



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
    
	JustinaHRI::loadGrammarSpeechRecognized("welcoming_visitors.xml");//load the grammar
	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
    
  	int nextState = 0;
    bool fail = false;
    bool success = false;
  	bool recog=false;
    bool findUmbrella = false;
    bool findGesture = false;
    
    int timeoutspeech = 10000;

    int attempsSpeechReco = 1;
    int attempsSpeechInt = 1;
    int attempsConfirmation = 1;
    int attempsWaitConfirmation = 1;
    int maxAttempsConfirmation = 3;
    int maxAttempsWaitConfirmation = 3;

    int minDelayAfterSay = 0;
    int maxDelayAfterSay = 300;
    
    std::vector<std::string> confirmCommands;
    confirmCommands.push_back("justina yes");
    confirmCommands.push_back("justina no");

    std::vector<std::string> validCommandsVisit;
    validCommandsVisit.push_back("i want to visit the kitchen");
    validCommandsVisit.push_back("i want to visit the bedroom");
    validCommandsVisit.push_back("i want to visit the bathroom");

    std::vector<std::string>plumberNotAllowed;
    plumberNotAllowed.push_back("bedroom");
    std::vector<std::string>delimanNotAllowed;
    delimanNotAllowed.push_back("bathroom");
    delimanNotAllowed.push_back("bedroom");

    std::vector<std::string> idsUmbrella;
    idsUmbrella.push_back("umbrella");
    

    Eigen::Vector3d centroidGesture;
    float gx_w, gy_w, gz_w; 
    float goalx, goaly, goala, angleError;
    float robot_y, robot_x, robot_a;
    bool reachedGoal = false;
    float dist_to_head;
    int numberGuest = 1;
    int maxNumberGuest = 2;
    int numberTaxi = 0;

    std::string idGuest;
    float posGuestX, posGuestY, posGuestZ,confidenceGuest;
    int genderGuest; 
    bool smileGuest;

    bool withLeftArm;

    std::stringstream ss;
    std::string lastRecoSpeech;

	JustinaHRI::setInputDevice(JustinaHRI::KINECT);

  	//almacena los rostros detectados por el servicio
  	vision_msgs::VisionFaceObjects faces;
  	//alamcena los gestos detectados
  	std::vector<vision_msgs::GestureSkeleton> gestures;
    
    
    ros::Rate loop(10);

    while(ros::ok() && !fail && !success)
  	{
        switch(nextState)
    	{
            case SM_INIT:
                std::cout << "Farewell Test...->start Farewell test" << std::endl;	
                JustinaManip::hdGoTo(0.0, 0.0, 2000);
                if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
					std::cout << "Farewell Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
						std::cout << "Farewell Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
							std::cout << "Farewell...->moving to the initial point" << std::endl;
						}
					} 
				}
                JustinaHRI::waitAfterSay("I'm ready for the farewell test, tell me, justina start, to performing the test", timeoutspeech, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);
                nextState = SM_WAIT_FOR_INIT_COMMAND;
                break;

            case SM_WAIT_FOR_INIT_COMMAND:
                std::cout << "Farewell Test...->SM_WAIT_FOR_INIT_COMMAND" << std::endl;
				if(JustinaHRI::waitForSpecificSentence("justina start", timeoutspeech)){
                    JustinaHRI::enableSpeechRecognized(false);
                    JustinaHRI::say("Hey human, please lend me the umbrella for the guests");
                    ros::Duration(1.5).sleep();
                    JustinaHRI::say("please close the umbrella and put in my gripper");
                    ros::Duration(1.5).sleep();
                    JustinaTasks::detectObjectInGripper("umbrella", true, 7000);
                    withLeftArm = true;
                    ros::Duration(1.0).sleep();
                    JustinaVision::startSkeletonFinding();
                    nextState = SM_SEARCH_WAVING;
                }else
                    nextState = SM_WAIT_FOR_INIT_COMMAND;
                break;
            
            case SM_SEARCH_WAVING:
                std::cout << "Farewell Test...->SM_SEARCH_WAVING" << std::endl;
                JustinaHRI::waitAfterSay("I will search the guests", 3500, minDelayAfterSay);
                findGesture = JustinaTasks::turnAndRecognizeGesture("waving", -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2, -0.2, -0.2, 0.0, 0.0f, 9.0, centroidGesture, "", true);
                if(findGesture){
                    JustinaVision::stopSkeletonFinding();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                    ros::spinOnce();
                   
                    JustinaTools::transformPoint("/base_link", centroidGesture(0, 0), centroidGesture(1, 0) , centroidGesture(2, 0), "/map", gx_w, gy_w, gz_w);

                    JustinaManip::hdGoTo(0.0, 0.0, 1000);
 
                    ss.str("");
                    ss << "I noticed that somebody are calling me " << std::endl;

                    if(centroidGesture(1, 0) > -0.4 && centroidGesture(1, 0) < 0.4)
                        ss << "in front of me";
                    else if(centroidGesture(1, 0) > 0.4)
                        ss << "in my left side";
                    else if(centroidGesture(1, 0) < -0.4)
                        ss << "in my right side";

                    JustinaHRI::waitAfterSay(ss.str(), 5000, minDelayAfterSay);
                    JustinaHRI::waitAfterSay("I am going to approach to you for confirmation", 5000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);
                    nextState = SM_CLOSE_TO_GUEST;
                }else
                    nextState = SM_SEARCH_WAVING;
                break;
            
            case SM_CLOSE_TO_GUEST:
                std::cout << "Farewell Test...-> SM_CLOSE_TO_GUEST" << std::endl;
                ss.str("");
                ss << "guest_" << numberGuest;
                JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                JustinaKnowledge::addUpdateKnownLoc(ss.str(), gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                JustinaKnowledge::getKnownLocation(ss.str(), goalx, goaly, goala);
                std::cout << "Farewell Test...->Centroid gesture:" << goalx << "," << goaly << "," << goala << std::endl;
                reachedGoal = JustinaTasks::closeToLoclWithDistanceTHR(ss.str(), 1.5, 180000);
                JustinaTasks::closeToGoalWithDistanceTHR(gx_w, gy_w, 1.5, 180000);
                reachedGoal = true;
                
                JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly- robot_y, 2));

                if(reachedGoal)
                    JustinaKnowledge::addUpdateKnownLoc(ss.str(), robot_a);


                float torsoSpine, torsoWaist, torsoShoulders;
                JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);

                JustinaManip::startHdGoTo(atan2(goaly - robot_y, goalx - robot_x) - robot_a, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
                // *JustinaManip::startHdGoTo(0, atan2(gz_w - 1.6, dist_to_head));
                 
                JustinaHRI::waitAfterSay("Hello my name is Justina, and I think that you want to go, is that correct, tell me justina yes or justina no", 12000);
                ros::Duration(1.0).sleep();
                JustinaHRI::enableSpeechRecognized(true);
                JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, timeoutspeech);
                if(lastRecoSpeech.find("yes") != std::string::npos || attempsConfirmation == 3){
                    attempsSpeechReco = 1;
                    attempsSpeechInt = 1;
                    attempsConfirmation = 1;
                    attempsWaitConfirmation = 1;
                    JustinaHRI::enableSpeechRecognized(false);
                    nextState = SM_RecognizeGuest;
                }
                else{
                     JustinaHRI::enableSpeechRecognized(false);
                     nextState = SM_ReturnSearchWaving;
                }
                break;

            case SM_RecognizeGuest:
                std::cout << "Farewell Test...-> SM_RecognizeVisitor" << std::endl;
                JustinaHRI::say("Human please look at me to recognize you");
        		ros::Duration(1.0).sleep();
                if(!JustinaVision::getMostConfidentFace(idGuest, posGuestX, posGuestY, posGuestZ, confidenceGuest, genderGuest, smileGuest)){
                    std::cout << "Farewell Test...->First attempt to recognize guest" << std::endl;
                    if(!JustinaVision::getMostConfidentFace(idGuest, posGuestX, posGuestY, posGuestZ, confidenceGuest, genderGuest, smileGuest)){
                        std::cout << "Farewell Test...->Second attempt to recognize guest" << std::endl;
                        if(!JustinaVision::getMostConfidentFace(idGuest, posGuestX, posGuestY, posGuestZ, confidenceGuest, genderGuest, smileGuest)){
                            std::cout << "Farewell Test...->Third attempt to recognize guest" << std::endl;
                            idGuest = "guest";
                        }
                    }
                }

                ss.str("");
                ss << "Thank you " << idGuest;
                JustinaHRI::say(ss.str());
        		ros::Duration(1.0).sleep();

                JustinaHRI::say("It is rainning outside and I think you will need an umbrella");
				ros::Duration(1.0).sleep();
                JustinaManip::laGoTo("navigation", 3000);
                JustinaTasks::dropObject("umbrella", withLeftArm, 10000);

                nextState = SM_GoCoatRack;
                break;
            
            case SM_ReturnSearchWaving:
                std::cout << "Farewell Test...-> SM_ReturnSearchWaving" << std::endl;
                JustinaHRI::say("Sorry for my mistake, please enjoy the party");
        		ros::Duration(1.0).sleep();
                JustinaNavigation::startMoveDistAngle(-0.2, 1.15);

                if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
					std::cout << "Farewell Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
						std::cout << "Farewell Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
							std::cout << "Farewell...->moving to the initial point" << std::endl;
							nextState = SM_SEARCH_WAVING;
						}
					} 
					else{
						std::cout << "Farewell...->moving to the initial point" << std::endl;
						nextState = SM_SEARCH_WAVING;
					}
				}
                else {
					nextState = SM_SEARCH_WAVING;
				} 
                JustinaVision::startSkeletonFinding();

                break;
            
            case SM_GoCoatRack:
                std::cout << "Farewell Test...-> SM_GoCoatRack" << std::endl;
                JustinaHRI::say("I am going to guide you to the coat rack");
                ros::Duration(1.0).sleep();
                JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                ros::Duration(1.0).sleep();
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            
                JustinaTasks::guideAPerson("arena", 50000000, 1.5);
                
                JustinaHRI::say("please take your coat");
        		ros::Duration(2.0).sleep();
                JustinaHRI::say("ready, now i will take you outside to guide you to the taxi");
        		ros::Duration(2.0).sleep();

                JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                ros::Duration(1.0).sleep();
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            
                JustinaTasks::guideAPerson("corridor", 50000000, 1.5);
                
                JustinaHRI::say("wait here with me I am looking for the taxi driver");
        		ros::Duration(1.5).sleep();
                nextState = SM_SearchTaxiDriver;
                
                break;

            case SM_SearchTaxiDriver:
                std::cout << "Farewell Test...-> SM_SearchTaxiDriver" << std::endl;
                JustinaHRI::waitAfterSay("I am looking for the taxi driver", 3500, minDelayAfterSay);
                findUmbrella = JustinaTasks::findAndGuideYolo(idsUmbrella);
                //findGesture = JustinaTasks::turnAndRecognizeGesture("waving", -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2, -0.2, -0.2, 0.0, 0.0f, 9.0, centroidGesture, "", true);
                if(findUmbrella){
                    //JustinaVision::stopSkeletonFinding();
                    //boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                    //ros::spinOnce();
                   
                    //JustinaTools::transformPoint("/base_link", centroidGesture(0, 0), centroidGesture(1, 0) , centroidGesture(2, 0), "/map", gx_w, gy_w, gz_w);

                    JustinaManip::hdGoTo(0.0, 0.0, 1000);

                    //JustinaHRI::waitAfterSay("I have found the taxi driver", 5000, minDelayAfterSay);
                    nextState = SM_CLOSE_TO_TAXI_DRIVER;
                }else
                    nextState = SM_SearchTaxiDriver;
                break;

            case SM_CLOSE_TO_TAXI_DRIVER:
                 
                JustinaHRI::waitAfterSay("Hello Taxi driver, my name is Justina, I came here with a guest that want to go home", 12000);
                ros::Duration(1.0).sleep();

                if(numberGuest<maxNumberGuest){
                    JustinaHRI::say("Hey guest someone else are waiting for me inside, could you please lend me the umbrella");
                    ros::Duration(1.5).sleep();
                    JustinaHRI::say("please close the umbrella and put in my gripper");
                    ros::Duration(1.5).sleep();
                    JustinaTasks::detectObjectInGripper("umbrella", true, 7000);
                    withLeftArm = true;
                    ros::Duration(1.0).sleep();
                    ss.str("");
                    ss << "Good bye " << idGuest;
                    JustinaHRI::say(ss.str());
                    ros::Duration(1.0).sleep();
                    JustinaHRI::say("hey taxi driver, please drive carefully, good bye");
                    ros::Duration(1.5).sleep();
                    nextState = SM_RETURN_INITIAL_POINT;
                }

                else{
                    JustinaHRI::say("Hey guest i hope you have a nice trip, could you please lend me the umbrella");
                    ros::Duration(1.5).sleep();
                    JustinaHRI::say("please close the umbrella and put in my gripper");
                    ros::Duration(1.5).sleep();
                    JustinaTasks::detectObjectInGripper("umbrella", true, 7000);
                    withLeftArm = true;
                    ros::Duration(1.0).sleep();
                    ss.str("");
                    ss << "Good bye " << idGuest;
                    JustinaHRI::say(ss.str());
                    ros::Duration(1.0).sleep();
                    JustinaHRI::say("hey taxi driver, please drive carefully, good bye");
                    ros::Duration(1.5).sleep();
                    nextState = SM_FINAL_STATE;
                }
                
                break;

            case SM_RETURN_INITIAL_POINT:
                std::cout << "Farewell Test...-> SM_RETURN_INITIAL_POINT" << std::endl;
                JustinaManip::hdGoTo(0.0, 0.0, 2000);
                if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
			    	std::cout << "Farewell Test...->Second attempt to move" << std::endl;
			    	if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
			    		std::cout << "Farewell Test...->Third attempt to move" << std::endl;
			    		if (JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
			    			std::cout << "Farewell...->moving to the initial point" << std::endl;
			    		}
			    	} 
			    }
                nextState= SM_SEARCH_WAVING;
                numberGuest++;
                JustinaVision::startSkeletonFinding();
                break;

            case SM_FINAL_STATE:
                std::cout << "Farewell Test...-> SM_FINAL_STATE" << std::endl;
                JustinaManip::hdGoTo(0.0, 0.0, 2000);
                JustinaHRI::say("I have finished the test");
                ros::Duration(1.0).sleep();
                success=true;
                break;
        }


        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
