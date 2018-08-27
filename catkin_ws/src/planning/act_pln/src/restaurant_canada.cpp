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
#include "justina_tools/JustinaRepresentation.h"
#include "std_msgs/Bool.h"
#include "string"
#include "vision_msgs/FindPlane.h"

#define SM_INIT 0
#define SM_WAIT_FOR_INIT_COMMAND 10
#define SM_SEARCH_BAR 20
#define SM_SEARCH_WAVING 30
#define SM_ALIGN_WAVING 32
#define SM_FIND_PERSONS 33
#define SM_WAIT_FOR_TAKE_ORDER 35
#define SM_CLOSE_TO_CLIENT 36
#define SM_SAY_TYPE_ORDER 40
#define SM_TYPE_ORDER_CONFIRM 41
#define SM_TAKE_ORDER 45
#define SM_ORDER_CONFIRM 50
#define SM_RETURN_BAR 60
#define SM_REPETE_ORDER 65
#define SM_WAIT_TO_REPETE_ORDER 70
#define SM_WAIT_TO_PUT_ORDER 72
#define SM_WAIT_OBJECT 75
#define SM_GRASP_OBJECT 80
#define SM_NAVIGATE_TABLE 85
#define SM_DELIVER_OBJECT 90
#define SM_FINISH_TEST 150

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING RESTAURANT TEST..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaKnowledge::setNodeHandle(&n);
    JustinaRepresentation::setNodeHandle(&n);

    bool hdMotion = false;

    if(ros::param::has("~hdMotion"))
        ros::param::get("~hdMotion", hdMotion);

    std::cout << "restaurant_canada_node.-> Hd motion:" << hdMotion << std::endl;
   
    ros::Rate loop(10);
    
    int nextState = 0;
    bool fail = false;
    bool success = false;
    bool stop=false;
    bool findGesture = false;
    bool reachedGoal = false;

    std::string lastRecoSpeech;
    std::string lastInteSpeech;
    int timeoutspeech = 10000;
    std::string obj1, obj2, obj1C, obj2C, typeOrder;

    bool isTableLeft = false;
    float robot_y, robot_x, robot_a;    
    float gx_w, gy_w, gz_w;    
    float goalx, goaly, goala, angleError;    
    float dist_to_head;

    std::vector<std::string> attendCommands;
    attendCommands.push_back("justina take the order");
    attendCommands.push_back("justina wait");
    std::vector<std::string> confirmCommands;
    confirmCommands.push_back("justina yes");
    confirmCommands.push_back("justina no");
    std::vector<std::string> typeOrderCommands;
    confirmCommands.push_back("I want a drink");
    confirmCommands.push_back("I want a combo");

    std::stringstream ss;
    std::stringstream ss2;

    Eigen::Vector3d centroidGesture;

    bool findGestureOrAttendOrder = false;
    bool validateCombo = true;
    bool isCombo = false;
    int numberTable = 0;
    int maxNumberTable = 2;

    int attempsNavigation = 1;
    int maxAttempsNavigation = 2;

    int attempsSpeechReco = 1;
    int maxAttempsSpeechReco = 4;
    int attempsSpeechInt = 1;
    int maxAttempsSpeechInt = 2;
    int attempsWaitConfirmation = 1;
    int maxAttempsWaitConfirmation = 3;
    int attempsConfirmation = 1;
    int maxAttempsConfirmation = 3;
    int countRepetOrder = 1;
    int maxCountRepetOrder = 3;
    int attempsWaitToPutOrder = 1; 
    int maxAttempsWaitToPutOrder = 2;
    bool alignWithTable = false;
    int attempsGrasp = 1;
    int maxAttempsGrasp = 2;

    int minDelayAfterSay = 0;
    int maxDelayAfterSay = 300;
    
    geometry_msgs::Pose pose;
    std::vector<std::string> objsToGrasp;
    std::vector<std::string> objsToTake;
    bool armsFree[2] = {true, true};
    std::string idObject;
    bool withLeftOrRightArm;

    std::string bar_search="";
    
    std::vector<std::string> tokens;

    std::string grammarCommands = "restaurant_commands.xml";
    std::string grammarBeverage = "restaurant_beverage.xml";
    std::string grammarCombo = "restaurant_combo.xml";
    
    JustinaHRI::setInputDevice(JustinaHRI::KINECT);
    JustinaHRI::loadGrammarSpeechRecognized(grammarCommands);//load the grammar

    while(ros::ok() && !fail && !success){
        switch(nextState){
            case SM_INIT:
                std::cout << "State machine: SM_INIT" << std::endl;	
                JustinaManip::hdGoTo(0, 0, 2000);
                JustinaHRI::waitAfterSay("I'm ready for the restaurant test, tell me, justina start, to performing the test", timeoutspeech, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);
                nextState = SM_WAIT_FOR_INIT_COMMAND;
                break;

            case SM_WAIT_FOR_INIT_COMMAND:
                std::cout << "State machine: SM_WAIT_FOR_INIT_COMMAND" << std::endl;
				if(JustinaHRI::waitForSpecificSentence("justina start", timeoutspeech)){
                    JustinaHRI::enableSpeechRecognized(false);
                    JustinaHRI::waitAfterSay("I will search the bar", 3500, minDelayAfterSay);
                    nextState = SM_SEARCH_BAR;
                }else
                    nextState = SM_WAIT_FOR_INIT_COMMAND;
                break;

            case SM_SEARCH_BAR:
                std::cout << "State machine: SM_SERACH_BAR" << std::endl;
                JustinaTasks::findTable(bar_search, hdMotion);  
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                // TODO delimited the table to find
                if (bar_search.compare("center") == 0){
                    JustinaHRI::waitAfterSay("I see the bar in front of me", 10000, minDelayAfterSay);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    JustinaKnowledge::addUpdateKnownLoc("kitchen_bar", robot_a);
                }
                else if (bar_search.compare("right") == 0){
                    JustinaHRI::waitAfterSay("I see the bar in my right side", 10000, minDelayAfterSay);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    JustinaKnowledge::addUpdateKnownLoc("kitchen_bar", robot_a);
                    JustinaNavigation::startMoveDistAngle(0.0, M_PI_2);
                }else if (bar_search.compare("left") == 0){
                    JustinaHRI::waitAfterSay("I see the bar in my left side", 10000, minDelayAfterSay);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    JustinaKnowledge::addUpdateKnownLoc("kitchen_bar", robot_a);
                    JustinaNavigation::startMoveDistAngle(0.0, -M_PI_2);
                }else{
                    std::cout << "SM_SERACH_BAR: Bar default" << std::endl;
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                        JustinaKnowledge::addUpdateKnownLoc("kitchen_bar", robot_a);
                        JustinaHRI::waitAfterSay("I see the bar in my left side", 10000, minDelayAfterSay);
                    }
                findGestureOrAttendOrder = true;
                numberTable = 1;
                JustinaHRI::waitAfterSay("I will find a customer", 5000, minDelayAfterSay);
                JustinaVision::startSkeletonFinding();
                nextState = SM_SEARCH_WAVING;     
                break;

            case SM_SEARCH_WAVING:
                std::cout << "State machine: SM_SEARCH_WAVING" << std::endl;
                findGesture = JustinaTasks::turnAndRecognizeGesture("waving", -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2, -0.2, -0.2, 0.0, 0.0f, 9.0, centroidGesture, "", true);
                // findGesture = JustinaTasks::turnAndRecognizeGesture("waving", 0, 0, 0, -0.2f, -0.2f, -0.2f, 0.0f, 0.0f, 9.0, centroidGesture, "", true);
                if(findGesture){
                    JustinaVision::stopSkeletonFinding();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                    ros::spinOnce();
                   
                    JustinaTools::transformPoint("/base_link", centroidGesture(0, 0), centroidGesture(1, 0) , centroidGesture(2, 0), "/map", gx_w, gy_w, gz_w);

                    if (bar_search.compare("left") == 0)
                        JustinaNavigation::moveDistAngle(0.0, M_PI_2, 3000);
                    else if (bar_search.compare("right") == 0)
                        JustinaNavigation::moveDistAngle(0.0, -M_PI_2, 3000);
                    else
                        JustinaNavigation::moveDistAngle(0.0, M_PI_2, 3000);

                    JustinaManip::hdGoTo(0.0, 0.0, 1000);
 
                    ss.str("");
                    ss << "I noticed that somebody are asking for my service " << std::endl;

                    if(centroidGesture(1, 0) > -0.4 && centroidGesture(1, 0) < 0.4)
                        ss << "in front of me";
                    else if(centroidGesture(1, 0) > 0.4)
                        ss << "in my left side";
                    else if(centroidGesture(1, 0) < -0.4)
                        ss << "in my right side";

                    JustinaHRI::waitAfterSay(ss.str(), 5000, minDelayAfterSay);
                    JustinaHRI::waitAfterSay("Tell me justina take the order for confirmation", 5000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);
                    nextState = SM_WAIT_FOR_TAKE_ORDER;
                }else
                    nextState = SM_SEARCH_WAVING;
                break;

            case SM_WAIT_FOR_TAKE_ORDER:
                std::cout << "State machine: SM_WAIT_FOR_TAKE_ORDER" << std::endl;
                if(JustinaHRI::waitForSpecificSentence(attendCommands, lastRecoSpeech, timeoutspeech)){
                    if(lastRecoSpeech.find("take the order") != std::string::npos){
                        JustinaHRI::waitAfterSay("Ok, I am going to approach to my customer", 6000, minDelayAfterSay);
                        ss.str("");
                        ss << "table_" << numberTable;
                        JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                        JustinaKnowledge::addUpdateKnownLoc(ss.str(), gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                        nextState = SM_CLOSE_TO_CLIENT;
                    }
                    else if(lastRecoSpeech.find("wait") != std::string::npos){
                        if (bar_search.compare("left") == 0)
                            JustinaNavigation::startMoveDistAngle(0.0, -M_PI_2);
                        else if (bar_search.compare("right") == 0)
                            JustinaNavigation::startMoveDistAngle(0.0, M_PI_2);
                        else
                            JustinaNavigation::startMoveDistAngle(0.0, -M_PI_2);
                        JustinaHRI::waitAfterSay("I will find to a another customer", 5000, minDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaVision::startSkeletonFinding();
                        nextState = SM_SEARCH_WAVING;
                    }
                }
                else{
                    JustinaHRI::enableSpeechRecognized(false);
                    JustinaHRI::waitAfterSay("I noticed that somebody are asking for my service", 5000, minDelayAfterSay);
                    JustinaHRI::waitAfterSay("Tell me justina take the order for confirmation", 5000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);
                }
                break;

            case SM_CLOSE_TO_CLIENT:
                std::cout << "State machine: SM_CLOSE_TO_CLIENT" << std::endl;
                ss.str("");
                ss << "table_" << numberTable;
                JustinaKnowledge::getKnownLocation(ss.str(), goalx, goaly, goala);
                std::cout << "restaruant_canada.->Centroid gesture:" << goalx << "," << goaly << "," << goala << std::endl;
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
                 
                JustinaHRI::waitAfterSay("Hello my name is Justina, and for today I will take your order", 12000);
                attempsConfirmation = 1;
                attempsWaitConfirmation = 1;
                attempsSpeechReco = 1;
                attempsSpeechInt = 1;
                validateCombo = false;
                nextState = SM_SAY_TYPE_ORDER;
                break;

            case SM_SAY_TYPE_ORDER:
                std::cout << "State machine: SM_TAKE_TYPE_ORDER" << std::endl;
                JustinaHRI::enableSpeechRecognized(false);
                if(validateCombo)
                    JustinaHRI::waitAfterSay("Do you want a combo, please tell me justina yes or justina no", 10000, maxDelayAfterSay);
                else 
                    JustinaHRI::waitAfterSay("Do you want a beverage, please tell me justina yes or justina no", 10000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);
                nextState = SM_TYPE_ORDER_CONFIRM;
                break;

            case SM_TYPE_ORDER_CONFIRM:
                std::cout << "State machine: SM_TYPE_ORDER_CONFIRM" << std::endl;
                if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, timeoutspeech)){
                    if(lastRecoSpeech.find("yes") != std::string::npos){
                        attempsSpeechReco = 1;
                        attempsSpeechInt = 1;
                        attempsConfirmation = 1;
                        attempsWaitConfirmation = 1;
                        isCombo = validateCombo;
                        JustinaHRI::enableSpeechRecognized(false);
                        if(isCombo){
                            JustinaHRI::waitAfterSay("Please tell me what combo, do you want, for example, i want a pringles and cereal", 5000, maxDelayAfterSay);
                            JustinaHRI::loadGrammarSpeechRecognized(grammarCombo);
                        }else{
                            JustinaHRI::waitAfterSay("Please tell me wich beverage, do you want, for example, i want a sprite", 5000, maxDelayAfterSay);
                            JustinaHRI::loadGrammarSpeechRecognized(grammarBeverage);
                        }
                        JustinaHRI::enableSpeechRecognized(true);
                        nextState = SM_TAKE_ORDER;
                    }
                    else{
                        if(attempsConfirmation <= maxAttempsConfirmation){
                            attempsConfirmation++;
                            attempsWaitConfirmation = 1;
                            nextState = SM_SAY_TYPE_ORDER;
                        }
                        else{
                            attempsSpeechReco = 1;
                            attempsSpeechInt = 1;
                            attempsConfirmation = 1;
                            attempsWaitConfirmation = 1;
                            isCombo = validateCombo;
                            JustinaHRI::enableSpeechRecognized(false);
                            if(isCombo){
                                JustinaHRI::waitAfterSay("Please tell me what combo, do you want, for example, i want a pringles and cereal", 5000, maxDelayAfterSay);
                                JustinaHRI::loadGrammarSpeechRecognized(grammarCombo);
                            }else{
                                JustinaHRI::waitAfterSay("Please tell me wich beverage, do you want, for example, i want a sprite", 5000, maxDelayAfterSay);
                                JustinaHRI::loadGrammarSpeechRecognized(grammarBeverage);
                            }
                            JustinaHRI::enableSpeechRecognized(true);
                            nextState = SM_TAKE_ORDER;
                        }
                        if(validateCombo)
                            validateCombo = false;
                        else
                            validateCombo = true;
                    }
                }
                else {
                    if(attempsWaitConfirmation <= maxAttempsWaitConfirmation){
                        attempsWaitConfirmation++;
                        nextState = SM_SAY_TYPE_ORDER;
                    }
                    else{
                        attempsSpeechReco = 1;
                        attempsSpeechInt = 1;
                        attempsConfirmation = 1;
                        attempsWaitConfirmation = 1;
                        isCombo = validateCombo;
                        JustinaHRI::enableSpeechRecognized(false);
                        if(isCombo){
                            JustinaHRI::waitAfterSay("Please tell me what combo, do you want, for example, i want a pringles and cereal", 5000, maxDelayAfterSay);
                            JustinaHRI::loadGrammarSpeechRecognized(grammarCombo);
                        }else{
                            JustinaHRI::waitAfterSay("Please tell me wich beverage, do you want, for example, i want a sprite", 5000, maxDelayAfterSay);
                            JustinaHRI::loadGrammarSpeechRecognized(grammarBeverage);
                        }
                        nextState = SM_TAKE_ORDER;
                    }
                }
                break;

            case SM_TAKE_ORDER:
                std::cout << "State machine: SM_TAKE_ORDER" << std::endl;
                if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeoutspeech)){
                    if(JustinaRepresentation::stringInterpretation(lastRecoSpeech, lastInteSpeech)){
                        if(JustinaRepresentation::orderInterpeted(lastInteSpeech, typeOrder, obj1, obj2)){
                            // TODO Review what happend with object with two words
                            ss.str("");
                            ss << "Do you want ";
                            if(typeOrder.compare("take_order_beverage") == 0){
                                tokens.clear();
                                boost::algorithm::split(tokens, obj1, boost::algorithm::is_any_of("_"));
                                ss2.str("");
                                for(int i = 0; i < tokens.size(); i++){
                                    ss << tokens[i] << " ";
                                    ss2 << tokens[i];
                                    if(i < tokens.size() -1)
                                        ss2 << " ";
                                }
                                obj1C = ss2.str();
                                isCombo = false;
                            }else if(typeOrder.compare("take_order_combo") == 0){
                                tokens.clear();
                                if(obj1.compare(obj2) == 0 || obj2.compare(" ") == 0 || obj2.compare("") == 0){
                                    JustinaHRI::enableSpeechRecognized(false);
                                    if(isCombo){
                                        JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what combo, do you want", 5000, maxDelayAfterSay);
                                        //JustinaHRI::loadGrammarSpeechRecognized(grammarCombo);
                                    }else{
                                        JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me wich beverage, do you want", 5000, maxDelayAfterSay);
                                        //JustinaHRI::loadGrammarSpeechRecognized(grammarBeverage);
                                    }
                                    attempsSpeechInt++;
                                    JustinaHRI::enableSpeechRecognized(true);
                                    nextState = SM_TAKE_ORDER;
                                }

                                boost::algorithm::split(tokens, obj1, boost::algorithm::is_any_of("_"));
                                ss2.str("");
                                for(int i = 0; i < tokens.size(); i++){
                                    ss << tokens[i] << " ";
                                    ss2 << tokens[i];
                                    if(i < tokens.size() -1)
                                        ss2 << " ";
                                }
                                obj1C = ss2.str();
                                ss << "and ";
                                tokens.clear();
                                boost::algorithm::split(tokens, obj2, boost::algorithm::is_any_of("_"));
                                ss2.str("");
                                for(int i = 0; i < tokens.size(); i++){
                                    ss << tokens[i] << " ";
                                    ss2 << tokens[i];
                                    if(i < tokens.size() -1)
                                        ss2 << " ";
                                }
                                obj2C = ss2.str();
                                isCombo = true;
                            }
                            ss << ", please tell me justina yes or justina no";
                            JustinaHRI::enableSpeechRecognized(false);
                            JustinaHRI::waitAfterSay(ss.str(), 10000, maxDelayAfterSay);
                            JustinaHRI::loadGrammarSpeechRecognized(grammarCommands);
                            JustinaHRI::enableSpeechRecognized(true);
                            nextState = SM_ORDER_CONFIRM;
                            break;
                        }
                    }
                    if(maxAttempsSpeechInt <= maxAttempsSpeechInt){
                        JustinaHRI::enableSpeechRecognized(false);
                        if(isCombo){
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what combo, do you want", 5000, maxDelayAfterSay);
                            //JustinaHRI::loadGrammarSpeechRecognized(grammarCombo);
                        }else{
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me wich beverage, do you want", 5000, maxDelayAfterSay);
                            //JustinaHRI::loadGrammarSpeechRecognized(grammarBeverage);
                        }
                        attempsSpeechInt++;
                        JustinaHRI::enableSpeechRecognized(true);
                        nextState = SM_TAKE_ORDER;
                    }
                    else{
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay("Sorry I did not understand you, I'm going back to the kitchen bar", 12000, minDelayAfterSay);
                        attempsSpeechReco = 1;
                        attempsSpeechInt = 1;
                        findGestureOrAttendOrder = true;
                        nextState = SM_RETURN_BAR;
                    }
                }
                else{
                    if(attempsSpeechReco <= maxAttempsSpeechReco){
                        JustinaHRI::enableSpeechRecognized(false);
                        if(isCombo){
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what combo, do you want", 5000, maxDelayAfterSay);
                            //JustinaHRI::loadGrammarSpeechRecognized(grammarCombo);
                        }else{
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me witch beverage, do you want", 5000, maxDelayAfterSay);
                            //JustinaHRI::loadGrammarSpeechRecognized(grammarBeverage);
                        }
                        attempsSpeechReco++;
                        JustinaHRI::enableSpeechRecognized(true);
                        nextState = SM_TAKE_ORDER;
                    }
                    else{
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay("Sorry I did not understand you, I'm going back to the kitchen bar", 12000, minDelayAfterSay);
                        attempsSpeechReco = 1;
                        attempsSpeechInt = 1;
                        findGestureOrAttendOrder = true;
                        nextState = SM_RETURN_BAR;
                    }
                }
                break;

            case SM_ORDER_CONFIRM:
                std::cout << "State machine: SM_FIRST_ORDER_CONFIRM" << std::endl;
                attempsSpeechReco = 1;
                attempsSpeechInt = 1;
                if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, timeoutspeech)){
                    if(lastRecoSpeech.find("yes") != std::string::npos){
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000, minDelayAfterSay);
                        findGestureOrAttendOrder = false;
                        nextState = SM_RETURN_BAR;
                    }
                    else{
                        if(attempsConfirmation <= maxAttempsConfirmation){
                            attempsConfirmation++;
                            JustinaHRI::enableSpeechRecognized(false);
                            if(isCombo){
                                JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what combo, do you want", 5000, minDelayAfterSay);
                                JustinaHRI::loadGrammarSpeechRecognized(grammarCombo);
                            }else{
                                JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me wich beverage, do you want", 5000, minDelayAfterSay);
                                JustinaHRI::loadGrammarSpeechRecognized(grammarBeverage);
                            }
                            JustinaHRI::enableSpeechRecognized(true);
                            nextState = SM_TAKE_ORDER;
                        }
                        else{
                            JustinaHRI::enableSpeechRecognized(false);
                            JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000, minDelayAfterSay);
                            findGestureOrAttendOrder = false;
                            nextState = SM_RETURN_BAR;
                        }
                    }
                }
                else {
                    if(attempsWaitConfirmation <= maxAttempsWaitConfirmation){
                        attempsWaitConfirmation++;
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay(ss.str(), 10000);
                        JustinaHRI::enableSpeechRecognized(true);
                        nextState = SM_ORDER_CONFIRM;
                    }
                    else{
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000, minDelayAfterSay);
                        findGestureOrAttendOrder = false;
                        nextState = SM_RETURN_BAR;
                    }
                }
                break;	

            case SM_RETURN_BAR:
                std::cout << "State machine: SM_RETURN_BAR" << std::endl;
                JustinaHRI::loadGrammarSpeechRecognized(grammarCommands);//load the grammar
                JustinaHRI::enableSpeechRecognized(false);
                if(!JustinaNavigation::getClose("kitchen_bar", 120000)){
                    attempsNavigation++;
                    if(attempsNavigation <= maxAttempsNavigation)
                        break;
                    else{
                        JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                        JustinaKnowledge::getKnownLocation("kitchen_bar", goalx, goaly, goala);
                        angleError = goala - robot_a;
                        if(angleError > M_PI) angleError -= 2*M_PI;
                        if(angleError <= -M_PI) angleError += 2*M_PI;
                        JustinaNavigation::moveDistAngle(0.0, angleError, 3000);
                    }
                }
                if(numberTable <= maxNumberTable){
                    JustinaHRI::waitAfterSay("I arrived to the kitchen bar", 2000, minDelayAfterSay);
                    if(findGestureOrAttendOrder){
                        if (bar_search.compare("left") == 0)
                            JustinaNavigation::startMoveDistAngle(0.0, -M_PI_2);
                        else if (bar_search.compare("right") == 0)
                            JustinaNavigation::startMoveDistAngle(0.0, M_PI_2);
                        else
                            JustinaNavigation::startMoveDistAngle(0.0, -M_PI_2);
                        JustinaHRI::waitAfterSay("I will find to the client", 5000, minDelayAfterSay);
                        JustinaVision::startSkeletonFinding();
                        nextState=SM_SEARCH_WAVING;
                    }
                    else{
                        countRepetOrder = 1;
                        nextState = SM_REPETE_ORDER;
                    }
                }
                else
                    nextState = SM_FINISH_TEST;
                break;
            case SM_REPETE_ORDER:
                std::cout << "State machine: SM_REPETE_ORDER" << std::endl;
                ss.str("");
                ss << "Hey barman, I need a " << obj1C;
                if(isCombo)
                    ss << " and " << obj2C;
                ss << ", for the table " << numberTable;
                JustinaHRI::waitAfterSay(ss.str(), 5000, minDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(false);
                JustinaHRI::waitAfterSay("you understood the order, tell me justina yes", 5000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);
                countRepetOrder++;
                nextState = SM_WAIT_TO_REPETE_ORDER;
                break;
            case SM_WAIT_TO_REPETE_ORDER:
                std::cout << "State machine: SM_WAIT_FOR_REPETE_ORDER" << std::endl;
                if(countRepetOrder <= maxCountRepetOrder){
                    if(JustinaHRI::waitForSpecificSentence("justina yes", timeoutspeech)){
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay("please put the order in the kitchen bar and tell me, justina this is the order, when the order is ready", 10000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);
                        attempsWaitToPutOrder = 1;
                        nextState = SM_WAIT_TO_PUT_ORDER;
                    }else
                        nextState = SM_REPETE_ORDER;
                }
                else{
                    JustinaHRI::enableSpeechRecognized(false);
                    JustinaHRI::waitAfterSay("please put the order in the kitchen bar and tell me, justina this is the order, when the order is ready", 10000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);
                    attempsWaitToPutOrder = 1;
                    nextState = SM_WAIT_TO_PUT_ORDER;
                }
                break;
            case SM_WAIT_TO_PUT_ORDER:
                std::cout << "State machine: SM_WAIT_TO_PUT_ORDER" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("justina this is the order", timeoutspeech)){
                    JustinaHRI::enableSpeechRecognized(false);
                    JustinaHRI::waitAfterSay("I attemp to take the order", 10000, minDelayAfterSay);
                    if(isCombo){
                        objsToTake.push_back(obj1);
                        objsToTake.push_back(obj2);
                        armsFree[0] = true;
                        armsFree[1] = true;
                    }else{
                        objsToTake.push_back(obj1);
                        objsToGrasp.push_back(obj1);
                        armsFree[0] = true;
                        armsFree[1] = true;
                    }
                    alignWithTable = false;
                    attempsGrasp = 1;
                    nextState = SM_GRASP_OBJECT;
                }
                else{
                    if(attempsWaitToPutOrder <= maxAttempsWaitToPutOrder){
                        attempsWaitToPutOrder++;
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay("please put the order in the kitchen bar and tell me, justina this is the order, when the order is ready", 10000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);
                        nextState = SM_WAIT_TO_PUT_ORDER;
                    }
                    else{
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay("I attemp to grasp the order", 10000, minDelayAfterSay);
                        if(isCombo){
                            objsToTake.push_back(obj1);
                            objsToTake.push_back(obj2);
                            armsFree[0] = true;
                            armsFree[1] = true;
                        }else{
                            objsToTake.push_back(obj1);
                            objsToGrasp.push_back(obj1);
                            armsFree[0] = true;
                            armsFree[1] = true;
                        }
                        alignWithTable = false;
                        attempsGrasp = 1;
                        nextState = SM_GRASP_OBJECT;
                    }
                }
                break;
            case SM_GRASP_OBJECT:
                std::cout << "State machine: SM_GRASP_OBJECT" << std::endl;
                if(objsToGrasp.size() > 0){
                    idObject = objsToGrasp[0];
                    if(!alignWithTable && !JustinaTasks::alignWithTable(0.35)){
                        std::cout << "I can´t align with table   :´(" << std::endl;
                        JustinaNavigation::moveDistAngle(-0.05, M_PI_4/4, 2000);
                        JustinaTasks::alignWithTable(0.35);
                        JustinaTasks::alignWithTable(0.35);
                    }
                    alignWithTable = true;
                    if(attempsGrasp <= maxAttempsGrasp){
                        attempsGrasp++;
                        if(JustinaTasks::findObject(idObject, pose, withLeftOrRightArm)){
                            if(!(withLeftOrRightArm && armsFree[1]))
                                withLeftOrRightArm = false;
                            else if(!(!withLeftOrRightArm && armsFree[0]))
                                withLeftOrRightArm = true;
                            if(JustinaTasks::moveActuatorToGrasp(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, idObject)){
                                objsToGrasp.erase(objsToGrasp.begin());
                                objsToTake.erase(objsToTake.begin());
                                if(withLeftOrRightArm)
                                    armsFree[1] = false;
                                else  
                                    armsFree[0] = false;
                            }
                        }
                    }
                    else{
                        alignWithTable = false;
                        attempsGrasp = 1;
                        nextState = SM_WAIT_OBJECT;
                    }
                }
                else{
                    alignWithTable = false;
                    attempsGrasp = 1;
                    nextState = SM_WAIT_OBJECT;
                }

                break;
            case SM_WAIT_OBJECT:
                std::cout << "State machine: SM_WAIT_OBJECT" << std::endl;
                if(objsToTake.size() > 0){
                    idObject = objsToTake[0];
                    ss.str("");
                    ss << "I can not take the " << idObject << ", but i will take the " << idObject << " if you put it in my gripper";
                    JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
                    if(armsFree[0]){
                        JustinaManip::raGoTo("navigation", 3000);
                        JustinaTasks::detectObjectInGripper(idObject, false, 7000);
                        armsFree[0] = false;
                    }else if(armsFree[1]){
                        JustinaManip::laGoTo("navigation", 3000);
                        JustinaTasks::detectObjectInGripper(idObject, true, 7000);
                        armsFree[1] = false;
                    }
                    objsToTake.erase(objsToTake.begin());
                    std::vector<std::string>::iterator it = std::find(objsToGrasp.begin(), objsToGrasp.end(), idObject);
                    if(it != objsToGrasp.end())
                        objsToGrasp.erase(it);
                    nextState = SM_WAIT_OBJECT;
                    if(objsToGrasp.size() > 0)
                        nextState = SM_GRASP_OBJECT;
                }
                else{
                    JustinaHRI::waitAfterSay("thanks barman", 2000, minDelayAfterSay);
                    attempsNavigation = 1;
                    nextState = SM_NAVIGATE_TABLE;
                }
                break;
            case SM_NAVIGATE_TABLE:
                std::cout << "State machine: SM_NAVIGATE_TABLE" << std::endl;
                ss.str("");
                ss << "table_" << numberTable;
                JustinaHRI::enableSpeechRecognized(false);
                if(!JustinaNavigation::getClose(ss.str(), 120000)){
                    attempsNavigation++;
                    if(attempsNavigation <= maxAttempsNavigation)
                        break;
                    else{
                        JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                        JustinaKnowledge::getKnownLocation("kitchen_bar", goalx, goaly, goala);
                        angleError = goala - robot_a;
                        if(angleError > M_PI) angleError -= 2*M_PI;
                        if(angleError <= -M_PI) angleError += 2*M_PI;
                        JustinaNavigation::moveDistAngle(0.0, angleError, 3000);
                    }
                }
                ss.str("");
                ss << "I arrived to the table_" << numberTable;
                JustinaHRI::waitAfterSay(ss.str(), 2000, minDelayAfterSay);
                nextState = SM_DELIVER_OBJECT;
                break;
            case SM_DELIVER_OBJECT:
                std::cout << "State machine: SM_DELIVER_OBJECT" << std::endl;
                if(armsFree[0] && armsFree[1]){
                    JustinaHRI::waitAfterSay("Human enjoy it, thanks for your preference", 4000, minDelayAfterSay);
                    attempsNavigation = 1;
                    findGestureOrAttendOrder = true;
                    numberTable++;
                    nextState = SM_RETURN_BAR; 
                }
                else{
                    if(!armsFree[0]){
                        JustinaManip::raGoTo("navigation", 3000);
                        JustinaTasks::dropObject("", false, 10000);
                        armsFree[0] = true;
                    }
                    else if(!armsFree[1]){
                        JustinaManip::laGoTo("navigation", 3000);
                        JustinaTasks::dropObject("", true, 10000);
                        armsFree[1] = true;
                    }
                }
                break;
            case SM_FINISH_TEST:
                JustinaHRI::waitAfterSay("I have finished the test", 5000);
                success = true;
                break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 1;
}

