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
#define SM_WAITING_FOR_MISSING_ORDER 55
#define SM_RETURN_BAR 60
#define SM_REPETE_ORDER 65
#define SM_WAIT_TO_REPETE_ORDER 70
#define SM_WAIT_TO_PUT_ORDER 72
#define SM_WAIT_OBJECT 75
#define SM_GRASP_OBJECT 80
#define SM_NAVIGATE_TABLE 85
#define SM_GUIDE_TABLE 87
#define SM_DELIVER_OBJECT 90
#define SM_WAIT_FOR_GUIDE_BARMAN 95
#define SM_VERIFY_OBJECT 98
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
    std::vector<std::string> orderItems;
    std::vector<std::string> orderItemsSp;
    std::vector<std::string> typeItems;

    std::stringstream ss;
    std::stringstream ss2;

    std::vector<Eigen::Vector3d> centroidGestures;

    bool findGestureOrAttendOrder = false;
    bool validateFood = true;
    bool isFood = false;
    int numberTable = 0;
    int maxNumberTable = 3;

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
    std::vector<std::string> objsToGuide;
    std::vector<vision_msgs::VisionObject> recognizedObjects;
    // First is of the right arm, Second is  of the left arm and third is of the guiding object
    std::string objsToDeliv[3] = {"", "", ""};
    bool armsFree[2] = {true, true};
    std::string idObject;
    bool withLeftOrRightArm;

    std::string bar_search="";
    
    std::vector<std::string> tokens;

    std::string grammarCommands = "restaurant_commands.xml";
    std::string grammarBeverage = "restaurant_beverage.xml";
    //std::string grammarCombo = "restaurant_combo.xml";
    std::string grammarFood = "restaurant_food.xml";
    
    //JustinaHRI::setInputDevice(JustinaHRI::KINECT);
    JustinaHRI::loadGrammarSpeechRecognized(grammarCommands);//load the grammar

    while(ros::ok() && !fail && !success){
        switch(nextState){
            case SM_INIT:
                std::cout << "State machine: SM_INIT" << std::endl;	
                JustinaManip::hdGoTo(0, 0, 2000);
                JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
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
                centroidGestures = std::vector<Eigen::Vector3d>();
                objsToTake = std::vector<std::string>();
                objsToGrasp = std::vector<std::string>();
                objsToGuide = std::vector<std::string>();
                objsToDeliv[0] = "";
                objsToDeliv[1] = "";
                objsToDeliv[2] = "";
                armsFree[0] = 0;
                armsFree[1] = 0;
                findGesture = JustinaTasks::turnAndRecognizeGesture("waving", -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2, -0.2, -0.2, 0.1, 0.1f, 9.0, centroidGestures, "", true);
                // findGesture = JustinaTasks::turnAndRecognizeGesture("waving", 0, 0, 0, -0.2f, -0.2f, -0.2f, 0.0f, 0.0f, 9.0, centroidGesture, "", true);
                if(findGesture){
                    JustinaVision::stopSkeletonFinding();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                    ros::spinOnce();
                   
                    JustinaTools::transformPoint("/base_link", centroidGestures[0](0, 0), centroidGestures[0](1, 0) , centroidGestures[0](2, 0), "/map", gx_w, gy_w, gz_w);

                    if (bar_search.compare("left") == 0)
                        JustinaNavigation::moveDistAngle(0.0, M_PI_2, 3000);
                    else if (bar_search.compare("right") == 0)
                        JustinaNavigation::moveDistAngle(0.0, -M_PI_2, 3000);
                    else
                        JustinaNavigation::moveDistAngle(0.0, M_PI_2, 3000);

                    JustinaManip::hdGoTo(0.0, 0.0, 1000);
 
                    ss.str("");
                    ss << "I noticed that somebody are asking for my service " << std::endl;

                    if(centroidGestures[0](1, 0) > -0.4 && centroidGestures[0](1, 0) < 0.4)
                        ss << "in front of me";
                    else if(centroidGestures[0](1, 0) > 0.4)
                        ss << "in my left side";
                    else if(centroidGestures[0](1, 0) < -0.4)
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
                JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
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

                JustinaManip::startHdGoTo(atan2(goaly - robot_y, goalx - robot_x) - robot_a, atan2(gz_w - (1.53 + torsoSpine), dist_to_head));
                // *JustinaManip::startHdGoTo(0, atan2(gz_w - 1.6, dist_to_head));
                 
                JustinaHRI::waitAfterSay("Hello my name is Justina, and for today I will take your order", 12000);
                attempsConfirmation = 1;
                attempsWaitConfirmation = 1;
                attempsSpeechReco = 1;
                attempsSpeechInt = 1;
                validateFood = false;
                orderItems = std::vector<std::string>();
                orderItemsSp = std::vector<std::string>();
                nextState = SM_SAY_TYPE_ORDER;
                break;

            case SM_SAY_TYPE_ORDER:
                std::cout << "State machine: SM_TAKE_TYPE_ORDER" << std::endl;
                JustinaHRI::enableSpeechRecognized(false);
                if(validateFood){
                    //JustinaHRI::waitAfterSay("Do you want a food, say justina yes or justina no", 10000, maxDelayAfterSay);
                    JustinaHRI::waitAfterSay("Do you want some food, say justina yes or justina no", 10000, maxDelayAfterSay);
                }else{
                    //JustinaHRI::waitAfterSay("Do you want a beverage, please tell me justina yes or justina no", 10000, maxDelayAfterSay);
                    JustinaHRI::waitAfterSay("Do you want some drink, say justina yes or justina no", 10000, maxDelayAfterSay);
                }
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
                        isFood = validateFood;
                        JustinaHRI::enableSpeechRecognized(false);
                        if(isFood){
                            //JustinaHRI::waitAfterSay("Please tell me what combo, do you want, for example, i want a pringles and cereal", 5000, maxDelayAfterSay);
                            JustinaHRI::waitAfterSay("Please tell me which food, do you want, for example, i want a pringles", 7000, maxDelayAfterSay);
                            JustinaHRI::loadGrammarSpeechRecognized(grammarFood);
                        }else{
                            JustinaHRI::waitAfterSay("Please tell me which beverage, do you want, for example, i want a sprite", 7000, maxDelayAfterSay);
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
                            isFood = validateFood;
                            JustinaHRI::enableSpeechRecognized(false);
                            if(isFood){
                                JustinaHRI::waitAfterSay("Please tell me which food, do you want, for example, i want a pringles", 5000, maxDelayAfterSay);
                                JustinaHRI::loadGrammarSpeechRecognized(grammarFood);
                            }else{
                                JustinaHRI::waitAfterSay("Please tell me which beverage, do you want, for example, i want a sprite", 5000, maxDelayAfterSay);
                                JustinaHRI::loadGrammarSpeechRecognized(grammarBeverage);
                            }
                            JustinaHRI::enableSpeechRecognized(true);
                            nextState = SM_TAKE_ORDER;
                        }
                        if(!validateFood)
                            validateFood = true;
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
                        isFood = validateFood;
                        JustinaHRI::enableSpeechRecognized(false);
                        if(isFood){
                            JustinaHRI::waitAfterSay("Please tell me which food, do you want, for example, i want a pringles", 5000, maxDelayAfterSay);
                            JustinaHRI::loadGrammarSpeechRecognized(grammarFood);
                        }else{
                            JustinaHRI::waitAfterSay("Please tell me which beverage, do you want, for example, i want a sprite", 5000, maxDelayAfterSay);
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
                                // TODO REVIEW REY COMMENT
                                //isFood = false;
                            }else if(typeOrder.compare("take_order_combo") == 0){
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
                            }
                            ss << ", say justina yes or justina no";
                            if(isFood)
                                typeItems.push_back("food");
                            else
                                typeItems.push_back("drinks");
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
                        if(isFood){
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me which food, do you want", 5000, maxDelayAfterSay);
                            //JustinaHRI::loadGrammarSpeechRecognized(grammarFood);
                        }else{
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me which beverage, do you want", 5000, maxDelayAfterSay);
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
                        if(isFood){
                            JustinaHRI::waitAfterSay("Please tell me which food, do you want", 5000, maxDelayAfterSay);
                            //JustinaHRI::loadGrammarSpeechRecognized(grammarFood);
                        }else{
                            JustinaHRI::waitAfterSay("Please tell me witch beverage, do you want", 5000, maxDelayAfterSay);
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
                        //JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000, minDelayAfterSay);
                        orderItems.push_back(obj1);
                        orderItemsSp.push_back(obj1C);
                        if(orderItems.size() == 3){
                            JustinaHRI::enableSpeechRecognized(false);
                            JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000, minDelayAfterSay);
                            findGestureOrAttendOrder = false;
                            nextState = SM_RETURN_BAR;
                        }
                        else{
                            JustinaHRI::waitAfterSay("Ok, Do you want something else, say justina yes or justina no", 10000, maxDelayAfterSay);
                            //findGestureOrAttendOrder = false;
                            //nextState = SM_RETURN_BAR;
                            nextState = SM_WAITING_FOR_MISSING_ORDER;
                        }
                    }
                    else{
                        if(attempsConfirmation <= maxAttempsConfirmation){
                            attempsConfirmation++;
                            JustinaHRI::enableSpeechRecognized(false);
                            if(isFood){
                                JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me which food, do you want", 5000, minDelayAfterSay);
                                JustinaHRI::loadGrammarSpeechRecognized(grammarFood);
                            }else{
                                JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me which beverage, do you want", 5000, minDelayAfterSay);
                                JustinaHRI::loadGrammarSpeechRecognized(grammarBeverage);
                            }
                            nextState = SM_TAKE_ORDER;
                        }
                        else{
                            if(orderItems.size() == 3){
                                JustinaHRI::enableSpeechRecognized(false);
                                JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000, minDelayAfterSay);
                                findGestureOrAttendOrder = false;
                                nextState = SM_RETURN_BAR;
                            }
                            else{
                                JustinaHRI::enableSpeechRecognized(false);
                                //JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000, minDelayAfterSay);
                                JustinaHRI::waitAfterSay("Ok, Do you want something else, say justina yes or justina no", 10000, maxDelayAfterSay);
                                //findGestureOrAttendOrder = false;
                                //nextState = SM_RETURN_BAR;
                                nextState = SM_WAITING_FOR_MISSING_ORDER;
                            }
                        }
                    }
                }
                else {
                    if(attempsWaitConfirmation <= maxAttempsWaitConfirmation){
                        attempsWaitConfirmation++;
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay(ss.str(), 10000);
                        nextState = SM_ORDER_CONFIRM;
                    }
                    else{
                        if(orderItems.size() == 3){
                            JustinaHRI::enableSpeechRecognized(false);
                            JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000, minDelayAfterSay);
                            findGestureOrAttendOrder = false;
                            nextState = SM_RETURN_BAR;
                        }
                        else{
                            JustinaHRI::enableSpeechRecognized(false);
                            //JustinaHRI::waitAfterSay("Ok, i will go to the kitchen bar and i will be back with your order", 10000, minDelayAfterSay);
                            JustinaHRI::waitAfterSay("Ok, Do you want something else, say justina yes or justina no", 10000, maxDelayAfterSay);
                            //findGestureOrAttendOrder = false;
                            //nextState = SM_RETURN_BAR;
                            nextState = SM_WAITING_FOR_MISSING_ORDER;
                        }
                    }
                }
                JustinaHRI::enableSpeechRecognized(true);
                break;

            case SM_WAITING_FOR_MISSING_ORDER:
                std::cout << "State machine: SM_WAITING_FOR_MISSING_ORDER" << std::endl;
                if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, timeoutspeech)){
                    if(lastRecoSpeech.find("yes") != std::string::npos){
                        if(isFood){
                            //JustinaHRI::waitAfterSay("Please tell me what combo, do you want, for example, i want a pringles and cereal", 5000, maxDelayAfterSay);
                            JustinaHRI::waitAfterSay("Please tell me what food, do you want", 7000, maxDelayAfterSay);
                            JustinaHRI::loadGrammarSpeechRecognized(grammarFood);
                            JustinaHRI::enableSpeechRecognized(true);
                            nextState = SM_TAKE_ORDER;
                        }else{
                            //JustinaHRI::waitAfterSay("Please tell me which beverage, do you want", 7000, maxDelayAfterSay);
                            //JustinaHRI::loadGrammarSpeechRecognized(grammarBeverage);
                            nextState = SM_SAY_TYPE_ORDER;
                        }
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
                JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
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
                ss << "Hey barman, I need a ";
                for(int i = 0; i < orderItemsSp.size(); i++){
                    if(orderItemsSp.size() == 1){
                        ss << orderItemsSp[i] << ", ";
                    }
                    else{
                        if(i == orderItemsSp.size() - 2)
                            ss << orderItemsSp[i] << " and ";
                        else
                            ss << orderItemsSp[i] << ", ";
                    }
                }
                ss << "for the table " << numberTable;
                JustinaHRI::waitAfterSay(ss.str(), 5000, minDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(false);
                JustinaHRI::waitAfterSay("you understood the order, say justina yes", 5000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);
                countRepetOrder++;
                nextState = SM_WAIT_TO_REPETE_ORDER;
                break;
            case SM_WAIT_TO_REPETE_ORDER:
                std::cout << "State machine: SM_WAIT_FOR_REPETE_ORDER" << std::endl;
                if(countRepetOrder <= maxCountRepetOrder){
                    if(JustinaHRI::waitForSpecificSentence("justina yes", timeoutspeech)){
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay("please, put the order in front of me, on the kitchen bar and tell me, justina this is the order, when the order is ready", 10000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);
                        attempsWaitToPutOrder = 1;
                        nextState = SM_WAIT_TO_PUT_ORDER;
                    }else
                        nextState = SM_REPETE_ORDER;
                }
                else{
                    JustinaHRI::enableSpeechRecognized(false);
                    JustinaHRI::waitAfterSay("please, put the order in front of me, on the kitchen bar and tell me, justina this is the order, when the order is ready", 10000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);
                    attempsWaitToPutOrder = 1;
                    nextState = SM_WAIT_TO_PUT_ORDER;
                }
                break;
            case SM_WAIT_TO_PUT_ORDER:
                std::cout << "State machine: SM_WAIT_TO_PUT_ORDER" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("justina this is the order", timeoutspeech)){
                    JustinaHRI::enableSpeechRecognized(false);
                    JustinaHRI::waitAfterSay("I will attempt to grasp the order", 10000, minDelayAfterSay);
                    if(orderItems.size() > 1){
                        for(int i = 0; i < orderItems.size(); i++){
                            if(i < 2){
                                objsToTake.push_back(orderItems[i]);
                                objsToGrasp.push_back(orderItems[i]);
                            }
                            else
                                objsToGuide.push_back(orderItems[i]);
                        }
                        armsFree[0] = true;
                        armsFree[1] = true;
                    }else{
                        objsToTake.push_back(orderItems[0]);
                        objsToGrasp.push_back(orderItems[0]);
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
                        JustinaHRI::waitAfterSay("please, put the order in front of me, on the kitchen bar and tell me, justina this is the order, when the order is ready", 10000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);
                        nextState = SM_WAIT_TO_PUT_ORDER;
                    }
                    else{
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay("I will attempt to grasp the order", 10000, minDelayAfterSay);
                        if(orderItems.size() > 1){
                            for(int i = 0; i < orderItems.size(); i++){
                                if(i < 2){
                                    objsToTake.push_back(orderItems[i]);
                                    objsToGrasp.push_back(orderItems[i]);
                                }
                            }
                            armsFree[0] = true;
                            armsFree[1] = true;
                        }else{
                            objsToTake.push_back(orderItems[0]);
                            objsToGrasp.push_back(orderItems[0]);
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
                        objsToDeliv[0] = idObject;
                        armsFree[0] = false;
                    }else if(armsFree[1]){
                        JustinaManip::laGoTo("navigation", 3000);
                        objsToDeliv[1] = idObject;
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
                    if(objsToGuide.size() > 0){
                        ss.str("");
                        ss << "Barman i can not take the " << objsToGuide[0] << " , help me please";
                        JustinaHRI::waitAfterSay(ss.str(), 5000);
                        ss.str("");
                        ss << "take the " << objsToGuide[0] << " and tell me, justina continue, when you have taken the " << objsToGuide[0];
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay(ss.str(), 7000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);
                        countRepetOrder = 1;
                        nextState = SM_WAIT_FOR_GUIDE_BARMAN;
                    }
                    else{
                        JustinaHRI::waitAfterSay("thanks barman", 2000, minDelayAfterSay);
                        attempsNavigation = 1;
                        nextState = SM_NAVIGATE_TABLE;
                    }
                }
                break;
            case SM_WAIT_FOR_GUIDE_BARMAN:
                std::cout << "State machine: SM_WAIT_FOR_GUIDE_BARMAN" << std::endl;
                if(countRepetOrder <= maxCountRepetOrder){
                    if(JustinaHRI::waitForSpecificSentence("justina continue", timeoutspeech)){
                        JustinaHRI::enableSpeechRecognized(false);
                        nextState = SM_VERIFY_OBJECT;
                    }else{
                        ss.str("");
                        ss << "Barman i can not take the " << objsToGuide[0] << " , help me please";
                        JustinaHRI::waitAfterSay(ss.str(), 5000);
                        ss.str("");
                        ss << "take the " << objsToGuide[0] << " and tell me, justina continue, when you have taken the " << objsToGuide[0];
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay(ss.str(), 7000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);
                        countRepetOrder++;
                    }
                }
                else{
                    JustinaHRI::enableSpeechRecognized(false);
                    nextState = SM_VERIFY_OBJECT;
                }
                break;
            
            case SM_VERIFY_OBJECT:
                std::cout << "State machine: SM_VERIFY_OBJECT" << std::endl;
                recognizedObjects = std::vector<vision_msgs::VisionObject>();
                if(JustinaVision::detectObjects(recognizedObjects)){
                    int indexFound = 0;
                    bool found = false;
                    for (int i = 0; i < recognizedObjects.size(); i++) {
                        vision_msgs::VisionObject vObject = recognizedObjects[i];
                        if (vObject.id.compare(idObject) == 0) {
                            found = true;
                            indexFound = i;
                            break;
                        }
                    }
                    if(found){
                        objsToDeliv[2] = objsToGuide[2];
                        JustinaHRI::waitAfterSay("Barman thank you, i will guide you to the client", 6000, minDelayAfterSay);
                        objsToGuide = std::vector<std::string>();
                        nextState = SM_WAIT_OBJECT;
                    }
                    else{
                        ss.str("");
                        ss << "barman the correct object is " << objsToGuide[0] << ", please try again";
                        JustinaHRI::waitAfterSay(ss.str(), 7000, minDelayAfterSay);
                        boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
                        JustinaHRI::waitAfterSay("Barman thank you, i will guide you to the client", 6000, minDelayAfterSay);
                        objsToGuide = std::vector<std::string>();
                        nextState = SM_GUIDE_TABLE;
                    }
                }
                else{
                    objsToDeliv[2] = objsToGuide[2];
                    objsToGuide = std::vector<std::string>();
                    nextState = SM_WAIT_OBJECT;
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
            case SM_GUIDE_TABLE:
                std::cout << "State machine: SM_GUIDE_TABLE" << std::endl;
                ss.str("");
                ss << "table_" << numberTable;
                JustinaHRI::enableSpeechRecognized(false);
                if(!JustinaTasks::guideAPerson(ss.str(), 140000, 1.5)){
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
                ss.str("");
                ss << "barman put the " << objsToDeliv[2] << " on the customer table, please";
                JustinaHRI::waitAfterSay(ss.str(), 5000, minDelayAfterSay);
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                ss << "thank you barman, now you can go back to the kitchen bar";
                ss.str("");
                JustinaHRI::waitAfterSay(ss.str(), 5000, minDelayAfterSay);
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

