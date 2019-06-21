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

#define MAX_FIND_PERSON_COUNT 1
#define MAX_FIND_PERSON_RESTART 2
#define MAX_FIND_PERSON_ATTEMPTS 2
#define TIMEOUT_SPEECH 10000
#define MIN_DELAY_AFTER_SAY 0
#define MAX_DELAY_AFTER_SAY 300
#define MAX_ATTEMPTS_SPEECH_INT 3
#define MAX_ATTEMPTS_SPEECH_RECO 3
#define MAX_ATTEMPTS_CONFIRMATION 2
#define MAX_ATTEMPTS_WAIT_CONFIRMATION 2
#define MAX_ATTEMPTS_MEMORIZING 2
#define MAX_FIND_SEAT_COUNT 4
#define TIMEOUT_MEMORIZING 3000
#define GRAMMAR_POCKET_COMMANDS "grammars/pre_sydney/commands.jsgf"
#define GRAMMAR_POCKET_DRINKS "grammars/pre_sydney/order_drink.jsgf"
#define GRAMMAR_POCKET_NAMES "grammars/pre_sydney/people_names.jsgf"
#define GRAMMAR_COMMANDS "commands.xml"
#define GRAMMAR_DRINKS "order_drink.xml"
#define GRAMMAR_NAMES "peopple_names.xml"

enum STATE{
    SM_INIT,
    SM_NAVIGATE_TO_ENTRANCE_DOOR,
    SM_NAVIGATE_TO_RECO_LOC,
    SM_SAY_OPEN_DOOR,
    SM_WAIT_FOR_OPEN_DOOR,
    SM_WAIT_FOR_PERSON_ENTRANCE,
    SM_INTRO_GUEST,
    SM_WAIT_FOR_PRESENTATION,
    SM_PRESENTATION_CONFIRM,
    SM_MEMORIZING_OPERATOR,
    SM_WAITING_FOR_MEMORIZING_OPERATOR,
    SM_GUIDE_TO_LOC,
    SM_FIND_TO_HOST,
    SM_FIND_TO_GUEST,
    SM_INTRODUCING,
    SM_FIND_EMPTY_SEAT,
    SM_OFFER_EMPTY_SEAT,
    SM_FINISH_TEST
};

std::string lastRecoSpeech;
std::string lastInteSpeech;

std::string test("receptionist");

int main(int argc, char **argv){

    ros::init(argc, argv, "receptionist_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    bool doorOpenFlag = false;
    bool opened = false;
    bool success = false;
    bool findPerson = false;
    bool findSeat = false;
    bool recogName = false;
    bool completeTrainig = false;
    std::vector<bool> memorizingOperators;
    int findPersonCount = 0;
    int findPersonAttemps = 0;
    int findPersonRestart = 0;
    int findSeatCount = 0;
    int attemptsSpeechReco = 0;
    int attemptsSpeechInt = 0;
    int attemptsConfirmation = 0;
    int attemptsWaitConfirmation = 0;
    int attemptsMemorizing = 0;
    float pitchAngle;
    int genderRecog;
    int numGuests = 1;
    std::string param, typeOrder;
    std::string lastName, lastDrink;
    std::vector<std::string> names;
    std::vector<std::string> drinks;
    std::string grammarCommandsID = "receptionisCommands";
    std::string grammarDrinksID = "receptionistDrinks";
    std::string grammarNamesID = "receptionistNames";
    std::string recogLoc = "kitchen";
    std::string entranceLoc = "entrance_door";
    std::string hostDrink = "coke";

    Eigen::Vector3d centroid;
    std::vector<Eigen::Vector3d> faceCentroids;
    std::vector<Eigen::Vector3d> centroids;
    
    std::stringstream ss;
    std::stringstream ss2;
    
    float robot_y, robot_x, robot_a;    
    float torsoSpine, torsoWaist, torsoShoulders;
    float gx_w, gy_w, gz_w, guest_z, host_z;    
    float goalx, goaly, goala;
    float dist_to_head;
    float theta = 0, thetaToGoal = 0, angleHead = 0;
    float pointingArmX, pointingArmY, pointingArmZ;
    float pointingDirX, pointingDirY, pointingDirZ, pointingNormal;
    float distanceArm = 0.6;
    bool usePointArmLeft = false;
    
    std::vector<std::string> confirmCommands;
    confirmCommands.push_back("justina yes");
    confirmCommands.push_back("justina no");
    confirmCommands.push_back("robot yes");
    confirmCommands.push_back("robot no");

    std::vector<std::string> idsPerson;
    idsPerson.push_back("person");
    std::vector<std::string> idsSeat;
    idsSeat.push_back("chair");
    idsSeat.push_back("sofa");
	
    boost::posix_time::ptime prev;
	boost::posix_time::ptime curr;
    
    std::vector<std::string> tokens;

    STATE state = SM_INIT;
    
    std::vector<vision_msgs::VisionObject> yoloObjects;

    JustinaVision::setNodeHandle(&nh);
    JustinaTasks::setNodeHandle(&nh);
    JustinaHardware::setNodeHandle(&nh);
    JustinaHRI::setNodeHandle(&nh);
    JustinaManip::setNodeHandle(&nh);
    JustinaNavigation::setNodeHandle(&nh);
    JustinaTools::setNodeHandle(&nh);
    JustinaVision::setNodeHandle(&nh);
    JustinaNavigation::setNodeHandle(&nh);
    JustinaKnowledge::setNodeHandle(&nh);
    JustinaRepresentation::setNodeHandle(&nh);

    JustinaHRI::usePocketSphinx = false;

    while(ros::ok() && !success){

        switch(state){
            case SM_INIT:
                std::cout << test << ".-> State SM_INIT: Init the test." << std::endl;
                JustinaHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_COMMANDS);
                ros::spinOnce();
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaHRI::loadGrammarSpeechRecognized(grammarNamesID, GRAMMAR_POCKET_NAMES);
                ros::spinOnce();
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaHRI::loadGrammarSpeechRecognized(grammarDrinksID, GRAMMAR_POCKET_DRINKS);
                ros::spinOnce();
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                JustinaHRI::waitAfterSay("I am ready for the receptionist test", 6000, MIN_DELAY_AFTER_SAY);
                state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                break;

            case SM_NAVIGATE_TO_ENTRANCE_DOOR:
                std::cout << test << ".-> State SM_NAVIGATE_TO_ENTRANCE_DOOR: Navigate to the entrance door." << std::endl;
                //JustinaHRI::insertAsyncSpeech("I will navigate to the entrance door", 4000, ros::Time::now().sec, 10);
                JustinaHRI::waitAfterSay("I will navigate to the entrance door", 4000, MIN_DELAY_AFTER_SAY);
                if(!JustinaNavigation::getClose(entranceLoc, 80000))
                    JustinaNavigation::getClose(entranceLoc, 80000);
                //JustinaHRI::insertAsyncSpeech("I have reached the entrance door", 4000, ros::Time::now().sec, 10);
                JustinaHRI::waitAfterSay("I have reached the entrance door", 4000, MIN_DELAY_AFTER_SAY);
                if(doorOpenFlag)
                {
                    JustinaHRI::waitAfterSay("Hello human, please enter to the house", 6000, MIN_DELAY_AFTER_SAY);
                    //JustinaHRI::insertAsyncSpeech("Hello human, please enter to the house", 5000, ros::Time::now().sec, 10);
                    JustinaManip::hdGoTo(0.0, -0.3, 4000);
                    state = SM_WAIT_FOR_PERSON_ENTRANCE;
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                }
                else
                {   
                    doorOpenFlag = true;
                    state = SM_SAY_OPEN_DOOR;
                }
                break;
            
            case SM_NAVIGATE_TO_RECO_LOC:
                std::cout << test << ".-> State SM_NAVIGATE_TO_RECOG_LOC: Navigate to the recog loc." << std::endl;
                if(!JustinaNavigation::getClose(recogLoc, 80000) )
                    JustinaNavigation::getClose(recogLoc, 80000);
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                JustinaHRI::waitAfterSay("John, I'm going to find you", 5000);
                //JustinaHRI::insertAsyncSpeech("John, I'm going to find you", 5000, ros::Time::now().sec, 10);
                state = SM_FIND_TO_HOST;
                break;

            case SM_SAY_OPEN_DOOR:
                std::cout << test << ".-> State SM_SAY_OPEN_DOOR: Saying open the door." << std::endl;
                //JustinaHRI::insertAsyncSpeech("Human, please open the door", 4000, ros::Time::now().sec, 10);
                JustinaHRI::waitAfterSay("Human, please open the door", 6000, MIN_DELAY_AFTER_SAY);
                state = SM_WAIT_FOR_OPEN_DOOR;
                break;

            case SM_WAIT_FOR_OPEN_DOOR:
                std::cout << test << ".-> State SM_WAIT_FOR_OPEN_DOOR: Wait for the open the door." << std::endl;
                opened = JustinaNavigation::doorIsOpen(0.9, 2000);
                /*
                if(!JustinaNavigation::obstacleInFront())
                    nextState = SM_NAVIGATE_TO_INSPECTION;
                break;
                */
                state = SM_SAY_OPEN_DOOR;
                if(opened){
                    JustinaHRI::waitAfterSay("Hello human, please enter to the house", 6000, MIN_DELAY_AFTER_SAY);
                    //JustinaHRI::insertAsyncSpeech("Hello human, please enter to the house", 5000, ros::Time::now().sec, 10);
                    JustinaManip::hdGoTo(0.0, -0.3, 4000);
                    state = SM_WAIT_FOR_PERSON_ENTRANCE;
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                }
                else
                    ros::Duration(3).sleep();
                break;
            case SM_WAIT_FOR_PERSON_ENTRANCE:
                std::cout << test << ".-> State SM_WAIT_FOR_PERSON_ENTRANCE: Intro Guest." << std::endl;
                if(findPersonAttemps < MAX_FIND_PERSON_ATTEMPTS){
                    findPerson = JustinaTasks::turnAndRecognizeYolo(idsPerson, JustinaTasks::NONE, 0.0f, 0.1f, 0.0f, -0.2f, -0.2f, -0.3f, 0.1f, 0.1f, 9.0, centroids, "entrance");
                    if(findPerson){
                        centroid = centroids[0];
                        findPersonCount++;
                    }
                    if(findPersonCount > MAX_FIND_PERSON_COUNT){
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                    
                        JustinaTools::transformPoint("/base_link", centroid(0, 0), centroid(1, 0) , centroid(2, 0), "/map", gx_w, gy_w, gz_w);
                        goalx = gx_w;
                        goaly = gy_w;

                        JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                        thetaToGoal = atan2(goaly - robot_y, goalx - robot_x);
                        if (thetaToGoal < 0.0f)
                            thetaToGoal += 2 * M_PI;
                        theta = thetaToGoal - robot_a;
                        std::cout << "JustinaTasks.->Theta To goal:" << thetaToGoal << std::endl;
                        std::cout << "JustinaTasks.->Turn in direction of robot:" << theta << std::endl;
                        JustinaNavigation::moveDistAngle(0, theta, 2000);
                        dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly - robot_y, 2));
                        JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
                        JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                        angleHead = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
                        if(angleHead < -M_PI)
                            angleHead = 2 * M_PI + angleHead;
                        if(angleHead > M_PI)
                            angleHead = 2 * M_PI - angleHead;
                        recogName = true;
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRO_GUEST;
                    }
                    else{
                        if(findPersonRestart > MAX_FIND_PERSON_RESTART){
                            findPersonCount = 0;
                            findPersonRestart = 0;
                            findPersonAttemps++;
                            JustinaHRI::waitAfterSay("Hello human, please enter to the house", 6000, MIN_DELAY_AFTER_SAY);
                            //JustinaHRI::insertAsyncSpeech("Hello human, please enter to the house", 5000, ros::Time::now().sec, 10);
                        }
                        else
                            findPersonRestart++;
                    }
                }else{
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    recogName = true;
                    state = SM_INTRO_GUEST;
                }
                break;
            case SM_INTRO_GUEST:
                std::cout << test << ".-> State SM_INTRO_GUEST: Intro Guest." << std::endl;
                attemptsSpeechReco = 0;
                attemptsSpeechInt = 0;
                lastName = "unknown";
                lastDrink = "unknown";
                JustinaHRI::enableSpeechRecognized(false);//Enable recognized speech
                if(recogName){
                    JustinaHRI::waitAfterSay("Hello, my name is Justina, please tell me, what is your name", 10000, MAX_DELAY_AFTER_SAY);
                    if(JustinaHRI::usePocketSphinx)
                        JustinaHRI::enableGrammarSpeechRecognized(grammarNamesID, 0);//load the grammar
                    else
                        JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_NAMES);
                }else{
                    JustinaHRI::waitAfterSay("Please tell me, what is your favorite drink", 10000, MAX_DELAY_AFTER_SAY);
                    if(JustinaHRI::usePocketSphinx)
                        JustinaHRI::enableGrammarSpeechRecognized(grammarDrinksID, 0);//load the grammar
                    else
                        JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_DRINKS);
                }
                if(!JustinaHRI::usePocketSphinx)
                    JustinaHRI::enableSpeechRecognized(true);//Enable recognized speech
                attemptsConfirmation = 0;
                attemptsWaitConfirmation = 0;
                state = SM_WAIT_FOR_PRESENTATION;
                break;

            case SM_WAIT_FOR_PRESENTATION:
                std::cout << test << ".-> State SM_WAIT_FOR_NAME: Waiting for the names." << std::endl;
                if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, TIMEOUT_SPEECH)){
                    if(JustinaRepresentation::stringInterpretation(lastRecoSpeech, lastInteSpeech)){
                        if(JustinaRepresentation::receptionistInterpeted(lastInteSpeech, typeOrder, param)){
                            ss.str("");
                            if(recogName && typeOrder.compare("receptionist_guest_name") == 0){
                                tokens.clear();
                                if(param.compare(" ") != 0 || param.compare("") != 0){
                                    ss << "Ok, your name is ";
                                    boost::algorithm::split(tokens, param, boost::algorithm::is_any_of("_"));
                                    ss2.str("");
                                    for(int i = 0; i < tokens.size(); i++){
                                        ss << tokens[i] << " ";
                                        ss2 << tokens[i];
                                        if(i < tokens.size() -1)
                                            ss2 << " ";
                                    }
                                    lastName = ss2.str();
                                    //names.push_back(ss2.str());
                                    ss << ", please tell me justina yes or justina no";
                                    JustinaHRI::enableSpeechRecognized(false);
                                    JustinaHRI::waitAfterSay(ss.str(), 10000, MAX_DELAY_AFTER_SAY);
                                    if(JustinaHRI::usePocketSphinx)
                                        JustinaHRI::enableGrammarSpeechRecognized(grammarCommandsID, 0);//load the grammar
                                    else
                                        JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_COMMANDS);
                                    if(!JustinaHRI::usePocketSphinx)
                                        JustinaHRI::enableSpeechRecognized(true);
                                    //attemptsConfirmation = 0;
                                    //attemptsWaitConfirmation = 0;
                                    state = SM_PRESENTATION_CONFIRM;
                                    break;
                                }
                            }
                            if(!recogName && typeOrder.compare("receptionist_favorite_drink") == 0){
                                tokens.clear();
                                if(param.compare(" ") != 0 || param.compare("") != 0){
                                    ss << "Ok, your favorite drink is ";
                                    boost::algorithm::split(tokens, param, boost::algorithm::is_any_of("_"));
                                    ss2.str("");
                                    for(int i = 0; i < tokens.size(); i++){
                                        ss << tokens[i] << " ";
                                        ss2 << tokens[i];
                                        if(i < tokens.size() -1)
                                            ss2 << " ";
                                    }
                                    lastDrink = ss2.str();
                                    //drinks.push_back(ss2.str());
                                    ss << ", please tell me justina yes or justina no";
                                    JustinaHRI::enableSpeechRecognized(false);
                                    JustinaHRI::waitAfterSay(ss.str(), 10000, MAX_DELAY_AFTER_SAY);
                                    if(JustinaHRI::usePocketSphinx)
                                        JustinaHRI::enableGrammarSpeechRecognized(grammarCommandsID, 0);//load the grammar
                                    else
                                        JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_COMMANDS);
                                    if(!JustinaHRI::usePocketSphinx)
                                        JustinaHRI::enableSpeechRecognized(true);
                                    //attemptsConfirmation = 0;
                                    //attemptsWaitConfirmation = 0;
                                    state = SM_PRESENTATION_CONFIRM;
                                    break;
                                }
                            }
                        }
                    }
                    if(attemptsSpeechInt < MAX_ATTEMPTS_SPEECH_INT){
                        JustinaHRI::enableSpeechRecognized(false);
                        if(recogName)
                            JustinaHRI::waitAfterSay("Please tell me what is your name", 7000, MAX_DELAY_AFTER_SAY);
                        else
                            JustinaHRI::waitAfterSay("Please tell me what is your favorite drink", 7000, MAX_DELAY_AFTER_SAY);
                            //JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what is your favorite drink", 7000, MAX_DELAY_AFTER_SAY);
                        attemptsSpeechInt++;
                        JustinaHRI::enableSpeechRecognized(true);
                    }
                    else{
                        JustinaHRI::enableSpeechRecognized(false);
                        attemptsSpeechReco = 0;
                        attemptsSpeechInt = 0;
                        if(recogName){
                            ss2.str("");
                            if(lastName.compare("unknown") == 0)
                                ss2 << "Sorry I did not understand you, you are an unknown person ";
                            else
                                ss2 << "Ok, your name is " << lastName;
                            JustinaHRI::waitAfterSay(ss2.str(), 12000, MIN_DELAY_AFTER_SAY);
                            names.push_back(lastName);
                            recogName = false;
                            if(JustinaHRI::usePocketSphinx)
                                JustinaHRI::enableGrammarSpeechRecognized(grammarDrinksID, 0);//load the grammar
                            else
                                JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_DRINKS);
                            if(!JustinaHRI::usePocketSphinx)
                                JustinaHRI::enableSpeechRecognized(true);//Enable recognized speech
                            state = SM_INTRO_GUEST;
                        }else{
                            ss2.str("");
                            if(lastDrink.compare("unknown") == 0)
                                ss2 << "Sorry I did not understand you, your favorite drink is unknown";
                            else
                                ss2 << "Ok, your favorite drink is " << lastDrink;
                            JustinaHRI::waitAfterSay(ss2.str(), 12000, MIN_DELAY_AFTER_SAY);
                            drinks.push_back(lastDrink);
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                }
                else{
                    if(attemptsSpeechReco < MAX_ATTEMPTS_SPEECH_RECO){
                        JustinaHRI::enableSpeechRecognized(false);
                        if(recogName)
                            JustinaHRI::waitAfterSay("Please tell me what is your name", 7000, MAX_DELAY_AFTER_SAY);
                        else
                            JustinaHRI::waitAfterSay("Please tell me what is your favorite drink", 7000, MAX_DELAY_AFTER_SAY);
                        attemptsSpeechReco++;
                        JustinaHRI::enableSpeechRecognized(true);
                    }
                    else{
                        JustinaHRI::enableSpeechRecognized(false);
                        attemptsSpeechReco = 0;
                        attemptsSpeechInt = 0;
                        if(recogName){
                            ss2.str("");
                            if(lastName.compare("unknown") == 0)
                                ss2 << "Sorry I did not understand you, you are an unknown person ";
                            else
                                ss2 << "Ok, your name is " << lastName;
                            JustinaHRI::waitAfterSay(ss2.str(), 7000, MIN_DELAY_AFTER_SAY);
                            names.push_back(lastName);
                            recogName = false;
                            JustinaHRI::enableSpeechRecognized(true);
                            state = SM_INTRO_GUEST;
                        }else{
                            ss2.str("");
                            if(lastDrink.compare("unknown") == 0)
                                ss2 << "Sorry I did not understand you, your favorite drink is unknown";
                            else
                                ss2 << "Ok, your favorite drink is " << lastDrink;
                            JustinaHRI::waitAfterSay(ss2.str(), 7000, MIN_DELAY_AFTER_SAY);
                            drinks.push_back(lastDrink);
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                }
                break;

            case SM_PRESENTATION_CONFIRM:
                std::cout << test << ".-> State SM_PRESENTATION_CONFIRM: Confirm presentation." << std::endl;
                attemptsSpeechReco = 0;
                attemptsSpeechInt = 0;
                if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, TIMEOUT_SPEECH)){
                    if(lastRecoSpeech.find("yes") != std::string::npos){
                        JustinaHRI::enableSpeechRecognized(false);
                        if(recogName){
                            names.push_back(lastName);
                            ss2.str("");
                            ss2 << "Ok, your name is " << names[names.size() - 1];
                            JustinaHRI::waitAfterSay(ss2.str(), 6000, MAX_DELAY_AFTER_SAY);
                            recogName = false;
                            state = SM_INTRO_GUEST;
                        }
                        else{
                            drinks.push_back(lastDrink);
                            ss2.str("");
                            ss2 << "Ok, your favorite drink is " << drinks[drinks.size() - 1];
                            JustinaHRI::waitAfterSay(ss2.str(), 6000, MIN_DELAY_AFTER_SAY);
                            attemptsMemorizing = 0;
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                    else{
                        if(attemptsConfirmation < MAX_ATTEMPTS_CONFIRMATION){
                            attemptsConfirmation++;
                            JustinaHRI::enableSpeechRecognized(false);
                            if(recogName){
                                //names.erase(names.end()- 1);
                                JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what is your name", 7000, MAX_DELAY_AFTER_SAY);
                                if(JustinaHRI::usePocketSphinx)
                                    JustinaHRI::enableGrammarSpeechRecognized(grammarNamesID, 0);//load the grammar
                                else
                                    JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_NAMES);
                                if(!JustinaHRI::usePocketSphinx)
                                    JustinaHRI::enableSpeechRecognized(true);
                            }else{
                                //drinks.erase(names.end() - 1);
                                JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what is your favorite drink", 7000, MAX_DELAY_AFTER_SAY);
                                if(JustinaHRI::usePocketSphinx)
                                    JustinaHRI::enableGrammarSpeechRecognized(grammarDrinksID, 0);//load the grammar
                                else
                                    JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_DRINKS);
                                if(!JustinaHRI::usePocketSphinx)
                                    JustinaHRI::enableSpeechRecognized(true);
                            }
                            if(!JustinaHRI::usePocketSphinx)
                                JustinaHRI::enableSpeechRecognized(true);
                            state = SM_WAIT_FOR_PRESENTATION;
                        }
                        else{
                            JustinaHRI::enableSpeechRecognized(false);
                            if(recogName){
                                names.push_back(lastName);
                                ss2.str("");
                                ss2 << "Ok, your name is " << names[names.size() - 1];
                                JustinaHRI::waitAfterSay(ss2.str(), 6000, MAX_DELAY_AFTER_SAY);
                                JustinaHRI::enableSpeechRecognized(true);
                                recogName = false;
                                state = SM_INTRO_GUEST;
                            }
                            else{
                                drinks.push_back(lastDrink);
                                ss2.str("");
                                ss2 << "Ok, your favorite drink is " << drinks[drinks.size() - 1];
                                JustinaHRI::waitAfterSay(ss2.str(), 6000, MIN_DELAY_AFTER_SAY);
                                attemptsMemorizing = 0;
                                state = SM_MEMORIZING_OPERATOR;
                            }
                        }
                    }
                }
                else {
                    if(attemptsWaitConfirmation < MAX_ATTEMPTS_WAIT_CONFIRMATION){
                        attemptsWaitConfirmation++;
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay(ss.str(), 10000);
                        JustinaHRI::enableSpeechRecognized(true);
                        state = SM_PRESENTATION_CONFIRM;
                    }
                    else{
                        JustinaHRI::enableSpeechRecognized(false);
                        if(recogName){
                            names.push_back(lastName);
                            ss2.str("");
                            ss2 << "Ok, your name is " << names[names.size() - 1];
                            JustinaHRI::waitAfterSay(ss2.str(), 6000, MAX_DELAY_AFTER_SAY);
                            recogName = false;
                            state = SM_INTRO_GUEST;
                        }else{
                            drinks.push_back(lastDrink);
                            ss2.str("");
                            ss2 << "Ok, your favorite drink is " << drinks[drinks.size() - 1];
                            JustinaHRI::waitAfterSay(ss2.str(), 6000, MIN_DELAY_AFTER_SAY);
                            attemptsMemorizing = 0;
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                }
                break;

            case SM_MEMORIZING_OPERATOR:
                std::cout << test << ".-> State SM_MEMORIZING_OPERATOR: Memorizing operator." << std::endl;
                if(attemptsMemorizing < MAX_ATTEMPTS_MEMORIZING){
                    JustinaManip::hdGoTo(0, 0, 2000);
                    //JustinaHRI::waitAfterSay("Human, please stay in front of me", 6000, MIN_DELAY_AFTER_SAY);
                    JustinaHRI::waitAfterSay("Human, please not move, and look at me", 6000, MIN_DELAY_AFTER_SAY);
                    JustinaVision::faceTrain(names[names.size() - 1], 4);
                    // TODO Get service of the face and gender
                    state = SM_WAITING_FOR_MEMORIZING_OPERATOR;
                }
                else{
                    memorizingOperators.push_back(false);
                    state = SM_GUIDE_TO_LOC;
                }
                break;

            case SM_WAITING_FOR_MEMORIZING_OPERATOR:
                std::cout << test << ".-> State SM_WAITING_FOR_MEMORIZING_OPERATOR: Waiting for Memorizing operator." << std::endl;
                JustinaHRI::waitAfterSay("I'm memorizing your face", 6000, MIN_DELAY_AFTER_SAY);
                    
                state = SM_WAITING_FOR_MEMORIZING_OPERATOR;
                if(JustinaVision::waitForTrainingFace(TIMEOUT_MEMORIZING)){
                    memorizingOperators.push_back(true);
                    state = SM_GUIDE_TO_LOC;
                }
                attemptsMemorizing++;
                break;
            
            case SM_GUIDE_TO_LOC:
                std::cout << test << ".-> State SM_GUIDING_TO_LOC: Guide to loc." << std::endl;
                //ss.str("");
                //JustinaHRI::waitAfterSay(ss.str(), 4000, MAX_DELAY_AFTER_SAY);
                JustinaNavigation::moveDistAngle(0, M_PI, 3500);
                JustinaTasks::guideAPerson(recogLoc, 90000, 1.75);
                attemptsMemorizing = 0;
                findSeatCount = 0;
                JustinaHRI::waitAfterSay("I'm going to find a empty seat for you", 5000);
                //JustinaHRI::insertAsyncSpeech("I'm going to find a empty seat for you", 5000, ros::Time::now().sec, 10);
                state = SM_FIND_EMPTY_SEAT;
                break;

            case SM_FIND_TO_HOST:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to John." << std::endl;
                theta = 0;
                faceCentroids = std::vector<Eigen::Vector3d>();
                findPerson = JustinaTasks::turnAndRecognizeFace("john", -1, -1, JustinaTasks::NONE, -M_PI_4, M_PI_4 / 2.0, M_PI_4, 0, -M_PI_4 / 2.0, -M_PI_4 / 2.0, 1.0f, 1.0f, faceCentroids, genderRecog, "kitchen");
                if(findPerson){
                    JustinaHRI::waitAfterSay("John, I found you", 3000);
                    //JustinaHRI::insertAsyncSpeech("John, I found you", 5000, ros::Time::now().sec, 10);
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    JustinaTools::transformPoint("/base_link", faceCentroids[0](0, 0), faceCentroids[0](1, 0) , faceCentroids[0](2, 0), "/map", gx_w, gy_w, gz_w);
                    host_z = gz_w;
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    JustinaKnowledge::addUpdateKnownLoc("john", gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                    state = SM_INTRODUCING;
                }
                else{
                    if(findPersonAttemps > MAX_FIND_PERSON_ATTEMPTS){
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRODUCING;
                    }
                    else
                        findPersonAttemps++;
                    JustinaHRI::waitAfterSay("John, I'm going to find you again", 5000);
                    //JustinaHRI::insertAsyncSpeech("John, I'm going to find you again", 5000, ros::Time::now().sec, 10);
                }
                
                break;
            
            case SM_FIND_TO_GUEST:
                std::cout << test << ".-> State SM_FIND_TO_GUEST: Finding to ." << std::endl;
                theta = 0;
                faceCentroids = std::vector<Eigen::Vector3d>();
                findPerson = JustinaTasks::turnAndRecognizeFace(names[names.size() - 1], -1, -1, JustinaTasks::NONE, 0.0f, 0.1f, 0.0f, -0.2f, -0.2f, -0.3f, 0.1f, 0.1f, faceCentroids, genderRecog, "living_room");
                if(findPerson){
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    JustinaTools::transformPoint("/base_link", faceCentroids[0](0, 0), faceCentroids[0](1, 0) , faceCentroids[0](2, 0), "/map", gx_w, gy_w, gz_w);
                    guest_z = gz_w;
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    JustinaKnowledge::addUpdateKnownLoc("guest", gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                    state = SM_NAVIGATE_TO_RECO_LOC;
                }
                else{
                    if(findPersonAttemps > MAX_FIND_PERSON_ATTEMPTS){
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_NAVIGATE_TO_RECO_LOC;
                    }
                    else
                        findPersonAttemps++;
                    ros::Duration(0.5).sleep(); 
                }
                
                break;

            case SM_INTRODUCING:
                std::cout << test << ".-> State SM_INTRODUCING: Introducing person to Jhon." << std::endl;
                ss.str("");
                ss << "John you have a visitor, his name is " << names[names.size() - 1] << " and his favorite drink is " << drinks[drinks.size() - 1];
                //JustinaHRI::insertAsyncSpeech(ss.str(), 8000, ros::Time::now().sec, 10);
                if(JustinaKnowledge::existKnownLocation("john")){
                    JustinaKnowledge::getKnownLocation("john", goalx, goaly, goala);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    thetaToGoal = atan2(goaly - robot_y, goalx - robot_x);
                    if (thetaToGoal < 0.0f)
                        thetaToGoal = 2 * M_PI + thetaToGoal;
                    theta = thetaToGoal - robot_a;
                    std::cout << "JustinaTasks.->Turn in direction of robot:" << theta << std::endl;
                    JustinaManip::startHdGoTo(0, -0.3);
                    JustinaNavigation::moveDistAngle(0, theta, 4000);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly- robot_y, 2));
                    float torsoSpine, torsoWaist, torsoShoulders;
                    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
                    //JustinaManip::startHdGoTo(atan2(goaly - robot_y, goalx - robot_x) - robot_a, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
                    float angleHead = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
                    if(angleHead < -M_PI)
                        angleHead = 2 * M_PI + angleHead;
                    if(angleHead > M_PI)
                        angleHead = 2 * M_PI - angleHead;
                    JustinaManip::startHdGoTo(angleHead, atan2(host_z - (1.52 + torsoSpine), dist_to_head));
                    //JustinaHRI::waitAfterSay(ss.str(), 6000, MAX_DELAY_AFTER_SAY);
                    if(JustinaKnowledge::existKnownLocation("guest")){
                        JustinaKnowledge::getKnownLocation("guest", goalx, goaly, goala);
                        JustinaTools::transformPoint("/map", goalx, goaly , guest_z, "/base_link", pointingArmX, pointingArmY, pointingArmZ);
                        if(pointingArmY > 0){
                            usePointArmLeft = true;
                            JustinaTools::transformPoint("/map", goalx, goaly , guest_z, "/left_arm_link0", pointingArmX, pointingArmY, pointingArmZ);
                        }else{
                            usePointArmLeft = false;
                            JustinaTools::transformPoint("/map", goalx, goaly , guest_z, "/right_arm_link0", pointingArmX, pointingArmY, pointingArmZ);
                        }
                        pointingNormal = sqrt(pointingArmX * pointingArmX + pointingArmY * pointingArmY + pointingArmZ * pointingArmZ);
                        pointingDirX = pointingArmX / pointingNormal;
                        pointingDirY = pointingArmY / pointingNormal;
                        pointingDirZ = pointingArmZ / pointingNormal;
                        pitchAngle = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
                        if(pitchAngle <= -M_PI)
                            pitchAngle += 2 * M_PI;
                        else if(pitchAngle >= M_PI)
                            pitchAngle -= 2 * M_PI;
                        if(usePointArmLeft){
                            JustinaManip::laGoToCartesian(distanceArm * pointingDirX, distanceArm * pointingDirY, distanceArm * pointingDirZ, 0, pitchAngle, 1.5708, 3000);
                            JustinaHRI::waitAfterSay(ss.str(), 6000, MAX_DELAY_AFTER_SAY);
                            JustinaManip::startLaGoTo("home");
                        }else{
                            JustinaManip::raGoToCartesian(distanceArm * pointingDirX, distanceArm * pointingDirY, distanceArm * pointingDirZ, 0, pitchAngle, 1.5708, 3000);
                            JustinaHRI::waitAfterSay(ss.str(), 6000, MAX_DELAY_AFTER_SAY);
                            JustinaManip::startRaGoTo("home");
                        }
                    }

                }
                ss.str("");
                ss << names[names.size() - 1] << " he is John and his favorite drink is " << hostDrink << std::endl;
                if(JustinaKnowledge::existKnownLocation("guest")){
                    //JustinaHRI::insertAsyncSpeech(ss.str(), 8000, ros::Time::now().sec, 10);
                    JustinaKnowledge::getKnownLocation("guest", goalx, goaly, goala);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    thetaToGoal = atan2(goaly - robot_y, goalx - robot_x);
                    if (thetaToGoal < 0.0f)
                        thetaToGoal = 2 * M_PI + thetaToGoal;
                    theta = thetaToGoal - robot_a;
                    std::cout << "JustinaTasks.->Turn in direction of robot:" << theta << std::endl;
                    JustinaManip::startHdGoTo(0, -0.3);
                    JustinaNavigation::moveDistAngle(0, theta, 4000);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly- robot_y, 2));
                    float torsoSpine, torsoWaist, torsoShoulders;
                    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
                    //JustinaManip::startHdGoTo(atan2(goaly - robot_y, goalx - robot_x) - robot_a, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
                    angleHead = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
                    if(angleHead < -M_PI)
                        angleHead = 2 * M_PI + angleHead;
                    if(angleHead > M_PI)
                        angleHead = 2 * M_PI - angleHead;
                    JustinaManip::startHdGoTo(angleHead, atan2(guest_z - (1.52 + torsoSpine), dist_to_head));
                    if(JustinaKnowledge::existKnownLocation("john")){
                        JustinaKnowledge::getKnownLocation("john", goalx, goaly, goala);
                        JustinaTools::transformPoint("/map", goalx, goaly , host_z, "/base_link", pointingArmX, pointingArmY, pointingArmZ);
                        if(pointingArmY > 0){
                            usePointArmLeft = true;
                            JustinaTools::transformPoint("/map", goalx, goaly , host_z, "/left_arm_link0", pointingArmX, pointingArmY, pointingArmZ);
                        }else{
                            usePointArmLeft = false;
                            JustinaTools::transformPoint("/map", goalx, goaly , host_z, "/right_arm_link0", pointingArmX, pointingArmY, pointingArmZ);
                        }
                        pointingNormal = sqrt(pointingArmX * pointingArmX + pointingArmY * pointingArmY + pointingArmZ * pointingArmZ);
                        pointingDirX = pointingArmX / pointingNormal;
                        pointingDirY = pointingArmY / pointingNormal;
                        pointingDirZ = pointingArmZ / pointingNormal;
                        pitchAngle = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
                        if(pitchAngle <= -M_PI)
                            pitchAngle += 2 * M_PI;
                        else if(pitchAngle >= M_PI)
                            pitchAngle -= 2 * M_PI;
                        if(usePointArmLeft){
                            JustinaManip::laGoToCartesian(distanceArm * pointingDirX, distanceArm * pointingDirY, distanceArm * pointingDirZ, 0, pitchAngle, 1.5708, 3000);
                            JustinaHRI::waitAfterSay(ss.str(), 6000, MAX_DELAY_AFTER_SAY);
                            JustinaManip::startLaGoTo("home");
                        }else{
                            JustinaManip::raGoToCartesian(distanceArm * pointingDirX, distanceArm * pointingDirY, distanceArm * pointingDirZ, 0, pitchAngle, 1.5708, 3000);
                            JustinaHRI::waitAfterSay(ss.str(), 6000, MAX_DELAY_AFTER_SAY);
                            JustinaManip::startRaGoTo("home");
                        }
                    }
                }
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                if( numGuests++ < 2 )
                    state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                else
                    state = SM_FINISH_TEST;
                break;

            case SM_FIND_EMPTY_SEAT:
                std::cout << test << ".-> State SM_FIND_EMPTY_SEAT: Finding empty seat" << std::endl;
                if(findSeatCount < MAX_FIND_SEAT_COUNT){
                    centroids.clear();
                    findSeat = JustinaTasks::turnAndRecognizeYolo(idsSeat, JustinaTasks::NONE, -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2f, -0.2f, -0.3f, 0.1f, 0.1f, 9.0, centroids, "kitchen");
                    if(!findSeat){
                        findSeatCount++;
                        JustinaHRI::waitAfterSay("I'm going to find a empty seat for you again", 5000);
                        //JustinaHRI::insertAsyncSpeech("I'm going to find a empty seat for you again", 5000, ros::Time::now().sec, 10);
                        break;
                    }

                    centroid = centroids[0];
                    JustinaHRI::waitAfterSay("Please wait", 3000, MIN_DELAY_AFTER_SAY);
                    JustinaTools::transformPoint("/base_link", centroid(0, 0), centroid(1, 0) , centroid(2, 0), "/map", gx_w, gy_w, gz_w);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    std::cout << "$$$$$$$$$$$ gx:" << gx_w << " gy :" << gy_w << std::endl;
                    JustinaKnowledge::addUpdateKnownLoc("guest", gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                    goalx = gx_w;
                    goaly = gy_w;
                    guest_z = gz_w;
                    std::cout << "$$$$$$$$$$$ gx:" << gx_w << " gy :" << gy_w << std::endl;
                    JustinaTasks::closeToGoalWithDistanceTHR(goalx, goaly, 0.6, 30000);
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    thetaToGoal = atan2(goaly - robot_y, goalx - robot_x);
                    if (thetaToGoal < 0.0f)
                        thetaToGoal += 2 * M_PI;
                    theta = thetaToGoal - robot_a;
                    JustinaNavigation::moveDistAngle(0, theta, 3000);
                    dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly - robot_y, 2));
                    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
                    rate.sleep();
                    ros::spinOnce();
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    angleHead = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
                    if(angleHead < -M_PI)
                        angleHead = 2 * M_PI + angleHead;
                    if(angleHead > M_PI)
                        angleHead = 2 * M_PI - angleHead;
                    JustinaManip::startHdGoTo(angleHead, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
                    //JustinaManip::startHdGoTo(atan2(goaly - robot_y, goalx - robot_x) - robot_a, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
                    state = SM_OFFER_EMPTY_SEAT;
                }
                else
                    state = SM_OFFER_EMPTY_SEAT;
                break;

            case SM_OFFER_EMPTY_SEAT:
                std::cout << test << ".-> State SM_OFFER_EMPTY_SEAT: Offer empty seat" << std::endl;
                ss.str("");
                ss << names[names.size() - 1] << ", could you sit in this place, please";
                //JustinaHRI::insertAsyncSpeech(ss.str(), 5000, ros::Time::now().sec, 10);

                JustinaManip::startLaGoTo("navigation");
                JustinaManip::startRaGoTo("navigation");
                JustinaManip::waitForLaGoalReached(8000);

                JustinaManip::startLaGoTo("offer_seat");
                JustinaManip::startRaGoTo("offer_seat");
                JustinaManip::waitForLaGoalReached(8000);
                
                
                JustinaManip::startLaGoTo("navigation");
                JustinaManip::startRaGoTo("navigation");
                JustinaManip::waitForLaGoalReached(8000);
                JustinaHRI::waitAfterSay(ss.str(), 6000, MIN_DELAY_AFTER_SAY);
                ss.str("");
                ss << names[names.size() - 1] << "Please, look at me";
                JustinaHRI::waitAfterSay(ss.str(), 4000, MIN_DELAY_AFTER_SAY);
                
                //JustinaHRI::insertAsyncSpeech(ss.str(), 5000, ros::Time::now().sec, 10);


                //JustinaManip::startLaGoTo("home");
                //JustinaManip::startRaGoTo("home");
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                //JustinaHRI::waitAfterSay(ss.str(), 7000, MIN_DELAY_AFTER_SAY);
                state = SM_FIND_TO_GUEST;
                break;
            
            case SM_FINISH_TEST:
                std::cout << test << ".-> State SM_FINISH: Finish the test." << std::endl;
                JustinaHRI::waitAfterSay("I have finished the test", 6000, MIN_DELAY_AFTER_SAY);
                
                success = true;
                
                for(int i = 0; i < names.size(); i++ )
                {
                    
                    std::cout << test << names[i] << std::endl;
                    JustinaVision::facClearByID(names[i]);
                }

                break;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 1;
}
