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

#define MAX_FIND_PERSON_COUNT 3
#define MAX_FIND_PERSON_RESTART 7
#define MAX_FIND_PERSON_ATTEMPTS 3
#define TIMEOUT_SPEECH 10000
#define MIN_DELAY_AFTER_SAY 0
#define MAX_DELAY_AFTER_SAY 300
#define MAX_ATTEMPTS_SPEECH_INT 3
#define MAX_ATTEMPTS_SPEECH_RECO 3
#define MAX_ATTEMPTS_CONFIRMATION 2
#define MAX_ATTEMPTS_WAIT_CONFIRMATION 2
#define MAX_ATTEMPTS_MEMORIZING 2
#define TIMEOUT_MEMORIZING 3000

enum STATE{
    SM_INIT,
    SM_NAVIGATE_TO_ENTRANCE_DOOR,
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
    SM_INTRODUCING,
    SM_FINISH_TEST
};

std::string lastRecoSpeech;
std::string lastInteSpeech;

std::string test("receptionist");

int main(int argc, char **argv){

    ros::init(argc, argv, "receptionist_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    bool opened = false;
    bool success = false;
    bool findPerson = false;
    bool recogName = false;
    std::vector<bool> memorizingOperators;
    int findPersonCount = 0;
    int findPersonAttemps = 0;
    int findPersonRestart = 0;
    int attemptsSpeechReco = 0;
    int attemptsSpeechInt = 0;
    int attemptsConfirmation = 0;
    int attemptsWaitConfirmation = 0;
    int attemptsMemorizing = 0;
    int genderRecog;
    std::string param, typeOrder;
    std::vector<std::string> names;
    std::vector<std::string> drinks;
    std::string grammarAllID = "receptionistALL";
    Eigen::Vector3d centroidPerson;
    
    std::stringstream ss;
    std::stringstream ss2;
    
    float robot_y, robot_x, robot_a;    
    float torsoSpine, torsoWaist, torsoShoulders;
    float gx_w, gy_w, gz_w;    
    float goalx, goaly, goala;
    float dist_to_head;
    
    std::vector<std::string> confirmCommands;
    confirmCommands.push_back("justina yes");
    confirmCommands.push_back("justina no");
	
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

    JustinaHRI::usePocketSphinx = true;
    JustinaHRI::loadGrammarSpeechRecognized(grammarAllID, "grammars/pre_guadalajara/receptionist.jsgf");

    while(ros::ok() && !success){

        switch(state){
            case SM_INIT:
                std::cout << test << ".-> State SM_INIT: Init the test." << std::endl;
                JustinaHRI::waitAfterSay("I am ready for the receptionist test", 6000, MIN_DELAY_AFTER_SAY);
                JustinaHRI::enableGrammarSpeechRecognized(grammarAllID, 0);//load the grammar
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                break;

            case SM_NAVIGATE_TO_ENTRANCE_DOOR:
                std::cout << test << ".-> State SM_NAVIGATE_TO_ENTRANCE_DOOR: Navigate to the entrance door." << std::endl;
                JustinaHRI::waitAfterSay("I will navigate to the entrance door", 4000, MIN_DELAY_AFTER_SAY);
                if(!JustinaNavigation::getClose("entrance_door", 80000))
                    if(!JustinaNavigation::getClose("entrance_door", 80000))
                JustinaHRI::waitAfterSay("I have reached the entrance door", 4000, MIN_DELAY_AFTER_SAY);
                state = SM_SAY_OPEN_DOOR;
                break;

            case SM_SAY_OPEN_DOOR:
                std::cout << test << ".-> State SM_SAY_OPEN_DOOR: Saying open the door." << std::endl;
                JustinaHRI::waitAfterSay("Human, can you open the door please", 6000, MIN_DELAY_AFTER_SAY);
                state = SM_WAIT_FOR_OPEN_DOOR;
                break;

            case SM_WAIT_FOR_OPEN_DOOR:
                std::cout << test << ".-> State SM_WAIT_FOR_OPEN_DOOR: Wait for the open the door." << std::endl;
                opened = JustinaNavigation::doorIsOpen(0.9, 2000);
                state = SM_SAY_OPEN_DOOR;
                if(opened){
                    JustinaHRI::waitAfterSay("Hello human, can you entrance in the house please", 6000, MIN_DELAY_AFTER_SAY);
                    //JustinaVision::enableDetectObjsYOLO(true);
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
                    findPerson = JustinaTasks::turnAndRecognizeYolo("person", JustinaTasks::NONE, 0.0, 0.0, 0.0, -0.2, -0.1, -0.2, 0.0, 0.0f, 9.0, centroidPerson, "entrance");
                    if(findPerson)
                        findPersonCount++;
                    if(findPersonCount > MAX_FIND_PERSON_COUNT){
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                    
                        //JustinaVision::enableDetectObjsYOLO(false);
                        JustinaTools::transformPoint("/base_link", centroidPerson(0, 0), centroidPerson(1, 0) , centroidPerson(2, 0), "/map", gx_w, gy_w, gz_w);
                        goalx = gx_w;
                        goaly = gy_w;

                        JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                        float thetaToGoal = atan2(goaly - robot_y, goalx - robot_x);
                        if (thetaToGoal < 0.0f)
                            thetaToGoal += 2 * M_PI;
                        float theta = thetaToGoal - robot_a;
                        std::cout << "JustinaTasks.->Turn in direction of robot:" << theta << std::endl;
                        JustinaNavigation::moveDistAngle(0, theta, 2000);
                        dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly - robot_y, 2));
                        JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
                        JustinaManip::startHdGoTo(atan2(goaly - robot_y, goalx - robot_x) - robot_a, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
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
                            JustinaHRI::waitAfterSay("Hello human, can you entrance in the house please", 6000, MIN_DELAY_AFTER_SAY);
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
                // TODO Load grammar to recog a names
                if(recogName)
                    JustinaHRI::waitAfterSay("Hello, my name is Justina, please tell me, what is your name", 10000, MAX_DELAY_AFTER_SAY);
                else
                    JustinaHRI::waitAfterSay("Please tell me, what is your favorite drink", 10000, MAX_DELAY_AFTER_SAY);
                JustinaHRI::enableSpeechRecognized(true);//Enable recognized speech
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
                                    names.push_back(ss2.str());
                                    ss << ", please tell me justina yes or justina no";
                                    JustinaHRI::enableSpeechRecognized(false);
                                    JustinaHRI::waitAfterSay(ss.str(), 10000, MAX_DELAY_AFTER_SAY);
                                    // TODO load grammar to robot yes
                                    // JustinaHRI::loadGrammarSpeechRecognized(grammarCommands);
                                    JustinaHRI::enableSpeechRecognized(true);
                                    attemptsConfirmation = 0;
                                    attemptsWaitConfirmation = 0;
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
                                    drinks.push_back(ss2.str());
                                    ss << ", please tell me justina yes or justina no";
                                    JustinaHRI::enableSpeechRecognized(false);
                                    JustinaHRI::waitAfterSay(ss.str(), 10000, MAX_DELAY_AFTER_SAY);
                                    // TODO load grammar to robot yes
                                    // JustinaHRI::loadGrammarSpeechRecognized(grammarCommands);
                                    JustinaHRI::enableSpeechRecognized(true);
                                    attemptsConfirmation = 0;
                                    attemptsWaitConfirmation = 0;
                                    state = SM_PRESENTATION_CONFIRM;
                                    break;
                                }
                            }
                        }
                    }
                    if(attemptsSpeechInt < MAX_ATTEMPTS_SPEECH_INT){
                        JustinaHRI::enableSpeechRecognized(false);
                        if(recogName)
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what is your name", 7000, MAX_DELAY_AFTER_SAY);
                        else
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what is your favorite drink", 7000, MAX_DELAY_AFTER_SAY);
                        attemptsSpeechInt++;
                        JustinaHRI::enableSpeechRecognized(true);
                    }
                    else{
                        JustinaHRI::enableSpeechRecognized(false);
                        attemptsSpeechReco = 0;
                        attemptsSpeechInt = 0;
                        if(recogName){
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, you are a unkown person", 12000, MIN_DELAY_AFTER_SAY);
                            names.push_back("unknown");
                            recogName = false;
                            // TODO If is in only one grammar load grammar
                            JustinaHRI::enableSpeechRecognized(true);
                        }else{
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, your favorite drink is unnkown", 12000, MIN_DELAY_AFTER_SAY);
                            drinks.push_back("unknown");
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                }
                else{
                    if(attemptsSpeechReco < MAX_ATTEMPTS_SPEECH_RECO){
                        JustinaHRI::enableSpeechRecognized(false);
                        if(recogName)
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what is your name", 7000, MAX_DELAY_AFTER_SAY);
                        else
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what is your favorite drink", 7000, MAX_DELAY_AFTER_SAY);
                        attemptsSpeechReco++;
                        JustinaHRI::enableSpeechRecognized(true);
                    }
                    else{
                        JustinaHRI::enableSpeechRecognized(false);
                        attemptsSpeechReco = 0;
                        attemptsSpeechInt = 0;
                        if(recogName){
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, you are a unknown person", 12000, MIN_DELAY_AFTER_SAY);
                            names.push_back("unknown");
                            recogName = false;
                            // TODO If is in only one grammar load grammar
                            JustinaHRI::enableSpeechRecognized(true);
                        }else{
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, your favorite drink is unknown", 12000, MIN_DELAY_AFTER_SAY);
                            names.push_back("unknown");
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
                            ss2.str("");
                            ss2 << "Ok, your name is " << names[names.size() - 1];
                            JustinaHRI::waitAfterSay(ss2.str(), 6000, MAX_DELAY_AFTER_SAY);
                            // TODO LOAD grammar to favorite drink
                            JustinaHRI::enableSpeechRecognized(true);
                            recogName = false;
                            state = SM_INTRO_GUEST;
                        }
                        else{
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
                                JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what is your name", 7000, MAX_DELAY_AFTER_SAY);
                                // TODO load especific grammar
                                // JustinaHRI::loadGrammarSpeechRecognized(grammarCombo);
                            }else{
                                JustinaHRI::waitAfterSay("Sorry I did not understand you, Please tell me what is your favorite drink", 7000, MAX_DELAY_AFTER_SAY);
                                // TODO load especific grammar
                                // JustinaHRI::loadGrammarSpeechRecognized(grammarBeverage);
                            }
                            JustinaHRI::enableSpeechRecognized(true);
                            state = SM_WAIT_FOR_PRESENTATION;
                        }
                        else{
                            JustinaHRI::enableSpeechRecognized(false);
                            if(recogName){
                                ss2.str("");
                                ss2 << "Ok, your name is " << names[names.size() - 1];
                                JustinaHRI::waitAfterSay(ss2.str(), 6000, MAX_DELAY_AFTER_SAY);
                                // TODO LOAD grammar to favorite drink
                                JustinaHRI::enableSpeechRecognized(true);
                                state = SM_INTRO_GUEST;
                            }
                            else{
                                ss2.str("");
                                ss2 << "Ok, your favorite drink is " << drinks[drinks.size() - 1];
                                JustinaHRI::waitAfterSay(ss.str(), 6000, MIN_DELAY_AFTER_SAY);
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
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, you are a unknown person", 12000, MIN_DELAY_AFTER_SAY);
                            names.push_back("unknown");
                            recogName = false;
                            // TODO If is in only one grammar load grammar
                            JustinaHRI::enableSpeechRecognized(true);
                            state = SM_INTRO_GUEST;
                        }else{
                            JustinaHRI::waitAfterSay("Sorry I did not understand you, your favorite drink is unknown", 12000, MIN_DELAY_AFTER_SAY);
                            drinks.push_back("unknown");
                            attemptsMemorizing = 0;
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                }
                break;

            case SM_MEMORIZING_OPERATOR:
                std::cout << test << ".-> State SM_MEMORIZING_OPERATOR: Memorizing operator." << std::endl;
                if(attemptsMemorizing < MAX_ATTEMPTS_MEMORIZING){
                    JustinaHRI::waitAfterSay("Human, please put in front of me", 6000, MIN_DELAY_AFTER_SAY);
                    JustinaHRI::waitAfterSay("please not move, and look at me", 6000, MIN_DELAY_AFTER_SAY);
                    JustinaVision::faceTrain(names[names.size() - 1], 4);
                    // TODO Get service of the face and gender
                    JustinaNavigation::moveDistAngle(0, M_PI, 3500);
                    curr = boost::posix_time::second_clock::local_time();
                    prev = curr;
                    state = SM_WAITING_FOR_MEMORIZING_OPERATOR;
                }
                else{
                    memorizingOperators.push_back(false);
                    state = SM_GUIDE_TO_LOC;
                }
                break;

            case SM_WAITING_FOR_MEMORIZING_OPERATOR:
                std::cout << test << ".-> State SM_WAITING_FOR_MEMORIZING_OPERATOR: Waiting for Memorizing operator." << std::endl;
                bool completeTrainig;
                completeTrainig = false;
                if((curr - prev).total_milliseconds() < TIMEOUT_MEMORIZING){
                    if(JustinaVision::getLastTrainingResult() == 0)
                        completeTrainig = true;
                    curr = boost::posix_time::second_clock::local_time();
                    state = SM_WAITING_FOR_MEMORIZING_OPERATOR;
                    break;
                }
                if(!completeTrainig){
                    attemptsMemorizing++;
                    state = SM_MEMORIZING_OPERATOR; 
                    break;
                }
                memorizingOperators.push_back(true);
                state = SM_GUIDE_TO_LOC;

                break;
            
            case SM_GUIDE_TO_LOC:
                std::cout << test << ".-> State SM_GUIDING_TO_LOC: Guide to loc." << std::endl;
                JustinaTasks::guideAPerson("sofa", 90000, 1.75);
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                state = SM_FIND_TO_HOST;
                break;

            case SM_FIND_TO_HOST:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to John." << std::endl;
                findPerson = JustinaTasks::turnAndRecognizeFace("john", -1, JustinaTasks::NONE, -M_PI_4, M_PI_4 / 2.0, M_PI_4, 0, -M_PI_4 / 2.0, -M_PI_4 / 2.0, 0, 0, centroidPerson, genderRecog, "living_room");
                if(findPerson){
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
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
                }
                
                break;

            case SM_INTRODUCING:
                std::cout << test << ".-> State SM_INTRODUCING: Introducing person to Jhon." << std::endl;
                ss.str("");
                ss << "John you have a visitor, your name is " << names[names.size() - 1] << " and your favorite drink is " << drinks[drinks.size() - 1];
                JustinaHRI::waitAfterSay(ss.str(), 8000, MIN_DELAY_AFTER_SAY);
                ss.str("");
                ss << names[names.size() - 1] << " he is John" << std::endl;
                JustinaHRI::waitAfterSay(ss.str(), 5000, MIN_DELAY_AFTER_SAY);
                state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                break;
            
            case SM_FINISH_TEST:
                std::cout << test << ".-> State SM_FINISH: Finish the test." << std::endl;
                JustinaHRI::waitAfterSay("I have finished the test", 6000, MIN_DELAY_AFTER_SAY);
                success = true;
                break;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 1;
}
