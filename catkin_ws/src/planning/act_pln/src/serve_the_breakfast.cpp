#include <iostream>
#include <cctype>
#include <stdlib.h>
#include "ros/ros.h"
#include "string"
#include "std_msgs/Bool.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaAudio.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaRepresentation.h"

#define MAX_ATTEMPTS_GRASP 3
#define MAX_ATTEMPTS_ALIGN 4
#define MAX_ATTEMPTS_DOOR 5
#define TIMEOUT_SPEECH 5000
#define MIN_DELAY_AFTER_SAY 0
#define MAX_DELAY_AFTER_SAY 300
#define MAX_ATTEMPTS_SPEECH_INT 3
#define MAX_ATTEMPTS_SPEECH_RECO 3
#define MAX_ATTEMPTS_CONFIRMATION 2
#define MAX_ATTEMPTS_WAIT_CONFIRMATION 2
#define MAX_ATTEMPTS_MEMORIZING 2
#define MAX_FIND_SEAT_COUNT 4
#define TIMEOUT_MEMORIZING 3000
#define MAX_ATTEMPTS_FIND_BOWL 4
#define MAX_ATTEMPTS_FIND_SPOON 2
#define MAX_ATTEMPTS_NAME 2

#define GRAMMAR_COMMANDS "commands.xml"
#define GRAMMAR_DRINKS "test_lab_drinks.xml"
#define GRAMMAR_NAMES "test_lab_names.xml"

bool graspObjectColorCupBoardFeedback2(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);
bool pouringCereal(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);
bool placeSpoon(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);
bool placeObject(bool withLeftArm, float h=0, bool placeBag=false);

enum TYPE_CULTLERY{CUTLERY, BOWL, DISH, GLASS,EMPTY};

enum STATE{
    SM_INIT,
    SM_FINISH_TEST,
    SM_WAIT_FOR_OPEN,
    SM_CHECK_IF_DOOR,
    SM_NAVIGATE_TO_CUTLERY_LOC,
    SM_NAVIGATE_TO_ENTRANCE_DOOR,
    SM_GO_TO_TABLEWARE,
    SM_FIND_OBJECTS_ON_TABLE,
    SM_InspectTheObjetcs,
    SM_TAKE_OBJECT,
    SM_GO_TO_KITCHEN,
    SM_LOOK_FOR_TABLE,
    SM_PLACE_BOWL,
    SM_PLACE_SPOON,
    SM_GO_FOR_CEREAL,
    SM_ALIGN_WITH_TABLE,
    SM_RETURN_TO_TABLE,
    SM_TAKE_CEREAL,
    SM_LEAVE_CEREAL,
    SM_SEARCH_BOWL,
    SM_POURING_CEREAL,
    SM_PLACE_MILK,
    SM_TAKE_MILK,
    SM_FIND_BOWL,
    SM_TAKE_BOWL,
    SM_FIND_SPOON,
    SM_TAKE_SPOON,
    SM_GO_TO_TABLE,
    SM_FIND_GUEST,
    SM_INTRO_GUEST,
    SM_PRESENTATION_CONFIRM,
    SM_WAIT_FOR_PRESENTATION,
    SM_SAY_ACTION,
    SM_ASK_DRINK,
    SM_CONFIMR_DRINK,
    SM_DETECT_OBJECT,
    SM_GO_TO_TABLE_DRINKS,
    SM_HANDLER,
    SM_GRASP_OBJECT

};

enum arms{RIGHT_ARM,LEFT_ARM };

std::string lastRecoSpeech;
std::string lastInteSpeech;

std::string test("serve the breakfast");

void printSmTitle(const std::string& input)
{
    std::cout << "\x1b[1;36m"  << test << input << "\x1b[0m" << std::endl;
}

void printWarning(const std::string& input)
{
    std::cout << "\x1b[1;33m WARNING: "  << test << input << " ... \x1b[0m" << std::endl;
}

void printError(const std::string& input)
{
    std::cout << "\x1b[1;31m ERROR: "  << test << input << " U_U \x1b[0m" << std::endl;
}

void printSuccess(const std::string& input)
{
    std::cout << "\x1b[1;32m "  << test << " -"<< input << " n_n \x1b[0m" << std::endl;
}


bool alignWithTable()
{
    int countAlign = 1;
    while(!JustinaTasks::alignWithTable(0.42) && countAlign < MAX_ATTEMPTS_ALIGN )
    {   
        printWarning("Cant align trying again");
        JustinaNavigation::moveDistAngle(0.1, 0, 2000);
        if( countAlign % 3 == 0)
        {
            JustinaNavigation::moveDistAngle(-0.25, 0, 2000);    
        }
        countAlign++;
    }

    if(countAlign < MAX_ATTEMPTS_ALIGN)
        printSuccess("The robot has aligned with the table");    
    else
        printError("The robot could not align with the table");
    }

int left_arm = EMPTY;
int right_arm = EMPTY;

int reciveObject(int arm = RIGHT_ARM,const std::string& object="Object")
{
    int final_arm ; // 0 = right and 1 = left
    std::stringstream ss;
    if( left_arm != EMPTY && right_arm != EMPTY)
    {
        printError("Ambos brazos ocupados ??");
        return -1;
    }

    if( arm == RIGHT_ARM )
        final_arm = right_arm == EMPTY ? RIGHT_ARM:LEFT_ARM;
    else
        final_arm = left_arm == EMPTY ? LEFT_ARM:RIGHT_ARM;

    if(final_arm == RIGHT_ARM)
    {
        if (!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::startRaGoTo("navigation");
                                    
        JustinaManip::startRaOpenGripper(0.3);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

        ss.str("");
        ss << "Sorry, please give me the " << object << " in my right gripper";
        JustinaHRI::waitAfterSay(ss.str(), 6000, MIN_DELAY_AFTER_SAY);
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

        JustinaManip::startRaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        JustinaHRI::waitAfterSay("Thank you.", 6000, MIN_DELAY_AFTER_SAY);
        return RIGHT_ARM;
    }
    else
    {
        if (!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::startLaGoTo("navigation");

        JustinaManip::startLaOpenGripper(0.3);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

        ss.str("");
        ss << "Sorry, please give me the " << object << "in my left gripper" ;
        JustinaHRI::waitAfterSay(ss.str(), 6000, MIN_DELAY_AFTER_SAY);
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

        JustinaManip::startLaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        JustinaHRI::waitAfterSay("Thank you.", 6000, MIN_DELAY_AFTER_SAY);
        return LEFT_ARM;
    }

}

    

int main(int argc, char **argv){

    ros::init(argc, argv, "serve_the_breakfast_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    vision_msgs::VisionObject objCutlery;

    //FLAGS
    bool flagOnce = true;
    bool success = false;
    bool withLeft = false;
    bool findSeat = false;
    bool doorOpenFlag = false;
    bool flagNoBowl= false;

    bool onlyBowl = false;

    //COUNTERS
    int countGraspAttemps = 0;
    int findSeatCount = 0;
    int attempsDoorOpend = 0;
    int attempsGrasp = 0;
    int findObjectAttemps = 0;
    int attempsBowlPour = 0;

    
    //SPEACH AND GRAMAR
    std::string param, typeOrder;
    std::vector<std::string> names;
    std::vector<std::string> drinks;
    std::string lastName, lastDrink;
    std::string grammarNamesID = "receptionistNames";
    std::string grammarDrinksID = "serve_the_breakfastDrinks";
    std::string grammarCommandsID = "serve_the_breakfastCommands";
    

    //LOCATIONS
    std::string recogLoc = "kitchen";
    std::string cutleryLoc = "kitchen_cabinet";
    std::string tableLoc = "kitchen_table";
    std::string tableDrinksLoc = "table_drinks_test";
    std::string cerealsLoc = "dishwasher";
    std::string milkLoc = "sideboard";


    //FOR GRASP OBJECTS (CUTLERY)
    vision_msgs::VisionObjectList my_cutlery;     
    my_cutlery.ObjectList.resize(8);  
    
    my_cutlery.ObjectList[0].id="green";    
    my_cutlery.ObjectList[1].id="lemon";
    my_cutlery.ObjectList[2].id="melon";
    my_cutlery.ObjectList[3].id="blue";
    my_cutlery.ObjectList[4].id="pink_1";
    my_cutlery.ObjectList[5].id="yellow";
    my_cutlery.ObjectList[6].id="red";
    my_cutlery.ObjectList[7].id="green_2";

    //FOR GRASP OBJECTS (CUTLERY)
    vision_msgs::VisionObjectList my_spoon;     
    my_spoon.ObjectList.resize(2);      
    
    my_spoon.ObjectList[0].id="red";
    my_spoon.ObjectList[1].id="green_2";

    std::vector<vision_msgs::VisionObject> recoObjForTake;
    std::vector<vision_msgs::VisionObject> recoObjList;

    int type;
    int graspObjectID = BOWL;
    std::string id_cutlery = "";
    std::string graspObject = " bowl "; // To say object, First Justina will take the bowl

// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!
    std::string idObjectGrasp = "chocolate_milk"; // !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!
    Eigen::Vector3d centroid;
    std::vector<Eigen::Vector3d> centroids;

    geometry_msgs::Pose poseCereal;
    
    std::stringstream ss;
    std::stringstream ss2;
    
    float dist_to_head;
    float distanceArm = 0.6;
    float goalx, goaly, goala;
    float robot_y, robot_x, robot_a;    
    float gx_w, gy_w, gz_w, guest_z, host_z;    
    float torsoSpine, torsoWaist, torsoShoulders;
    float pointingArmX, pointingArmY, pointingArmZ;
    float theta = 0, thetaToGoal = 0, angleHead = 0;
    float pointingDirX, pointingDirY, pointingDirZ, pointingNormal;
    geometry_msgs::Pose pose;
    
    std::vector<std::string> confirmCommands;
    confirmCommands.push_back("justina yes");
    confirmCommands.push_back("justina no");
    confirmCommands.push_back("robot yes");
    confirmCommands.push_back("robot no");

    std::vector<std::string> idsSeatTable;
    idsSeatTable.push_back("chair");
    idsSeatTable.push_back("table");
    
    boost::posix_time::ptime prev;
    boost::posix_time::ptime curr;
    
    std::vector<std::string> tokens;

    std::vector<vision_msgs::VisionObject> yoloObjects;

    bool findPerson = false;
    std::vector<std::string> idsPerson;
    idsPerson.push_back("person");
    std::string seatPlace = "kitchen";
    int attemptsSpeechReco,attemptsSpeechInt;
    bool recogName = false;
    bool once_name = true;
    int attempsName = 0;

    bool objectDetected = false;
    std::vector<vision_msgs::VisionObject> recoObj;
    recoObj = std::vector<vision_msgs::VisionObject>();
    sensor_msgs::Image image;
    int index;
    bool la = false;
    bool ra = false;
    bool drop = false;
    drinks.push_back("orange juice");
    names.push_back("William");


    JustinaHRI::setNodeHandle(&nh);
    JustinaTools::setNodeHandle(&nh);
    JustinaTasks::setNodeHandle(&nh);
    JustinaManip::setNodeHandle(&nh);
    JustinaVision::setNodeHandle(&nh);
    JustinaVision::setNodeHandle(&nh);
    JustinaHardware::setNodeHandle(&nh);
    JustinaKnowledge::setNodeHandle(&nh);
    JustinaNavigation::setNodeHandle(&nh);
    JustinaRepresentation::setNodeHandle(&nh);

    int countAlign;

    JustinaHRI::usePocketSphinx = false;
    STATE state = SM_INTRO_GUEST;//SM_DETECT_OBJECT;//SM_FIND_GUEST;
    //SM_GO_TO_TABLE_DRINKS;//SM_FIND_GUEST;//SM_FIND_BOWL; //SM_INIT;

    while(ros::ok() && !success){

        switch(state){
            case SM_INIT:
                
                printSmTitle("> State SM_INIT: Init the test.");
                
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaManip::startTorsoGoTo(0.10, 0, 0);
                JustinaManip::waitForTorsoGoalReached(3000);
                JustinaHRI::waitAfterSay("I am ready for the serve the breakfast test", 6000, MIN_DELAY_AFTER_SAY);
                
                state = SM_WAIT_FOR_OPEN;
                break;



            case SM_WAIT_FOR_OPEN:
                
                printSmTitle("> State SM_WAIT_FOR_OPEN: Wait for open the door.");
                
                while( !JustinaNavigation::doorIsOpen(0.9, 2000) && attempsDoorOpend < MAX_ATTEMPTS_DOOR   )
                {
                    JustinaHRI::waitAfterSay("Human, can you open the door please", 6000, MIN_DELAY_AFTER_SAY);
                    attempsDoorOpend++;
                }

                if(attempsDoorOpend > 0)
                    JustinaHRI::waitAfterSay("Thaks", 2000, MIN_DELAY_AFTER_SAY);
                    
                state = SM_NAVIGATE_TO_CUTLERY_LOC;
                break;

            case SM_NAVIGATE_TO_CUTLERY_LOC:
                
                printSmTitle("> State SM_NAVIGATE_TO_CUTLERY_LOC: Navigate to the kitchen.");
                
                ss.str("");
                ss << "I will navigate to the " << cutleryLoc ;
                JustinaHRI::waitAfterSay(ss.str(), 4000, MIN_DELAY_AFTER_SAY);

                if(!JustinaNavigation::getClose(cutleryLoc, 80000) )
                    JustinaNavigation::getClose(cutleryLoc, 80000); 

                ss.str("");
                ss << "I have reached to the " << cutleryLoc ;
                JustinaHRI::waitAfterSay(ss.str(), 4000, MIN_DELAY_AFTER_SAY);
                
                state = SM_FIND_BOWL;// SM_CHECK_IF_DOOR;
                break;

            case SM_CHECK_IF_DOOR:

                printSmTitle("> State SM_CHECK_IF_DOORS: Check if there is a door in front of cutlery.");

                if(false)
                {
                    JustinaHRI::waitAfterSay("Human, Could you open the cabinet door, please", 4000, MIN_DELAY_AFTER_SAY);
                    ros::Duration(5.0).sleep();
                    JustinaHRI::waitAfterSay("Thank you", 4000, MIN_DELAY_AFTER_SAY);
                }
                
                state = SM_FIND_BOWL;
                break;
            
/////////////////////////////////////////////////////////////////////////////////////

            case SM_FIND_GUEST:
                printSmTitle("> State SM_FIND_GUEST.");
                JustinaHRI::waitAfterSay("I'm going to find a person", 5000);
                if(findSeatCount < MAX_FIND_SEAT_COUNT){
                    centroids.clear();
                    findPerson = JustinaTasks::turnAndRecognizeYolo(idsPerson, JustinaTasks::NONE, -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2f, -0.2f, -0.3f, 0.1f, 0.1f, 9.0, centroids, seatPlace);
                    if(!findPerson){
                        findSeatCount++;
                        JustinaHRI::waitAfterSay("Sorry, I will try again", 5000);
                         break;
                    }
                    JustinaHRI::waitAfterSay("I found a person, I'm going to get close to you.", 5000);
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
                    JustinaTasks::closeToGoalWithDistanceTHR(goalx, goaly, 1.2, 30000);
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
                    state = SM_INTRO_GUEST;
                }
                else
                {
                    JustinaHRI::waitAfterSay("I am going to prepare the table.", 5000);
                    //state = ;
                }
                break;

            case SM_INTRO_GUEST:

                printSmTitle("> State SM_INTRO_GUEST.");

                lastName = "unknown";
                lastDrink = "unknown";
                JustinaHRI::enableSpeechRecognized(false);
                

                JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_NAMES);
//                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

                

                 if(  attempsName < MAX_ATTEMPTS_NAME)
                 {
                    if( once_name )
                    {
                        JustinaHRI::waitAfterSay("Hello, my name is Justina, please tell me, what is your name", 10000, MAX_DELAY_AFTER_SAY);
                        once_name = false;
                    }
                    else
                    {
                        JustinaHRI::waitAfterSay(" repeat your name", 10000, MAX_DELAY_AFTER_SAY);
                    }

                    JustinaHRI::enableSpeechRecognized(true);

                    if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, TIMEOUT_SPEECH))
                    {
                        if(JustinaRepresentation::stringInterpretation(lastRecoSpeech, lastInteSpeech))
                        {
                            if(JustinaRepresentation::receptionistInterpeted(lastInteSpeech, typeOrder, param))
                            {
                                ss.str("");
                                tokens.clear();
                                if(param.compare(" ") != 0 || param.compare("") != 0)
                                {
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
                                    ss << ", tell me justina yes or justina no";
                                    JustinaHRI::enableSpeechRecognized(false);
                                    JustinaHRI::waitAfterSay(ss.str(), 10000, MAX_DELAY_AFTER_SAY);
                                    JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_COMMANDS);
                                    JustinaHRI::enableSpeechRecognized(true);
                                    //attemptsConfirmation = 0;
                                    //attemptsWaitConfirmation = 0;
                                    JustinaHRI::enableSpeechRecognized(true);
                                    state = SM_WAIT_FOR_PRESENTATION;
                                }
                            }
                        }
                    }
                }
                else
                {
                    names.push_back(" ");
                    JustinaHRI::waitAfterSay("I coud't understand your name", 10000, MAX_DELAY_AFTER_SAY);
                    state = SM_ASK_DRINK;
                    attempsName = 0;
                }
                break;

            case SM_WAIT_FOR_PRESENTATION:
               
                printSmTitle("> State SM_WAIT_FOR_PRESENTATION.");
                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));


                if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, TIMEOUT_SPEECH))
                {
                    if(lastRecoSpeech.find("yes") != std::string::npos)
                    {
                        JustinaHRI::enableSpeechRecognized(false);
                        names.push_back(lastName);
                        ss2.str("");
                        ss2 << "Ok, your name is " << names[names.size() - 1];
                        JustinaHRI::waitAfterSay(ss2.str(), 6000, MAX_DELAY_AFTER_SAY);
                            
                        state = SM_ASK_DRINK;
                        attempsName = 0;
                    }
                    else
                    {
                        attempsName++;
                        JustinaHRI::waitAfterSay("Sorry. ", 10000, MAX_DELAY_AFTER_SAY);
                        state = SM_INTRO_GUEST;
                    }
                }

                break;


            case SM_ASK_DRINK:

                printSmTitle("> State SM_ASK_DRINK.");

                JustinaHRI::enableSpeechRecognized(false);
            
                JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_DRINKS);
               // boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                

                 if(  attempsName < MAX_ATTEMPTS_NAME)
                 {
                    JustinaHRI::waitAfterSay("please tell me, what is your favorite drink ", 10000, MAX_DELAY_AFTER_SAY);
                    JustinaHRI::enableSpeechRecognized(true);

                    if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, TIMEOUT_SPEECH))
                    {
                        //if(JustinaRepresentation::stringInterpretation(lastRecoSpeech, lastInteSpeech))
                        {
                            //if(JustinaRepresentation::receptionistInterpeted(lastInteSpeech, typeOrder, param))
                            {
                                ss.str("");
                                //tokens.clear();
                                //f(param.compare(" ") != 0 || param.compare("") != 0){
                                    ss << "Ok, your favorite drink is ";
                                    ss << lastRecoSpeech;
                                 //   boost::algorithm::split(tokens, param, boost::algorithm::is_any_of("_"));
                                   // ss2.str("");
                                    //for(int i = 0; i < tokens.size(); i++){
                                     //   ss << tokens[i] << " ";
                                       // ss2 << tokens[i];
                                        //if(i < tokens.size() -1)
                                          //  ss2 << " ";
                                    //}
                                    lastDrink = lastRecoSpeech;// ss2.str();
                                    //drinks.push_back(ss2.str());
                                    ss << ", tell me justina yes or justina no";
                                    JustinaHRI::enableSpeechRecognized(false);
                                    JustinaHRI::waitAfterSay(ss.str(), 10000, MAX_DELAY_AFTER_SAY);
                                    
                                    JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_COMMANDS);
                                    JustinaHRI::enableSpeechRecognized(true);
                                    //attemptsConfirmation = 0;
                                    //attemptsWaitConfirmation = 0;
                                    state = SM_CONFIMR_DRINK;
                                //}
                            }
                        }
                    }
                }
                else
                {
                    drinks.push_back("coke");
                    JustinaHRI::waitAfterSay("Sorry, I coud't understand your drink", 10000, MAX_DELAY_AFTER_SAY);
                    state = SM_SAY_ACTION;
                }
            break;

            case SM_CONFIMR_DRINK:
               
                printSmTitle("> State SM_WAIT_FOR_PRESENTATION.");
                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

                if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, TIMEOUT_SPEECH))
                {
                    if(lastRecoSpeech.find("yes") != std::string::npos)
                    {
                        JustinaHRI::enableSpeechRecognized(false);
                            std::replace( lastDrink.begin(), lastDrink.end(), ' ', '_');
                            drinks.push_back(lastDrink);
                            ss2.str("");
                            ss2 << "Ok, your favorite drink is " << drinks[drinks.size() - 1];
                            JustinaHRI::waitAfterSay(ss2.str(), 6000, MIN_DELAY_AFTER_SAY);
                            state = SM_SAY_ACTION;
                    }
                    else
                    {
                        attempsName++;
                        JustinaHRI::waitAfterSay("Sorry. ", 10000, MAX_DELAY_AFTER_SAY);
                        state = SM_ASK_DRINK;
                    }
                }
            break;

            case SM_SAY_ACTION:
                JustinaHRI::waitAfterSay(" I going to set the table for you, please wait.", 10000, MAX_DELAY_AFTER_SAY);

                state = SM_NAVIGATE_TO_CUTLERY_LOC;
            break;



////////////////////////////////////////////////////////////////////////////


            case SM_FIND_BOWL:
                
                printSmTitle("> SM_FIND_BOWL: Trying to detect a bowl");

                alignWithTable();

                findObjectAttemps = 0;
                JustinaManip::hdGoTo(0, -.8, 2000);

                ss.str("");
                ss << "I'm looking for a bowl"  ;
                JustinaHRI::waitAfterSay(ss.str(), 6000, MIN_DELAY_AFTER_SAY);
                
                while(!JustinaVision::getObjectSeg(my_cutlery) && findObjectAttemps < MAX_ATTEMPTS_FIND_BOWL)
                {
                    printWarning("Cant find cutlery trying again");
                    JustinaNavigation::moveDist(-0.05,3000);
                    findObjectAttemps++;
                }

                if( !(findObjectAttemps < MAX_ATTEMPTS_FIND_BOWL) )
                {
                    printError("The robot could not find object");

                    if (reciveObject(RIGHT_ARM,"bowl") == RIGHT_ARM )
                        right_arm = BOWL;
                    else
                        left_arm = BOWL;
                    
                    state = SM_FIND_SPOON;                
                }
                else
                {
                    std::cout << ".-> sorting the objects" << std::endl;
                    if(!JustinaTasks::sortObjectColor(my_cutlery))
                        if(!JustinaTasks::sortObjectColor(my_cutlery)) 

                    std::cout << ".-> selecting one object" << std::endl;

                    for(int i=0; i < my_cutlery.ObjectList.size(); i ++)
                    {
                            if(my_cutlery.ObjectList[i].graspable == true && my_cutlery.ObjectList[i].type_object == BOWL )
                            {
                                std::cout << ".-> detect the " << my_cutlery.ObjectList[i].id << " object" << std::endl;
                                pose.position.x = my_cutlery.ObjectList[i].pose.position.x;
                                pose.position.y = my_cutlery.ObjectList[i].pose.position.y;
                                pose.position.z = my_cutlery.ObjectList[i].pose.position.z;
                                id_cutlery = my_cutlery.ObjectList[i].id;
                                type = my_cutlery.ObjectList[i].type_object;
                                JustinaHRI::waitAfterSay("I've found the bowl" , 4000, MIN_DELAY_AFTER_SAY);
                                
                                break; 
                            }
                    }

                    if( id_cutlery == "" )
                    {
                        printError("The robot could not find object");

                        if (reciveObject(RIGHT_ARM,"bowl") == RIGHT_ARM )
                            right_arm = BOWL;
                        else
                            left_arm = BOWL;

                        state = SM_FIND_SPOON;
                    }
                    else
                    {
                        state = SM_TAKE_BOWL;
                    }
                }

                break;
    
            case SM_TAKE_BOWL:
                
                printSmTitle("> SM_TAKE_BOWL: Trying to take the bowl");

                withLeft = (pose.position.y > 0 ? true : false);

                countGraspAttemps = 0;
                
                while( countGraspAttemps < MAX_ATTEMPTS_GRASP )
                {
                    if(!graspObjectColorCupBoardFeedback2(pose.position.x, pose.position.y, pose.position.z, withLeft, id_cutlery, true))
                    {
                        printWarning("cannot take the object trying again");
                    }
                    else
                        break;

                    countGraspAttemps++;
                }

                if( !(countGraspAttemps < MAX_ATTEMPTS_GRASP) )
                {
                    printError("The robot could not take the bowl");

                    if(withLeft)
                    {
                        if (reciveObject(LEFT_ARM,"bowl") == LEFT_ARM )
                            left_arm = BOWL;
                        else
                            right_arm = BOWL;
                    }
                    else
                    {
                        if (reciveObject(RIGHT_ARM,"bowl") == RIGHT_ARM )
                            right_arm = BOWL;
                        else
                            left_arm = BOWL;
                    }
                    JustinaHRI::waitAfterSay("Thank you.", 4000, MIN_DELAY_AFTER_SAY);
                }
                else
                {
                    if ( withLeft)
                        left_arm = BOWL;
                    else
                        right_arm = BOWL;
                }

                state = SM_FIND_SPOON;
                break;

            case SM_FIND_SPOON:
                
                printSmTitle("> SM_FIND_SPOON: Trying to detect a spoon");

                alignWithTable();

                findObjectAttemps = 0;
                JustinaManip::hdGoTo(0, -.8, 2000);

                ss.str("");
                ss << "I'm looking for a spoon"  ;
                JustinaHRI::waitAfterSay(ss.str(), 6000, MIN_DELAY_AFTER_SAY);
                
                while(!JustinaVision::getObjectSeg(my_cutlery) && findObjectAttemps < MAX_ATTEMPTS_FIND_SPOON)
                {
                    printWarning("Cant find cutlery trying again");
                    JustinaNavigation::moveDist(-0.05,3000);
                    findObjectAttemps++;
                }

                if( !(findObjectAttemps < MAX_ATTEMPTS_FIND_SPOON) )
                {
                    printError("The robot could not find object");

                    if (reciveObject(RIGHT_ARM,"spoon") == RIGHT_ARM )
                        right_arm = CUTLERY;
                    else
                        left_arm = CUTLERY;
                    
                    state = SM_GO_TO_TABLE;            
                } 
                else
                {
                    std::cout << ".-> sorting the objects" << std::endl;
                    if(!JustinaTasks::sortObjectColor(my_cutlery))
                        if(!JustinaTasks::sortObjectColor(my_cutlery)) 

                    std::cout << ".-> selecting one object" << std::endl;
                    int i =0;
                    for( i=0; i < my_cutlery.ObjectList.size(); i ++)
                    {
                            if(my_cutlery.ObjectList[i].graspable == true && my_cutlery.ObjectList[i].type_object == CUTLERY )
                            {
                                std::cout << ".-> detect the " << my_cutlery.ObjectList[i].id << " object" << std::endl;
                                pose.position.x = my_cutlery.ObjectList[i].pose.position.x;
                                pose.position.y = my_cutlery.ObjectList[i].pose.position.y;
                                pose.position.z = my_cutlery.ObjectList[i].pose.position.z;
                                id_cutlery = my_cutlery.ObjectList[i].id;
                                type = my_cutlery.ObjectList[i].type_object;
                                objCutlery = my_cutlery.ObjectList[i];
                                
                                JustinaHRI::waitAfterSay("I've found a spoon" , 4000, MIN_DELAY_AFTER_SAY);
                                
                                break;
                            }
                    }

                    if( id_cutlery == "" )
                    {
                        printError("The robot could not find object");

                        if ( reciveObject(RIGHT_ARM,"spoon") == RIGHT_ARM )
                            right_arm = CUTLERY;
                        else
                            left_arm = CUTLERY;

                        state = SM_GO_TO_TABLE; 
                    }
                    else
                    {
                        state = SM_TAKE_SPOON;
                    }
                }

                break;

            case SM_TAKE_SPOON:
                
                printSmTitle("> SM_TAKE_SPOON: Trying to take the spoon");

                countGraspAttemps = 0;

                if(left_arm == EMPTY)
                    withLeft = true;
                else
                    withLeft = false; 
                
                while( countGraspAttemps < MAX_ATTEMPTS_GRASP )
                {
                    //if(!JustinaTasks::graspObjectColorFeedback(objCutlery, false, id_cutlery, true))
                    if(!graspObjectColorCupBoardFeedback2(pose.position.x, pose.position.y, pose.position.z, withLeft, id_cutlery, true))
                    {
                        printWarning("cannot take the object trying again");
                    }
                    else
                        break;

                    countGraspAttemps++;
                }

                if( !(countGraspAttemps < MAX_ATTEMPTS_GRASP) )
                {
                    printError("The robot could not take the bowl");

                    if(withLeft)
                    {
                        if (reciveObject(LEFT_ARM,"spoon") == LEFT_ARM )
                            left_arm = CUTLERY;
                        else
                            right_arm = CUTLERY;
                    }
                    else
                    {
                        if (reciveObject(RIGHT_ARM,"spoon") == RIGHT_ARM )
                            right_arm = CUTLERY;
                        else
                            left_arm = CUTLERY;
                    }
                    JustinaHRI::waitAfterSay("Thank you.", 4000, MIN_DELAY_AFTER_SAY);
                }

                state = SM_GO_TO_TABLE;
                break;


            case SM_GO_TO_TABLE:

                printSmTitle("> SM_GO_TO_TABLE: go to table");

                if(!JustinaNavigation::getClose(tableLoc, 80000) )
                    JustinaNavigation::getClose(tableLoc, 80000); 
                JustinaHRI::waitAfterSay("I have reached the table.", 4000, MIN_DELAY_AFTER_SAY);

                state = SM_PLACE_BOWL;
                
            break;

            case SM_PLACE_BOWL:

                printSmTitle("> SM_PLACE_BOWL: place bowl");

                JustinaHRI::waitAfterSay("I'm going to place the bowl and spoon", 4000, MIN_DELAY_AFTER_SAY);
                
                alignWithTable();
                std::cout <<  "RIGT¨¨¨ "<< right_arm << std::endl;
                if(!placeObject( left_arm == BOWL ? true : false ))
                    state = SM_PLACE_BOWL;
                else
                    state = SM_PLACE_SPOON;

                if (left_arm == BOWL)
                    left_arm = EMPTY;
                else
                    right_arm = EMPTY;

                
                state = SM_GO_TO_TABLE_DRINKS;
                //exit(0);
                //JustinaNavigation::moveDistAngle(-0.3, 0, 2000);
                //JustinaManip::startTorsoGoTo(0.10, 0, 0);
                //JustinaManip::waitForTorsoGoalReached(3000);

            break;

            case SM_GO_TO_TABLE_DRINKS:

                printSmTitle("> SM_GO_TO_TABLE: go to table_drinks");

                if(!JustinaNavigation::getClose(tableDrinksLoc, 80000) )
                    JustinaNavigation::getClose(tableDrinksLoc, 80000); 
                JustinaHRI::waitAfterSay("I have reached the table drinks .", 4000, MIN_DELAY_AFTER_SAY);

                state = SM_DETECT_OBJECT;
                
            break;


            case SM_DETECT_OBJECT:


                printSmTitle("> SM_DETECT_OBJECT");
                
                alignWithTable();

                {
                    ss.str("");
                    ss << "I am looking for the " << drinks[drinks.size() - 1] << " on the table";
                    JustinaHRI::waitAfterSay(ss.str(), 5000);
                    //Obtiene la lista de objetos a detectar
                    //recoObj = std::vector<vision_msgs::VisionObject>();

                    objectDetected = false;
                    //Detecta los objetos en la mesa
                    if(JustinaVision::detectAllObjectsVot(recoObj, image, 5)){
                        for(int j = 0; j < recoObj.size() && !objectDetected; j++){
                            // id.compare es la lista de objetos a leer, en este caso es cocacola
                            if (recoObj[j].id.compare(drinks[drinks.size() - 1]) == 0){
                                index = j;
                                objectDetected = true;
                            }
                        }
                    } 
                }   
                state = (objectDetected) ? SM_GRASP_OBJECT : SM_HANDLER;
                
            break;

            case SM_GRASP_OBJECT:
                std::cout << "State machine: SM_GRASP_OBJECT" << std::endl;
                if(objectDetected && recoObj.size() > 0){
                    ss.str("");
                    ss << "I have found the " << drinks[drinks.size() - 1];
                    JustinaHRI::waitAfterSay(ss.str(), 5000);
                    //JustinaTasks::alignWithTable(0.35);
                    ss.str("");
                    ss << "I am going to take the " << drinks[drinks.size() - 1];
                    JustinaHRI::waitAfterSay(ss.str(), 5000);
                    // This is for grasp with two frames //false for right true for left, "", true torso 
                    //std::cout << "Index: " << index << std::endl;
                    //std::cout << "recoObj: " << recoObj.size() << std::endl;

                    if(recoObj[index].pose.position.y > 0)
                        ra = false;
                    else
                        ra = true;

                    if (ra){
                        JustinaTasks::graspObject(recoObj[index].pose.position.x, recoObj[index].pose.position.y, recoObj[index].pose.position.z, false, "", true);
                        drop = true;
                    }
                    else{
                        JustinaTasks::graspObject(recoObj[index].pose.position.x, recoObj[index].pose.position.y, recoObj[index].pose.position.z, true, "", true);
                        drop = false;                       
                    }

                }
                state = SM_RETURN_TO_TABLE;       
                break;

            case SM_HANDLER:
                JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 5000);
                JustinaNavigation::startMoveDist(-0.15);
                std::cout << "State machine: SM_HANDLER" << std::endl;
                ss.str("");
                ss << "Sorry i could not grasp the " << drinks[drinks.size() - 1] << ", please put the " << drinks[drinks.size() - 1] << " in my gripper";
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                if(drop){
                    JustinaManip::raGoTo("navigation", 3000);
                    JustinaTasks::detectObjectInGripper(drinks[drinks.size() - 1], false, 7000);
                }
                else{
                    JustinaManip::laGoTo("navigation", 3000);
                    JustinaTasks::detectObjectInGripper(drinks[drinks.size() - 1], true, 7000);
                }

                state = SM_RETURN_TO_TABLE;
                
                break;

            case SM_RETURN_TO_TABLE:
                

                printSmTitle("> SM_RETURN_TO_TABLE:  ");

                JustinaHRI::waitAfterSay("I'm going to the table", 4000, MIN_DELAY_AFTER_SAY);
                
                if(!JustinaNavigation::getClose(tableLoc, 80000) )
                    JustinaNavigation::getClose(tableLoc, 80000); 

                alignWithTable();
                JustinaTasks::placeObject(!ra);
                JustinaManip::laGoTo("home", 5000);
                JustinaManip::raGoTo("home", 5000);
                JustinaNavigation::moveDistAngle(-.2,0,3000);
                JustinaNavigation::moveDistAngle(0,3.1,6000);

                ss.str("");
                ss << names[names.size()-1] << "  the table is ready. i have finish the test, thank you" ;
                JustinaHRI::waitAfterSay(ss.str(), 5000);



                exit(0);
            break;


            case SM_PLACE_SPOON:

                printSmTitle("> SM_PLACE_SPOON: place spoon");

                if(right_arm == CUTLERY || left_arm == CUTLERY)
                {
                    state = SM_GO_FOR_CEREAL;
                    break;
                }    

                JustinaHRI::waitAfterSay("I'm going to place the spoon", 4000, MIN_DELAY_AFTER_SAY);
                if( !placeObject(left_arm == CUTLERY ? true : false) )
                {
                    printWarning(".-> Can not detect any object" );;
                    JustinaTasks::placeObject(withLeft);
                    state = SM_GO_FOR_CEREAL;
                }

                /*
                //alignWithTable();

                //JustinaManip::hdGoTo(0, -.8, 2000);

                id_cutlery == "";

                if(!JustinaVision::getObjectSeg(my_cutlery))
                {
                    
                }
                else
                {
                    std::cout << ".-> sorting the objects" << std::endl;
                    if(!JustinaTasks::sortObjectColor(my_cutlery))
                        if(!JustinaTasks::sortObjectColor(my_cutlery)) 

                    std::cout << ".-> selecting one object" << std::endl;

                    for(int i=0; i < my_cutlery.ObjectList.size(); i ++)
                    {
                        if(my_cutlery.ObjectList[i].graspable == true && ( my_cutlery.ObjectList[i].type_object == BOWL ) )
                        {
                            std::cout << ".-> detect the " << my_cutlery.ObjectList[i].id << " object" << std::endl;
                            pose.position.x = my_cutlery.ObjectList[i].pose.position.x;
                            pose.position.y = my_cutlery.ObjectList[i].pose.position.y;
                            pose.position.z = my_cutlery.ObjectList[i].pose.position.z;
                            id_cutlery = my_cutlery.ObjectList[i].id;
                            type = my_cutlery.ObjectList[i].type_object;
                            break;
                        }
                    } 
                }

                if(id_cutlery == "")
                {
                    printWarning(".-> Can not detect any object" );;
                    JustinaTasks::placeObject(withLeft);
                    state = SM_GO_FOR_CEREAL;
                }
                else
                {
                    if( !placeSpoon(pose.position.x, pose.position.y, pose.position.z, left_arm == CUTLERY ? true : false, id_cutlery, true) )
                    {
                        printWarning(".-> Can not detect any object" );;
                        JustinaTasks::placeObject(withLeft);
                        state = SM_GO_FOR_CEREAL;
                    }
                }
                */
            
                JustinaNavigation::moveDistAngle(-0.3, 0, 2000);
                
                state = SM_GO_FOR_CEREAL;
            break;

            case SM_GO_FOR_CEREAL:
                printSmTitle("> SM_GO_FOR_CEREAL: go for ceral");

                JustinaHRI::waitAfterSay("I'm going to the dish washer", 4000, MIN_DELAY_AFTER_SAY);
                
                if(!JustinaNavigation::getClose(cerealsLoc, 80000) )
                    JustinaNavigation::getClose(cerealsLoc, 80000); 

                JustinaHRI::waitAfterSay("I have reached.", 4000, MIN_DELAY_AFTER_SAY);
                
                state = SM_TAKE_CEREAL;
            break;

            case SM_TAKE_CEREAL:

                printSmTitle("> SM_TAKE_CEREAL: take cereal");

                if (!JustinaManip::isRaInPredefPos("navigation"))
                    JustinaManip::startRaGoTo("navigation");
                else
                    std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;

                JustinaManip::startRaOpenGripper(0.3);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

                JustinaHRI::waitAfterSay("Human please put the cereals in my right  gripper", 4000, MIN_DELAY_AFTER_SAY);
                boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
                
                JustinaManip::startRaCloseGripper(0.5);
                JustinaHRI::waitAfterSay("Thank you.", 4000, MIN_DELAY_AFTER_SAY);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

                
                JustinaManip::startTorsoGoTo(0.10, 0, 0);
                JustinaManip::waitForTorsoGoalReached(3000);
                state = SM_RETURN_TO_TABLE;

            break;
            

            case SM_SEARCH_BOWL:

                std::cout << ".-> trying to detect the objects" << std::endl;

                attempsBowlPour = 0;
                graspObjectID = BOWL;
                
                alignWithTable();

                JustinaManip::hdGoTo(0, -.8, 2000);
                if( !JustinaVision::getObjectSeg(my_cutlery) && attempsBowlPour++ < 3 )
                {
                        JustinaNavigation::moveDist(-0.05,3000);
                        std::cout << ".-> Can not detect any object" << std::endl;
                        state = SM_SEARCH_BOWL;
                }
                else
                {
                    std::cout << ".-> sorting the objects" << std::endl;
                        if(!JustinaTasks::sortObjectColor(my_cutlery))
                            if(!JustinaTasks::sortObjectColor(my_cutlery)) 

                        std::cout << ".-> selecting one object" << std::endl;

                        for(int i=0; i < my_cutlery.ObjectList.size(); i ++)
                        {
                            if(my_cutlery.ObjectList[i].graspable == true && (my_cutlery.ObjectList[i].type_object == graspObjectID || my_cutlery.ObjectList[i].type_object == 3 ) )
                            {
                                std::cout << ".-> detect the " << my_cutlery.ObjectList[i].id << " object" << std::endl;
                                pose.position.x = my_cutlery.ObjectList[i].pose.position.x;
                                pose.position.y = my_cutlery.ObjectList[i].pose.position.y;
                                pose.position.z = my_cutlery.ObjectList[i].pose.position.z;
                                id_cutlery = my_cutlery.ObjectList[i].id;
                                type = my_cutlery.ObjectList[i].type_object;
                                state = SM_POURING_CEREAL;
                                break;
                            }
                        } 
                    }
                break;

            case SM_POURING_CEREAL:


                countGraspAttemps = 0;
                
                JustinaHRI::waitAfterSay("I am going to pour the cereal carefully inside the bowl.", 6000, MIN_DELAY_AFTER_SAY);
                while(countGraspAttemps++ <= MAX_ATTEMPTS_GRASP )
                {
                    if(!pouringCereal(pose.position.x, pose.position.y, pose.position.z, false, id_cutlery, true))
                    {
                            std::cout << ".-> cannot take the object" << std::endl;
                    }
                    else
                        break;
                }
                state = SM_LEAVE_CEREAL;
            break;

            case SM_LEAVE_CEREAL:
                
              
                JustinaTasks::placeObject(false);
                JustinaHRI::waitAfterSay("I'm going for the milk ", 4000, MIN_DELAY_AFTER_SAY);
                
                if(!JustinaNavigation::getClose(milkLoc, 80000) )
                    JustinaNavigation::getClose(milkLoc, 80000); 
                state = SM_TAKE_MILK;
            break;

            case SM_TAKE_MILK:
                alignWithTable();
                JustinaHRI::waitAfterSay("I'm going to take the milk", 4000, MIN_DELAY_AFTER_SAY);
                
                if(JustinaTasks::findObject(idObjectGrasp, poseCereal, withLeft) )
                {
                    
                    JustinaTasks::graspObject(poseCereal.position.x, poseCereal.position.y, poseCereal.position.z, withLeft, idObjectGrasp, true, false);
                    state = SM_PLACE_MILK;
                }else
                {
                    state = SM_TAKE_MILK;
                }

            break;

            case SM_PLACE_MILK:
                JustinaHRI::waitAfterSay("I'm going to the table", 4000, MIN_DELAY_AFTER_SAY);
                
                if(!JustinaNavigation::getClose(tableLoc, 80000) )
                    JustinaNavigation::getClose(tableLoc, 80000); 

                ////alignWithTable();
                JustinaTasks::placeObject(withLeft);


                state = SM_FINISH_TEST;

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



bool graspObjectColorCupBoardFeedback2(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse) {
    //y += 0.5;
    //withLeftArm = false;
    float idealX = 0.5;
    int n_movements_bowl = 6;
    bool found = false;
    int typeCutlery;
    int waitTime = 3000;
    float objToGraspX = x;
    float objToGraspY = y;
    float objToGraspZ = z;
    float idealY = withLeftArm ? 0.225 : -0.255; //It is the distance from the center of the robot, to the center of the arm
    float torsoSpine, torsoWaist, torsoShoulders;
    float movTorsoFromCurrPos;
    float movLateral = -(idealY - objToGraspY);
    float distance_bowl[7][3] = {   {-0.10,-0.25,0.0},
                                    {-0.03,-0.20,0.0},
                                    {-0.02,-0.05,0.0},
                                    {0.0,0.0,0.0},
                                    {-0.03,-0.20,0.0},
                                    {-0.10,-0.25,0.0},
                                    {-0.07,-0.20,0.0}
                                     };
    vision_msgs::VisionObjectList objects;
    vision_msgs::VisionObject object_aux;
    std::vector<float> articular;
    std::stringstream ss; ss.str("");
    
    if( z > 1.1)
    {
        idealX = 0.65;
    }else
    {
        idealX = 0.5;
    }


    if (withLeftArm) 
    {
        std::cout << "Using left arm" << std::endl;
        if (!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::startLaGoTo("navigation");
        JustinaHRI::waitAfterSay("I am going to take an object with my left arm", 4000, 0);
    }
    else 
    {
        std::cout << "Using right arm" << std::endl;
        if (!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::startRaGoTo("navigation");
        JustinaHRI::waitAfterSay("I am going to take an object with my right arm", 4000, 0);
    }

    std::cout << "object x: " << x << "object y" << y << "object z" << z << std::endl;

    std::cout << "Adjusting pose to grasp object ..." << std::endl;
    JustinaNavigation::moveLateral(movLateral, 3000);
    
    JustinaNavigation::moveDist(  x > idealX ? x - idealX : -(idealX -x) ,3000);

    std::cout << "Adjusting height to grasp object" << std::endl;
    if ( objToGraspZ > 1.1 )
    {
        JustinaManip::startTorsoGoTo(0.30, 0, 0);
        JustinaManip::waitForTorsoGoalReached(4000);
    }else
    {
        JustinaManip::startTorsoGoTo(0.20, 0, 0);
        JustinaManip::waitForTorsoGoalReached(4000);
    }
    
    std::cout << "Searching the object again" << std::endl; 
    if (colorObject.compare("") != 0) 
    {
        JustinaManip::hdGoTo(0, -0.8, 2000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        object_aux.id = colorObject;
        objects.ObjectList.push_back(object_aux);
        found = JustinaVision::getObjectSeg(objects);
        printSuccess("The object was found again: ");
    }
    else
    {
        printError("Sorry could not find the object again.");
        return false;
    }


    if (found && objects.ObjectList[0].graspable) 
    {
        std::cout << "The object was found again, update the new coordinates." << std::endl;
        typeCutlery = objects.ObjectList.at(0).type_object;
        
        switch (typeCutlery) 
        {
            case 0: //Cutlery objects
                objToGraspX = objects.ObjectList.at(0).pose.position.x;
                objToGraspY = objects.ObjectList.at(0).pose.position.y;
                objToGraspZ = objects.ObjectList.at(0).minPoint.z  +0.27 ;//+ currentTorso - lastTorso ;
                break;
            
            case 1: // This for bowls
                objToGraspX = objects.ObjectList.at(0).pose.position.x ;
                if(withLeftArm)
                    objToGraspY = objects.ObjectList.at(0).pose.position.y-0.05;
                else
                    objToGraspY = objects.ObjectList.at(0).pose.position.y;
                //if( z > 1.1)
                  //  objToGraspZ =  objects.ObjectList.at(0).minPoint.z   ;
                //else
                    objToGraspZ =  objects.ObjectList.at(0).minPoint.z +0.17 ;// 15 si no esta en el borde 18 si esta en el borde
                break;
            default:
                break;
        }
        std::cout << "Final X: " << objToGraspX << "Final y: " << objToGraspY << "Final z: " << objToGraspZ << std::endl;
    }

    //The position it is adjusted and converted to coords wrt to the corresponding arm
    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";
    if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY, objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) 
    {
        printError("JustinaTasks.->Cannot transform point. ");
        return false;
    }

    std::cout << "FINAL FINAL X: " << objToGraspX << "Final y: " << objToGraspY << "Final z: " << objToGraspZ << std::endl;

        if ( typeCutlery == 0 ) 
        {
            if( withLeftArm ) 
                JustinaManip::laGoToCartesianTraj(objToGraspX , objToGraspY, objToGraspZ, 8000);
            else
                JustinaManip::raGoToCartesianTraj(objToGraspX, objToGraspY, objToGraspZ, 8000);
            
            if( withLeftArm ) 
                JustinaManip::startLaOpenGripper(0.3);
            else
                JustinaManip::startRaOpenGripper(0.3);

            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
            
            if( withLeftArm )
            {
                JustinaManip::laGoToCartesian(objToGraspX + 0.03, objToGraspY - 0.08, objToGraspZ,0.0, 0.0, 1.5708, -0.1, 5000);
                JustinaManip::laGoToCartesian(objToGraspX + 0.03, objToGraspY - 0.08, objToGraspZ,objects.ObjectList[0].roll, objects.ObjectList[0].pitch,objects.ObjectList[0].yaw, -0.1, 5000);
            }
            else
            {
                
                JustinaManip::raGoToCartesian(objToGraspX + 0.03, objToGraspY - 0.08, objToGraspZ,0.0, 0.0, 1.5708, -0.1, 5000);
                JustinaManip::raGoToCartesian(objToGraspX + 0.03, objToGraspY - 0.08, objToGraspZ,objects.ObjectList[0].roll, objects.ObjectList[0].pitch,objects.ObjectList[0].yaw, -0.1, 5000);
            }
            
            JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
            JustinaManip::startTorsoGoTo(torsoSpine-.03, 0, 0);
            JustinaManip::waitForTorsoGoalReached(waitTime);

            if(withLeftArm )
                JustinaManip::startLaCloseGripper(0.5);
            else
                JustinaManip::startRaCloseGripper(0.5);
            
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

            JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
            JustinaManip::startTorsoGoTo(torsoSpine+.05, 0, 0);
            JustinaManip::waitForTorsoGoalReached(waitTime);
            JustinaNavigation::moveDist(-.3, 2000);
        }
        else if (typeCutlery == 1 ) 
        {


            if( withLeftArm ) 
                JustinaManip::startLaOpenGripper(0.5); 
            else
                JustinaManip::startRaOpenGripper(0.5);


            if ( z > 1.1 )
            {
                objToGraspX = objToGraspX-0.02;
                if(withLeftArm)
                {
                    JustinaManip::laGoTo("take_bowl_1", 3500);
                    JustinaManip::laGoTo("take_bowl_2", 3500);
                }
                else
                {
                    JustinaManip::raGoTo("take_bowl_1", 3500);
                    JustinaManip::raGoTo("take_bowl_2", 3500);
                }
                
                articular.clear();
                if(JustinaManip::inverseKinematics(objToGraspX , objToGraspY-0.1 , objToGraspZ , articular))
                {
                    if( withLeftArm )
                        JustinaManip::startLaGoToArticular(articular);
                    else
                        JustinaManip::startRaGoToArticular(articular);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));

                    if(JustinaManip::inverseKinematics(objToGraspX , objToGraspY-.06 , objToGraspZ , articular))
                    {
                        if( withLeftArm )
                            JustinaManip::startLaGoToArticular(articular);
                        else
                            JustinaManip::startRaGoToArticular(articular);
                            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

                        boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    }

                }else
                {
                    printError("fail");
                    JustinaManip::laGoToCartesianTraj(objToGraspX , objToGraspY - 0.0, objToGraspZ, 5000);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
            
                }
                
                if( withLeftArm ){
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                    JustinaManip::startLaCloseGripper(0.5);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                }
                else
                {
                    printWarning(" * Aqui toi perro");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

                    JustinaManip::startRaCloseGripper(0.5);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                }

                if(withLeftArm)
                {
                    JustinaManip::laGoTo("take_bowl_2", 3500);
                    JustinaManip::laGoTo("take_bowl_1", 3500);
                }
                else
                {
                    JustinaManip::raGoTo("take_bowl_2", 3500);
                    JustinaManip::raGoTo("take_bowl_1", 3500);
                }
                

                //exit(0);
            }
            
            else
            {

                for(int i = 0; i < n_movements_bowl; ++i)
                {
                    articular.clear();
                    if(JustinaManip::inverseKinematics(objToGraspX + distance_bowl[i][0], objToGraspY + distance_bowl[i][1], objToGraspZ + distance_bowl[i][2] , articular))
                    {
                        std::cout << "Execuying move " << i << std::endl;
                        if(i != 0) 
                            if(withLeftArm)
                                JustinaManip::waitForLaGoalReached(2500);
                            else
                                JustinaManip::waitForRaGoalReached(2500);
                        
                        if( withLeftArm )
                            JustinaManip::startLaGoToArticular(articular);
                        else
                            JustinaManip::startRaGoToArticular(articular);

                        boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    }else
                    {
                        ss.str("");
                        ss <<  "Can not achive movement: " << i << " X: " << distance_bowl[i][0] << " Y: " << distance_bowl[i][1] << " Z: " << distance_bowl[i][2];
                        printWarning(ss.str() );
                    }
                    if(i == 3)
                    {   
                        if( withLeftArm )
                            JustinaManip::startLaCloseGripper(0.5);
                        else
                        {
                            printWarning(" * Aqui toi perro");
                            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                            JustinaManip::startRaCloseGripper(0.5);
                            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                        }
                    }
                }
            }
            
        }

        if( withLeftArm )
            JustinaManip::laGoTo("navigation", 3500);
        else
            JustinaManip::raGoTo("navigation", 3500);

        JustinaManip::startTorsoGoTo(0.1, 0, 0);
        JustinaManip::waitForTorsoGoalReached(waitTime);
        ros::spinOnce();
        printSuccess( "The object was successfully grasp.");
        
        return true;
}





bool placeSpoon(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse) 
{
    std::cout << "JustinaTasks.->Moving to a good-pose for grasping objects with ";
    if (withLeftArm) {
        std::cout << "left arm" << std::endl;
        if (!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::startLaGoTo("navigation");
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
    } else {
        std::cout << "right arm" << std::endl;
        if (!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::startRaGoTo("navigation");
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;
    }


    float idealX = 0.50;
    float idealY = withLeftArm ? 0.225 : -0.255; //It is the distance from the center of the robot, to the center of the arm
    float idealZ = 0.60; //It is the ideal height for taking an object when torso is at zero height.
    float missingZ = 0.0;
    float minTorso = 0.08;
    float maxTorso = 0.294;

    float torsoSpine, torsoWaist, torsoShoulders;
    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
    std::cout << "JustinaTasks.->torsoSpine:" << torsoSpine << std::endl;

    int typeCutlery;
    float objToGraspX = x;
    float objToGraspY = y;
    float objToGraspZ = z;
    float dz = 0.0;
    int maxIteration = 10;
    float movTorsoFromCurrPos;
    std::cout << "JustinaTasks.->ObjToGrasp: " << "  " << objToGraspX << "  " << objToGraspY << "  " << objToGraspZ << std::endl;
    float movFrontal = -(idealX - objToGraspX);
    float movLateral = -(idealY - objToGraspY);
    float movVertical = objToGraspZ - idealZ - torsoSpine;
    float goalTorso = torsoSpine + movVertical;
    std::cout << "JustinaTasks.->goalTorso:" << goalTorso << std::endl;
    int waitTime;

    if (goalTorso < minTorso)
    {
        missingZ = minTorso - goalTorso;
        goalTorso = minTorso;
    }if (goalTorso > maxTorso)
        goalTorso = maxTorso;

    movTorsoFromCurrPos = goalTorso - torsoSpine;
    waitTime = (int) (8000 * fabs(movTorsoFromCurrPos) / 0.3);
    std::cout << "JustinaTasks.->movTorsoFromCurrPos:" << movTorsoFromCurrPos<< std::endl;
    std::cout << "JustinaTasks.->goalTorso:" << goalTorso << std::endl;
    std::cout << "JustinaTasks.->waitTime:" << waitTime << std::endl;
    std::cout << "JustinaTasks.->Adjusting with frontal=" << movFrontal << " lateral=" << movLateral << " and vertical=" << movVertical << std::endl;
    float lastRobotX, lastRobotY, lastRobotTheta;
    JustinaNavigation::getRobotPoseFromOdom(lastRobotX, lastRobotY, lastRobotTheta);
    
    if (usingTorse)
        JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
    JustinaNavigation::moveLateral(movLateral/2, 3000);
    JustinaNavigation::moveDist(movFrontal, 3000);

    if (usingTorse)
        JustinaManip::waitForTorsoGoalReached(waitTime);

    bool found = false;
    vision_msgs::VisionObjectList objects;
    vision_msgs::VisionObject object_aux;
    
    int indexFound = 0;
    if (colorObject.compare("") != 0) {
        JustinaManip::hdGoTo(0, -0.9, 2000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        object_aux.id = colorObject;
        objects.ObjectList.push_back(object_aux);
        found = JustinaVision::getObjectSeg(objects);
        std::cout << "GET OBJECTS: " << found << std::endl;
    }

    if (found && objects.ObjectList[0].graspable) 
    {
        std::cout << "The object was found again, update the new coordinates." << std::endl;
        typeCutlery = objects.ObjectList.at(0).type_object;
        
        objToGraspX = objects.ObjectList.at(0).pose.position.x;

        if (withLeftArm)
            objToGraspY = objects.ObjectList.at(0).pose.position.y + 0.2;
        else
            objToGraspY = objects.ObjectList.at(0).pose.position.y - 0.2;

        objToGraspZ = objects.ObjectList.at(0).minPoint.z + 0.3;
        dz = minTorso;
            
    }
        std::cout << "MaxPoint en z:" << objToGraspZ << std::endl;
    

    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";
    if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY, objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) 
    {
        std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
        return false;
    }
    
    if( withLeftArm ) 
    {
        
        if (!JustinaManip::isLaInPredefPos("put1"))
                JustinaManip::laGoTo("put1", 2000);
            else
                std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
    
        //JustinaManip::laGoToCartesian(objToGraspX, objToGraspY, objToGraspZ, 0.0, 0.0, 1.5708, 0.0, 5000);
        JustinaManip::laGoToCartesian(objToGraspX-0.04 , objToGraspY , objToGraspZ,0.0,0.0,0.0, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(400));
        JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
        JustinaManip::startTorsoGoTo(torsoSpine-.08, 0, 0);
        JustinaManip::waitForTorsoGoalReached(waitTime);
        JustinaManip::startLaOpenGripper(0.3);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

        JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
        JustinaManip::startTorsoGoTo(torsoSpine+.08, 0, 0);
        JustinaManip::waitForTorsoGoalReached(waitTime);

        if (!JustinaManip::isLaInPredefPos("put1"))
            JustinaManip::laGoTo("put1", 3000);
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
    

        ros::spinOnce();
    } 
    else 
    {
        if (!JustinaManip::isRaInPredefPos("put1"))
            JustinaManip::raGoTo("put1", 2000);
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
    
        if(!JustinaManip::raGoToCartesian(objToGraspX-.04, objToGraspY, objToGraspZ, 0.0, 0.0, 0, 0.0, 5000))
            JustinaManip::raGoToCartesianTraj(objToGraspX-.04 , objToGraspY , objToGraspZ, 5000); 

        //JustinaManip::raGoToCartesian(objToGraspX , objToGraspY , objToGraspZ,0.0,0.0,0.0, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
        JustinaManip::startTorsoGoTo(torsoSpine-.08, 0, 0);
        JustinaManip::waitForTorsoGoalReached(waitTime);

        JustinaManip::startRaOpenGripper(0.3);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

        JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
        JustinaManip::startTorsoGoTo(torsoSpine+.08, 0, 0);
        JustinaManip::waitForTorsoGoalReached(waitTime);

        if (!JustinaManip::isRaInPredefPos("put1"))
            JustinaManip::raGoTo("put1", 3000);
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
    

        ros::spinOnce();
    }
   
    if (!JustinaManip::isLaInPredefPos("navigation"))
                    JustinaManip::startLaGoTo("navigation");
    if (!JustinaManip::isRaInPredefPos("navigation"))
                    JustinaManip::startRaGoTo("navigation");

    JustinaManip::waitForLaGoalReached(1500);
    JustinaManip::waitForRaGoalReached(1500);

    

    JustinaManip::startTorsoGoTo(0.1, 0, 0);
    JustinaManip::waitForTorsoGoalReached(waitTime);

    return true;
}



bool pouringCereal(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse) 
{
    std::cout << "JustinaTasks.->Moving to a good-pose for grasping objects with ";
    
    if (withLeftArm) {
        std::cout << "left arm" << std::endl;
        if (!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::startLaGoTo("navigation");
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
    } else {
        std::cout << "right arm" << std::endl;
        if (!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::startRaGoTo("navigation");
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;
    }


    float idealX = 0.50;
    float idealY = withLeftArm ? 0.225 : -0.255; //It is the distance from the center of the robot, to the center of the arm
    float idealZ = 0.60; //It is the ideal height for taking an object when torso is at zero height.
    float missingZ = 0.0;
    float minTorso = 0.08;
    float maxTorso = 0.294;

    float torsoSpine, torsoWaist, torsoShoulders;
    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
    std::cout << "JustinaTasks.->torsoSpine:" << torsoSpine << std::endl;

    int typeCutlery;
    float objToGraspX = x;
    float objToGraspY = y;
    float objToGraspZ = z;
    float dz = 0.0;
    int maxIteration = 10;
    float movTorsoFromCurrPos;
    std::cout << "JustinaTasks.->ObjToGrasp: " << "  " << objToGraspX << "  " << objToGraspY << "  " << objToGraspZ << std::endl;
    float movFrontal = -(idealX - objToGraspX);
    float movLateral = -(idealY - objToGraspY);
    float movVertical = objToGraspZ - idealZ - torsoSpine;
    float goalTorso = torsoSpine + movVertical;
    std::cout << "JustinaTasks.->goalTorso:" << goalTorso << std::endl;
    int waitTime;

    if (goalTorso < minTorso)
    {
        missingZ = minTorso - goalTorso;
        goalTorso = minTorso;
    }if (goalTorso > maxTorso)
        goalTorso = maxTorso;

    movTorsoFromCurrPos = goalTorso - torsoSpine;
    waitTime = (int) (8000 * fabs(movTorsoFromCurrPos) / 0.3);
    std::cout << "JustinaTasks.->movTorsoFromCurrPos:" << movTorsoFromCurrPos<< std::endl;
    std::cout << "JustinaTasks.->goalTorso:" << goalTorso << std::endl;
    std::cout << "JustinaTasks.->waitTime:" << waitTime << std::endl;
    std::cout << "JustinaTasks.->Adjusting with frontal=" << movFrontal << " lateral=" << movLateral << " and vertical=" << movVertical << std::endl;
    float lastRobotX, lastRobotY, lastRobotTheta;
    JustinaNavigation::getRobotPoseFromOdom(lastRobotX, lastRobotY, lastRobotTheta);
    
    if (usingTorse)
        JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
    JustinaNavigation::moveLateral(movLateral, 3000);
    JustinaNavigation::moveDist(movFrontal, 3000);

    if (usingTorse)
        JustinaManip::waitForTorsoGoalReached(waitTime);

    bool found = false;
    vision_msgs::VisionObjectList objects;
    vision_msgs::VisionObject object_aux;
    
    int indexFound = 0;
    if (colorObject.compare("") != 0) {
        JustinaManip::hdGoTo(0, -0.9, 2000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        object_aux.id = colorObject;
        objects.ObjectList.push_back(object_aux);
        //found = JustinaVision::getObjectSeg(objects);
        std::cout << "GET OBJECTS: " << found << std::endl;
    }

    if (true)//found && objects.ObjectList[0].graspable) 
    {
        std::cout << "The object was found again, update the new coordinates." << std::endl;
        typeCutlery = objects.ObjectList.at(0).type_object;
        
        objToGraspX = x +.30;// objects.ObjectList.at(0).pose.position.x + .3;
        if (withLeftArm)
            objToGraspY = y+.225;// objects.ObjectList.at(0).maxPoint.y - .25;
        else
            objToGraspY = y-0.225;//objects.ObjectList.at(0).minPoint.y + .25;

        objToGraspZ = z+0.3;//objects.ObjectList.at(0).minPoint.z + 0.45;
        dz = minTorso;
            
    }
        std::cout << "MaxPoint en z:" << objToGraspZ << std::endl;
    

    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";
    if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY, objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) 
    {
        std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
        return false;
    }
    
    if( withLeftArm ) 
    {
        JustinaManip::laGoToCartesianTraj(objToGraspX , objToGraspY , objToGraspZ, 5000);          

        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        //JustinaManip::laGoToCartesian(objToGraspX -.15, objToGraspY , objToGraspZ ,0.0, 0.0, 1.5707, 0.1, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(400));
        std::vector<float> currPose;
        JustinaManip::getLaCurrentPos(currPose);
        if (currPose.size() == 7) 
        {
            while( (currPose[0] += 0.1 ) < 1.4)
            {
                JustinaManip::laGoToArticular(currPose, 2000);
                //JustinaManip::waitForLaGoalReached(900);   
            }

            while( (currPose[6] += 0.1) > 0)
            {
                currPose[2] -= 0.1;
                JustinaManip::raGoToArticular(currPose,2000);   
            }

            while( (currPose[2] -= 0.1) > -1.6)
            {
                JustinaManip::raGoToArticular(currPose,2000);   
            }

            while( (currPose[5] += 0.1) > 1.8)
            {
                JustinaManip::raGoToArticular(currPose,2000);   
            }
        
        }
        ros::spinOnce();
    } 
    else 
    {
        //JustinaManip::laGoToCartesianTraj(objToGraspX , objToGraspY, objToGraspZ+0.1, 15000);
        //ustinaManip::laStopGoToCartesian();
        //boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        //ros::spinOnce();


        //JustinaManip::raGoToCartesian(objToGraspX -.15, objToGraspY , objToGraspZ+.08 ,0.0, 0.0, 1.5707, 0.1, 5000);

        std::cout << "Cords: x: " << objToGraspX << " y: " << objToGraspY   << " z: " <<  objToGraspZ << std::endl;
        JustinaManip::raGoToCartesianTraj(objToGraspX , objToGraspY , objToGraspZ, 5000);          
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        ros::spinOnce();

        std::vector<float> currPose;
        JustinaManip::getRaCurrentPos(currPose);

        JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
        JustinaManip::startTorsoGoTo(torsoSpine+.4, 0, 0);
        JustinaManip::waitForTorsoGoalReached(waitTime);

        JustinaNavigation::moveDist(0.15,3000);

        if (currPose.size() == 7) 
        {
            //while( (currPose[4] += 0.1) < 1.4)
           

            currPose[4] += currPose[2];
            
            currPose[6] = 0;
                JustinaManip::raGoToArticular(currPose,2000); 
                ros::spinOnce();

            while( (currPose[5] -= 0.4) > -1.295) // -0.95
            {
                JustinaManip::raGoToArticular(currPose,2000);  
                ros::spinOnce(); 
            }

            boost::this_thread::sleep(boost::posix_time::milliseconds(2500));

            while( (currPose[5] += 0.4) < 0)
            {
                JustinaManip::raGoToArticular(currPose,2000);  
                ros::spinOnce(); 
            }

            /*
            while( (currPose[6] += 0.1) < 1.4)
            {
                currPose[2] -= 0.1;
                JustinaManip::raGoToArticular(currPose,200); 
                ros::spinOnce();  
            }

            while( (currPose[2] -= 0.1) > -1.6)
            {
                JustinaManip::raGoToArticular(currPose,200);  
                ros::spinOnce(); 
            }

            while( (currPose[5] += 0.1) < 1.5)
            {
                JustinaManip::raGoToArticular(currPose,200); 
                ros::spinOnce();  
            }

            while( (currPose[2] -= 0.1) > -3.0)
            {
                JustinaManip::raGoToArticular(currPose,200);  
                ros::spinOnce(); 
            }

            boost::this_thread::sleep(boost::posix_time::milliseconds(2500));
        
            while( (currPose[2] += 0.1) < -1.6)
            {
                JustinaManip::raGoToArticular(currPose,200);  
                ros::spinOnce(); 
            }

            while( (currPose[5] -= 0.1) > 0)
            {
                JustinaManip::raGoToArticular(currPose,200); 
                ros::spinOnce();  
            }

            while( (currPose[2] += 0.1) < 0)
            {
                JustinaManip::raGoToArticular(currPose,200);  
                ros::spinOnce(); 
            }

            while( (currPose[6] -= 0.1) > 0)
            {
                
                JustinaManip::raGoToArticular(currPose,200); 
                ros::spinOnce();  
            }*/
        
        }
        




    }

    JustinaNavigation::moveDist(-.3, 3000);

    if (!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::startLaGoTo("navigation");
    if (!JustinaManip::isRaInPredefPos("navigation"))
           JustinaManip::startRaGoTo("navigation");

    JustinaManip::startTorsoGoTo(0.1, 0, 0);
    JustinaManip::waitForTorsoGoalReached(waitTime);

    return true;

}



bool placeObject(bool withLeftArm, float h, bool placeBag) {
    std::cout << "JustinaTasks::placeObject..." << std::endl;
    std::vector<float> vacantPlane;
    std::vector<int> inliers;
    std::vector<int> inliersRight;
    std::vector<int> inliersLeft;
    std::vector<float> xRight;
    std::vector<float> yRight;
    std::vector<float> zRight;
    std::vector<float> xLeft;
    std::vector<float> yLeft;
    std::vector<float> zLeft;
    std::vector<float> distance;
    float maximunInliers = 0;
    float objToGraspX;
    float objToGraspY;
    float objToGraspZ;
    float lateral;

    int maxInliersIndex;

    int aux = 0;
    int contLeft = 0;
    int contRight = 0;

    JustinaManip::hdGoTo(0, -0.7, 5000);
    if (!JustinaTasks::alignWithTable(0.32))
        JustinaTasks::alignWithTable(0.32);

    if (!JustinaVision::findVacantPlane(vacantPlane, inliers)) {
        JustinaNavigation::moveDist(0.04, 1000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
        if (!JustinaVision::findVacantPlane(vacantPlane, inliers)) {
            JustinaNavigation::moveDist(-0.06, 1000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
            if (!JustinaTasks::alignWithTable(0.32)) {
                if (!JustinaVision::findVacantPlane(vacantPlane, inliers))
                    return false;
            } else {
                if (!JustinaVision::findVacantPlane(vacantPlane, inliers))
                    return false;
            }
        }
    }

    for (int i = 0; i < (vacantPlane.size()); i = i + 3) {
        if (vacantPlane[i + 1] >= 0.0) {
            xLeft.push_back(vacantPlane[i]);
            yLeft.push_back(vacantPlane[i + 1]);
            zLeft.push_back(vacantPlane[i + 2]);
            inliersLeft.push_back(inliers[aux]);
            contLeft++;
            std::cout << "Justina::Tasks->PlaceObject plano lado izquierdo "
                << std::endl;
        } else {
            xRight.push_back(vacantPlane[i]);
            yRight.push_back(vacantPlane[i + 1]);
            zRight.push_back(vacantPlane[i + 2]);
            inliersRight.push_back(inliers[aux]);
            contRight++;
            std::cout << "Justina::Tasks->PlaceObject plano lado derecho "
                << std::endl;
        }
        aux++;
    }

    if (contLeft == 0 && contRight > 0) {
        std::cout
            << "Justina::Tasks->PlaceObject: No hay planos libres del lado izquierdo, se usaran los del derecho "
            << std::endl;
        for (int i = 0; i < xRight.size(); i++) {
            xLeft.push_back(xRight[i]);
            yLeft.push_back(yRight[i]);
            zLeft.push_back(zRight[i]);
            inliersLeft.push_back(inliersRight[i]);
        }
    } else if (contRight == 0 && contLeft > 0) {
        std::cout
            << "Justina::Tasks->PlaceObject: No hay planos libres del lado derecho, se usaran los del izquierdo "
            << std::endl;
        for (int i = 0; i < xLeft.size(); i++) {
            xRight.push_back(xLeft[i]);
            yRight.push_back(yLeft[i]);
            zRight.push_back(zLeft[i]);
            inliersRight.push_back(inliersLeft[i]);
        }
    } else if (contLeft == 0 && contRight == 0)
        std::cout << "Justina::Tasks->PlaceObject: No hay planos libres"
            << std::endl;
    else
        std::cout
            << "Justina::Tasks->PlaceObject: Planos libres en el lado izquierdo y derecho"
            << std::endl;

    if (withLeftArm) {
        for (int i = 0; i < xLeft.size(); i++) {
            if (inliersLeft[i] > maximunInliers) {
                maximunInliers = inliersLeft[i];
                maxInliersIndex = i;
            }
        }
        std::cout << "Justina::Tasks->PlaceObject  P_max[" << maxInliersIndex
            << "]:  (" << xLeft[maxInliersIndex] << ", "
            << yLeft[maxInliersIndex] << ", " << zLeft[maxInliersIndex]
            << " + " << h << ")" << std::endl;
        std::cout << "Justina::Tasks->PlaceObject  inliers_max["
            << maxInliersIndex << "]:  " << inliersLeft[maxInliersIndex]
            << std::endl;
    } else {
        for (int i = 0; i < xRight.size(); i++) {
            if (inliersRight[i] > maximunInliers) {
                maximunInliers = inliersRight[i];
                maxInliersIndex = i;
            }
        }
        std::cout << "Justina::Tasks->PlaceObject  P_max[" << maxInliersIndex
            << "]:  (" << xRight[maxInliersIndex] << ", "
            << yRight[maxInliersIndex] << ", " << zRight[maxInliersIndex]
            << " + " << h << ")" << std::endl;
        std::cout << "Justina::Tasks->PlaceObject  inliers_max["
            << maxInliersIndex << "]:  " << inliersRight[maxInliersIndex]
            << std::endl;
    }

    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";

    if (withLeftArm) {
        lateral = yLeft[maxInliersIndex] - 0.225;
        JustinaNavigation::moveLateral(lateral, 3000);
        yLeft[maxInliersIndex] = 0.22;
        if (!JustinaTools::transformPoint("base_link", xLeft[maxInliersIndex],
                    yLeft[maxInliersIndex],
                    zLeft[maxInliersIndex] + (zLeft[maxInliersIndex] * 0.05) + h,
                    destFrame, objToGraspX, objToGraspY, objToGraspZ)) {
            std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
            return false;
        }
        std::cout << "Moving left arm to P[wrtr]:  (" << xLeft[maxInliersIndex]
            << ", " << yLeft[maxInliersIndex] << ", "
            << zLeft[maxInliersIndex] + (zLeft[maxInliersIndex] * 0.05) + h
            << ")" << std::endl;
        if (!JustinaManip::isLaInPredefPos("navigation")) {
            std::cout << "Left Arm is not already on navigation position"
                << std::endl;
            JustinaManip::laGoTo("navigation", 7000);
        }

        // Verify if the height of plane is longer than 1.2 if not calculate the
        // inverse kinematic.
        if (zLeft[maxInliersIndex] > 1.2) {
            

        } else {
            JustinaManip::laGoTo("put1", 6000);
            if (placeBag) {

            } else
            {   
                JustinaManip::raGoTo("navigation", 7000);
                JustinaManip::raGoToCartesianTraj(objToGraspX-0.1 , objToGraspY, objToGraspZ, 8000);
                JustinaManip::laGoToCartesian(objToGraspX, objToGraspY,objToGraspZ + 0.1, 0, 0, 1.5708, 0, 5000);

                JustinaManip::raGoToCartesian(objToGraspX + 0.03-0.2, objToGraspY - 0.0, objToGraspZ - 0.15,0.0, 0.0, 1.5708, -0.1, 5000);
                JustinaManip::raGoToCartesian(objToGraspX + 0.03-0.1, objToGraspY - 0.0, objToGraspZ - 0.15,0.0, 0.0,0.0, -0.1, 5000);
            }


            std::cout << "Moving left arm to P[wrta]:  (" << objToGraspX << ", "
                << objToGraspY << ", " << objToGraspZ << ")" << std::endl;
            if (placeBag) {
               
            } else {
                //JustinaNavigation::moveDist(0.05, 1000);
                JustinaManip::startLaOpenGripper(0.5);
                JustinaManip::startRaOpenGripper(0.5);
                ros::spinOnce();
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));

                JustinaNavigation::moveDist(-0.2, 5000);
                JustinaManip::laGoTo("navigation", 5000);
                JustinaManip::raGoTo("navigation", 5000);
                JustinaManip::startLaOpenGripper(0.0);
                JustinaManip::startRaOpenGripper(0.0);
                JustinaManip::laGoTo("home", 5000);
                JustinaManip::raGoTo("home", 5000);
                

                //JustinaManip::startLaGoTo("home");
                JustinaManip::startHdGoTo(0.0, 0.0);
            }

        }
    } else {
        lateral = yRight[maxInliersIndex] + 0.225;
        JustinaNavigation::moveLateral(lateral, 3000);
        yRight[maxInliersIndex] = -0.22;
        if (!JustinaTools::transformPoint("base_link", xRight[maxInliersIndex],
                    yRight[maxInliersIndex],
                    zRight[maxInliersIndex] + (zRight[maxInliersIndex] * 0.05) + h,
                    destFrame, objToGraspX, objToGraspY, objToGraspZ)) {
            std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
            return false;
        }
        std::cout << "Moving right arm to P[wrtr]:  ("
            << xRight[maxInliersIndex] << ", " << yRight[maxInliersIndex]
            << ", "
            << zRight[maxInliersIndex] + (zRight[maxInliersIndex] * 0.05)
            + h << ")" << std::endl;
        if (!JustinaManip::isRaInPredefPos("navigation")) {
            std::cout << "Right Arm is not already on navigation position"
                << std::endl;
            JustinaManip::raGoTo("navigation", 7000);
        }

        if (zRight[maxInliersIndex] > 1.2) {
          
        } else {
            JustinaManip::raGoTo("put1", 6000);
            if (placeBag) {
               
            } else
            {   JustinaManip::laGoTo("navigation", 7000);
                JustinaManip::laGoToCartesianTraj(objToGraspX -0.1, objToGraspY, objToGraspZ, 8000);
                JustinaManip::raGoToCartesian(objToGraspX, objToGraspY,objToGraspZ -0.1, 0, 0, 1.5708, 0, 5000);
                JustinaManip::laGoToCartesian(objToGraspX + 0.03-0.2, objToGraspY - 0.0, objToGraspZ + 0.15,0.0, 0.0, 1.5708, -0.1, 5000);
                JustinaManip::laGoToCartesian(objToGraspX + 0.03-0.1, objToGraspY - 0.0, objToGraspZ + 0.15,0.0,0.0,0.0, -0.1, 5000);
            }

            std::cout << "Moving right arm to P[wrta]:  (" << objToGraspX
                << ", " << objToGraspY << ", " << objToGraspZ << ")"
                << std::endl;
            if (placeBag) {
               
            } else {
                //JustinaNavigation::moveDist(0.05, 1000);
                JustinaManip::startRaOpenGripper(0.5);
                JustinaManip::startLaOpenGripper(0.5);
                ros::spinOnce();
                boost::this_thread::sleep(
                        boost::posix_time::milliseconds(1000));

                JustinaNavigation::moveDist(-0.2, 5000);
                JustinaManip::raGoTo("navigation", 5000);
                JustinaManip::laGoTo("navigation", 5000);
                JustinaManip::startRaOpenGripper(0.0);
                JustinaManip::startLaOpenGripper(0.0);
                JustinaManip::raGoTo("home", 5000);
                JustinaManip::laGoTo("home", 5000);


                //JustinaManip::startRaGoTo("home");
                JustinaManip::startHdGoTo(0.0, 0.0);
            }
        }
    }

    return true;
}