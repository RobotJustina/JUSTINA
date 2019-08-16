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
#define MAX_ATTEMPTS_FIND_BOWL 4
#define MAX_ATTEMPTS_FIND_SPOON 2

bool graspObjectColorCupBoardFeedback2(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);
bool pouringCereal(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);
bool placeSpoon(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);

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
    SM_GO_TO_TABLE
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
    std::cout << "\x1b[1;32m "  << test << input << " n_n \x1b[0m" << std::endl;
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


    JustinaHRI::usePocketSphinx = true;
    STATE state = SM_FIND_BOWL; //SM_INIT;

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
                
                state = SM_CHECK_IF_DOOR;
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
            
            case SM_FIND_BOWL:
                
                printSmTitle("> SM_FIND_BOWL: Trying to detect a bowl");

                //alignWithTable();

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

                    for(int i=0; i < my_cutlery.ObjectList.size(); i ++)
                    {
                            if(my_cutlery.ObjectList[i].graspable == true && my_cutlery.ObjectList[i].type_object == CUTLERY )
                            {
                                std::cout << ".-> detect the " << my_cutlery.ObjectList[i].id << " object" << std::endl;
                                pose.position.x = my_cutlery.ObjectList[i].pose.position.x;
                                pose.position.y = my_cutlery.ObjectList[i].pose.position.y;
                                pose.position.z = my_cutlery.ObjectList[i].pose.position.z;
                                id_cutlery = my_cutlery.ObjectList[i].id;
                                type = my_cutlery.ObjectList[i].type_object;
                                
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

                JustinaHRI::waitAfterSay("I'm going to place the bowl", 4000, MIN_DELAY_AFTER_SAY);
                
                alignWithTable();
                std::cout <<  "RIGT¨¨¨ "<< right_arm << std::endl;
                if(!JustinaTasks::placeObject( left_arm == BOWL ? true : false ))
                    state = SM_PLACE_BOWL;
                else
                    state = SM_PLACE_SPOON;

                JustinaNavigation::moveDistAngle(-0.3, 0, 2000);
                JustinaManip::startTorsoGoTo(0.10, 0, 0);
                JustinaManip::waitForTorsoGoalReached(3000);

                
            break;

            case SM_PLACE_SPOON:

                printSmTitle("> SM_PLACE_SPOON: place spoon");

                if(right_arm == CUTLERY || left_arm == CUTLERY)
                {
                    state = SM_GO_FOR_CEREAL;
                    break;
                }    

                JustinaHRI::waitAfterSay("I'm going to place the spoon", 4000, MIN_DELAY_AFTER_SAY);

                alignWithTable();

                JustinaManip::hdGoTo(0, -.8, 2000);

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
            case SM_RETURN_TO_TABLE:

                printSmTitle("> SM_RETURN_TO_TABLE:  ");

                JustinaHRI::waitAfterSay("I'm going to the table", 4000, MIN_DELAY_AFTER_SAY);
                
                if(!JustinaNavigation::getClose(tableLoc, 80000) )
                    JustinaNavigation::getClose(tableLoc, 80000); 

                
                state = SM_SEARCH_BOWL;
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
    int kk=0;
    std::cout << "JustinaTasks.->Moving to a good-pose for grasping objects with ";
    if (withLeftArm) {
        std::cout << "left arm" << std::endl;
        if (!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::startLaGoTo("navigation");
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
        JustinaHRI::waitAfterSay("I am going to take an object with my left arm", 4000, 0);
    } else {
        std::cout << "right arm" << std::endl;
        if (!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::startRaGoTo("navigation");
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;
        JustinaHRI::waitAfterSay("I am going to take an object with my right arm", 4000, 0);
    }

    std::cout << "object x: " << x << "object y" << y << "object z" << z << std::endl;


    std::stringstream ss;
    ss.str("");

    int waitTime = 3000;

    float idealX = 0.50;
    float idealY = withLeftArm ? 0.225 : -0.255; //It is the distance from the center of the robot, to the center of the arm

    float torsoSpine, torsoWaist, torsoShoulders;

    int typeCutlery;
    float objToGraspX = x;
    float objToGraspY = y;
    float objToGraspZ = z;
    float dz = 0.0;
    int maxIteration = 10;
    float movTorsoFromCurrPos;
    std::cout << "JustinaTasks.->ObjToGrasp: " << "  " << objToGraspX << "  " << objToGraspY << "  " << objToGraspZ << std::endl;
    float movLateral = -(idealY - objToGraspY);

    float lastRobotX, lastRobotY, lastRobotTheta;
    JustinaNavigation::getRobotPoseFromOdom(lastRobotX, lastRobotY, lastRobotTheta);
        
    JustinaNavigation::moveLateral(movLateral, 3000);

    //JustinaManip::startTorsoGoTo(.20, 0, 0);
        JustinaManip::waitForTorsoGoalReached(4000);

    if ( z > 1.1 )
    {
        JustinaManip::startTorsoGoTo(.25, 0, 0);
        JustinaManip::waitForTorsoGoalReached(4000);

    }else
    {
        JustinaManip::startTorsoGoTo(.15, 0, 0);
        JustinaManip::waitForTorsoGoalReached(4000);
    }

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
        
        switch (typeCutlery) 
        {
            //Cutlery objects
            case 0:
                objToGraspX = objects.ObjectList.at(0).pose.position.x;
                objToGraspY = objects.ObjectList.at(0).pose.position.y;
                objToGraspZ = objects.ObjectList.at(0).minPoint.z  + .3 ;//+ currentTorso - lastTorso ;
                break;
            // This for bowls
            case 1:
            case 3:
                objToGraspX = objects.ObjectList.at(0).pose.position.x + 0.03 ;
                if (withLeftArm)
                    objToGraspY = objects.ObjectList.at(0).pose.position.y;//maxPoint.y;
                else
                    objToGraspY = objects.ObjectList.at(0).pose.position.y;//minPoint.y;

                objToGraspZ =  objects.ObjectList.at(0).minPoint.z +0.2 ;// + currentTorso -lastTorso;
                break;
            default:
                break;
        }
        std::cout << "Final X: " << objToGraspX << "Final y: " << objToGraspY << "Final z: " << objToGraspZ << std::endl;
    }
    else
    {
        return false;
    }

    //The position it is adjusted and converted to coords wrt to the corresponding arm
    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";
    if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY, objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) 
    {
        std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
        return false;
    }

    //objToGraspX -=0.1;
    std::cout << "FFINAL FINAL X: " << objToGraspX << "Final y: " << objToGraspY << "Final z: " << objToGraspZ << std::endl;


    std::cout << "JustinaTasks.->Moving ";
    if (withLeftArm)
        std::cout << "left arm";
    else
        std::cout << "right arm";
    std::cout << " to " << objToGraspX << "  " << objToGraspY << "  " << objToGraspZ << std::endl;

    
    if( withLeftArm ) {
        if ( typeCutlery == 0 ) 
        {
            JustinaManip::laGoToCartesianTraj(objToGraspX + 0.03, objToGraspY - 0.02, objToGraspZ, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

            JustinaManip::startLaOpenGripper(0.3);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
            
            JustinaManip::laGoToCartesian(objToGraspX + 0.00, objToGraspY + 0.0, objToGraspZ,objects.ObjectList[0].roll, objects.ObjectList[0].pitch,objects.ObjectList[0].yaw, 0.1, 5000);
            
            JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
            if ( usingTorse )
                JustinaManip::startTorsoGoTo(torsoSpine-.07, 0, 0);
                JustinaManip::waitForTorsoGoalReached(waitTime);

            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

            JustinaManip::startLaCloseGripper(0.5);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

            JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
            if ( usingTorse )
                JustinaManip::startTorsoGoTo(torsoSpine+.2, 0, 0);
            
            if ( usingTorse )
            JustinaManip::waitForTorsoGoalReached(waitTime+3000);

        JustinaNavigation::moveDist(-.3, 2000);

        }
        else if (typeCutlery == 1 ) 
        {
            std::vector<float> articular;            
            
            if(JustinaManip::inverseKinematics(objToGraspX - 0.12, objToGraspY - 0.25, objToGraspZ, articular))
            {   
                JustinaManip::startLaGoToArticular(articular);

                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            JustinaManip::startLaOpenGripper(0.5);


            articular.clear();
            if(JustinaManip::inverseKinematics(objToGraspX - 0.06, objToGraspY - 0.15, objToGraspZ, articular)){
                JustinaManip::waitForLaGoalReached(3500);
                JustinaManip::startLaGoToArticular(articular);
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            }

            articular.clear();
            if(JustinaManip::inverseKinematics(objToGraspX - 0.06, objToGraspY + 0.0, objToGraspZ, articular)){
                JustinaManip::waitForLaGoalReached(3500);
                JustinaManip::startLaGoToArticular(articular);
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            }
            JustinaManip::waitForLaGoalReached(3500);

            articular.clear();
            if(JustinaManip::inverseKinematics(objToGraspX - 0.06, objToGraspY - 0.0, objToGraspZ, articular)){
                JustinaManip::waitForLaGoalReached(3500);
                JustinaManip::startLaGoToArticular(articular);
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            }
                
            }
            else
            {
                std::cout << "JustinaTask.->Can not calculate inverse kinematics." << std::endl;
                JustinaManip::laGoToCartesianTraj(objToGraspX - 0.05, objToGraspY - 0.02, objToGraspZ, 5000);
            }
            
            
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        ros::spinOnce();

        //

        JustinaManip::startLaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaManip::startTorsoGoTo(0.2, 0, 0);
        JustinaManip::waitForTorsoGoalReached(waitTime);

        JustinaNavigation::moveDist(-0.2,3000);

        JustinaManip::laGoTo("navigation", 3500);
           
        JustinaManip::startTorsoGoTo(0.1, 0, 0);
        JustinaManip::waitForTorsoGoalReached(waitTime);
        
            if (!JustinaVision::isStillOnTable(objects.ObjectList.at(0))) {
                JustinaNavigation::moveDist(-0.35, 3000);
                std::cout << "The object was grasp with the left arm in the first test" << std::endl;
                return true;
            }
             
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            ros::spinOnce();
        //}
        std::cout << "The object was not grasp with the left arm" << std::endl;
        return false;


    } else 
    {
        if ( typeCutlery == 0) 
        {

            //JustinaManip::raGoToCartesian(objToGraspX + 0.03, objToGraspY - 0.02, objToGraspZ,0.0, 0.0, 1.5708, 0.1, 5000);          
            JustinaManip::raGoToCartesianTraj(objToGraspX + 0.03, objToGraspY - 0.02, objToGraspZ, 5000);          

            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

            JustinaManip::startRaOpenGripper(0.3);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
            
            JustinaManip::raGoToCartesian(objToGraspX + 0.00, objToGraspY + 0.0, objToGraspZ,objects.ObjectList[0].roll, objects.ObjectList[0].pitch,objects.ObjectList[0].yaw, 0.1, 5000);
            
            JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
            if ( usingTorse )
                JustinaManip::startTorsoGoTo(torsoSpine-.07, 0, 0);
            JustinaManip::waitForTorsoGoalReached(waitTime);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

            JustinaManip::startRaCloseGripper(0.5);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));


            JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
            if ( usingTorse )
                JustinaManip::startTorsoGoTo(torsoSpine+.2, 0, 0);

            
            if ( usingTorse )
            JustinaManip::waitForTorsoGoalReached(waitTime+3000);
        JustinaNavigation::moveDist(-.3, 3000);

        
        }
        else if (typeCutlery == 1 ) 
        {
            std::vector<float> articular;            
            
            float distance_bowl[7][3] = {
                                            {-0.12,-0.25,0.0},
                                            {-0.03,-0.15,0.0},
                                            {-0.02,-0.05,0.0},
                                            {0.0,0.0,0.0},
                                            {-0.02,-0.05,0.0},
                                            {-0.03,-0.15,0.0},
                                            {-0.12,-0.25,0.0}
                                        };
            JustinaManip::startRaOpenGripper(0.5); 
            for(int i = 0; i < 7; ++i)
            {
                articular.clear();
                if(JustinaManip::inverseKinematics(objToGraspX + distance_bowl[i][0], objToGraspY + distance_bowl[i][1], objToGraspZ + distance_bowl[i][2] , articular)){
                    std::cout << "Execuying move " << i << std::endl;
                    if(i != 0) JustinaManip::waitForRaGoalReached(2500);
                    JustinaManip::startRaGoToArticular(articular);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                }else
                {
                    ss.str("");
                    ss <<  "Can not achive movement: " << i << " X: " << distance_bowl[i][0] << " Y: " << distance_bowl[i][1] << " Z: " << distance_bowl[i][2];
                    printWarning(ss.str() );
                }
                if(i==3)JustinaManip::startRaCloseGripper(0.5);

            }

            exit(0);
            /*
            if(JustinaManip::inverseKinematics(objToGraspX - 0.12, objToGraspY - 0.25, objToGraspZ, articular))
            {
                JustinaManip::startRaGoToArticular(articular);
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                

                articular.clear();
                if(JustinaManip::inverseKinematics(objToGraspX -0.03, objToGraspY - 0.15, objToGraspZ, articular)){
                    std::cout << "Execuying move A" << std::endl;
                    JustinaManip::waitForRaGoalReached(2500);
                    JustinaManip::startRaGoToArticular(articular);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                }else
                {std::cout << "a" << std::endl;}
                
                articular.clear();
                if(JustinaManip::inverseKinematics(objToGraspX -0.02, objToGraspY -0.05, objToGraspZ, articular)){
                    std::cout << "Execuying move B" << std::endl;
                    JustinaManip::waitForRaGoalReached(2500);
                    JustinaManip::startRaGoToArticular(articular);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                }else
                {std::cout << "b" << std::endl;}

                JustinaManip::startRaOpenGripper(0.5);

                articular.clear();
                if(JustinaManip::inverseKinematics(objToGraspX , objToGraspY -0.0 , objToGraspZ, articular)){
                    std::cout << "Execuying move C" << std::endl;
                    JustinaManip::waitForRaGoalReached(3500);
                    JustinaManip::startRaGoToArticular(articular);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                }else
                {std::cout << "c" << std::endl;}

                JustinaManip::waitForRaGoalReached(2500);
                
                exit(0);
            }
            else
            {    
               // JustinaManip::raGoToCartesianTraj(objToGraspX - 0.05, objToGraspY - 0.0, objToGraspZ, 5000);          
                std::cout << "JustinaTask.->Can not calculate inverse kinematics." << std::endl;
            }*/
            

        }
        
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        ros::spinOnce();
        JustinaNavigation::moveDist(.03,3000);
        
        JustinaManip::startRaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        
        JustinaManip::startTorsoGoTo(0.2, 0, 0);
        JustinaManip::waitForTorsoGoalReached(waitTime);
        JustinaNavigation::moveDist(.2,3000);
        JustinaManip::raGoTo("navigation", 2000);
    
        JustinaManip::startTorsoGoTo(0.1, 0, 0);
        JustinaManip::waitForTorsoGoalReached(waitTime);
        
        if (!JustinaVision::isStillOnTable(objects.ObjectList.at(0))) {
            JustinaNavigation::moveDist(-0.35, 3000);
            std::cout << "The object was grasp with the left arm in the first test" << std::endl;
            return true;
        }
             
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        ros::spinOnce();

        std::cout << "The object was not grasp with the right arm" << std::endl;
        return false;
    }
    return false;
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