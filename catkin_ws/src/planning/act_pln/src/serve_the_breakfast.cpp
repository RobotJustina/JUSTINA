#include <iostream>
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
#define MAX_ATTEMPTS_ALIGN 3
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

bool graspObjectColorCupBoardFeedback2(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);
bool pouringCereal(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);
bool placeSpoon(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);

enum TYPE_CULTLERY{CUTLERY, BOWL, DISH, GLASS};

enum STATE{
    SM_INIT,
    SM_FINISH_TEST,
    SM_WAIT_FOR_OPEN,
    SM_NAVIGATE_TO_TABLEWARE,
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
    SM_POURING_CEREAL
};

std::string lastRecoSpeech;
std::string lastInteSpeech;

std::string test("serve the breakfast");

bool alignWithTable()
{
    int countAlign = 0;
    while(!JustinaTasks::alignWithTable(0.42))
    {   
        std::cout << ".-> Can not align with table." << std::endl;
        if(countAlign++ < MAX_ATTEMPTS_ALIGN)
        {
            JustinaNavigation::moveDistAngle(0.1, 0, 2000);
        }else
        {
            countAlign = 0;
            JustinaNavigation::moveDistAngle(-0.25, 0, 2000);
            break;
        }
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

    //COUNTERS
    int countGraspAttemps = 0;
    int findSeatCount = 0;
    int attempsDoorOpend = 0;
    int attempsGrasp = 0;
    
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
    std::string cutleryLoc = "cupboard";
    std::string tableLoc = "storage_table";

    //FOR GRASP OBJECTS (CUTLERY)
    vision_msgs::VisionObjectList my_cutlery;     
    my_cutlery.ObjectList.resize(6);  
    my_cutlery.ObjectList[0].id="red";
    my_cutlery.ObjectList[1].id="green";
    my_cutlery.ObjectList[2].id="blue";
    my_cutlery.ObjectList[3].id="purple";
    my_cutlery.ObjectList[4].id="yellow";
    my_cutlery.ObjectList[5].id="orange";

    std::vector<vision_msgs::VisionObject> recoObjForTake;
    std::vector<vision_msgs::VisionObject> recoObjList;

    int type;
    int graspObjectID = BOWL;
    std::string id_cutlery;
    std::string graspObject = " bowl "; // To say object, First Justina will take the bowl

// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!
    std::string idObjectGrasp = "cereal"; // !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!// !!!!!!!!1CHANGE FOR CEREAL !!!!!!!!!!!!!!!!!!!!!
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

    JustinaHRI::usePocketSphinx = true;
    STATE state =  SM_ALIGN_WITH_TABLE;//SM_INIT;//SM_SEARCH_BOWL;//SM_PLACE_SPOON;//SM_GO_TO_KITCHEN;//


    while(ros::ok() && !success){

        switch(state){
            case SM_INIT:
                
                std::cout << test << ".-> State SM_INIT: Init the test." << std::endl;
                
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaManip::startTorsoGoTo(0.10, 0, 0);
                JustinaManip::waitForTorsoGoalReached(3000);
                JustinaHRI::waitAfterSay("I am ready for the serve the breakfast test", 6000, MIN_DELAY_AFTER_SAY);
                state = SM_WAIT_FOR_OPEN;
                
                break;

            case SM_WAIT_FOR_OPEN:
                std::cout << test << ".-> State SM_WAIT_FOR_OPEN: Wait for open the door." << std::endl;
                
                JustinaHRI::waitAfterSay("Human, can you open the door please", 6000, MIN_DELAY_AFTER_SAY);
                if( JustinaNavigation::doorIsOpen(0.9, 2000) || attempsDoorOpend >= MAX_ATTEMPTS_DOOR )
                {
                    state = SM_NAVIGATE_TO_TABLEWARE;
                    JustinaHRI::waitAfterSay("Thank you, I will navigate to the cabinet", 4000, MIN_DELAY_AFTER_SAY);
                }
                else
                    attempsDoorOpend++;

                break;
            case SM_NAVIGATE_TO_TABLEWARE:
                std::cout << test << ".-> State SM_NAVIGATE_TO_KITCHEN: Navigate to the kitchen." << std::endl;
                if(!JustinaNavigation::getClose(cutleryLoc, 80000) )
                    JustinaNavigation::getClose(cutleryLoc, 80000); 
                JustinaHRI::waitAfterSay("I have reached ", 4000, MIN_DELAY_AFTER_SAY);

                
                
                state = SM_ALIGN_WITH_TABLE;       
                break;

            case SM_ALIGN_WITH_TABLE:

                if( flagOnce )
                {
                    JustinaHRI::waitAfterSay("Human, Could you open the cabinet door, please", 4000, MIN_DELAY_AFTER_SAY);
                    ros::Duration(5.0).sleep();
                    JustinaHRI::waitAfterSay("Thank you", 4000, MIN_DELAY_AFTER_SAY);
                }

                
                std::cout << ".-> Aligning with table" << std::endl;
                
                alignWithTable();
                if( !flagOnce )
                    JustinaHRI::waitAfterSay("Human, Could you open the drawer, please", 4000, MIN_DELAY_AFTER_SAY);

                JustinaManip::startTorsoGoTo(0.10, 0, 0);
                state = SM_FIND_OBJECTS_ON_TABLE;
            
            break;
            case SM_FIND_OBJECTS_ON_TABLE:

                std::cout << ".-> trying to detect the objects" << std::endl;
                ss.str("");
                ss << "I'm looking for a" << graspObject ;
                JustinaHRI::say(ss.str());
                ros::Duration(2.0).sleep();
                JustinaManip::hdGoTo(0, -.8, 2000);
                if(!JustinaVision::getObjectSeg(my_cutlery))
                {
                        std::cout << ".-> Can not detect any object" << std::endl;
                        state = SM_FIND_OBJECTS_ON_TABLE;
                }
                else
                {
                    std::cout << ".-> sorting the objects" << std::endl;
                        if(!JustinaTasks::sortObjectColor(my_cutlery))
                            if(!JustinaTasks::sortObjectColor(my_cutlery)) 

                        std::cout << ".-> selecting one object" << std::endl;

                        for(int i=0; i < my_cutlery.ObjectList.size(); i ++)
                        {
                            if(my_cutlery.ObjectList[i].graspable == true && my_cutlery.ObjectList[i].type_object == graspObjectID )
                            {
                                std::cout << ".-> detect the " << my_cutlery.ObjectList[i].id << " object" << std::endl;
                                pose.position.x = my_cutlery.ObjectList[i].pose.position.x;
                                pose.position.y = my_cutlery.ObjectList[i].pose.position.y;
                                pose.position.z = my_cutlery.ObjectList[i].pose.position.z;
                                id_cutlery = my_cutlery.ObjectList[i].id;
                                type = my_cutlery.ObjectList[i].type_object;
                                ss.str("");
                                ss << "I've found a" << graspObject;
                                JustinaHRI::say(ss.str());
                                ros::Duration(2.0).sleep();
                                state = SM_TAKE_OBJECT;
                                break;
                            }
                        } 
                    }
                break;
    
            case SM_TAKE_OBJECT:
                
                std::cout << ".-> Trying to take the object" << std::endl;
                    
                if( flagOnce )
                {
                    withLeft = (pose.position.y > 0 ? true : false);
                    state = SM_ALIGN_WITH_TABLE;
                    graspObjectID = CUTLERY;
                    graspObject = " spoon ";
                    flagOnce = false;
                }
                else
                {   
                    state= SM_LOOK_FOR_TABLE;//SM_GO_TO_KITCHEN;
                    withLeft ^= true;
                }

                countGraspAttemps = 0;
                
                while(countGraspAttemps++ <= MAX_ATTEMPTS_GRASP )
                {
                    if(!graspObjectColorCupBoardFeedback2(pose.position.x, pose.position.y, pose.position.z, withLeft, id_cutlery, true))
                        std::cout << ".-> cannot take the object" << std::endl;
                    else
                        break;
                }
                
                //JustinaManip::startTorsoGoTo(0, 0, 0);
                //JustinaManip::waitForTorsoGoalReached(4000);
                break;
        
            case SM_GO_TO_KITCHEN:
                std::cout << test << ".-> State SM_NAVIGATE_TO_KITCHEN: Navigate to the kitchen." << std::endl;
                

                if(!JustinaNavigation::getClose(recogLoc, 80000) )
                    JustinaNavigation::getClose(recogLoc, 80000); 

                JustinaHRI::waitAfterSay("I have reached the kitchen", 4000, MIN_DELAY_AFTER_SAY);
                state = SM_LOOK_FOR_TABLE;       
                break;
            break;

            case SM_LOOK_FOR_TABLE:
                if(!JustinaNavigation::getClose(tableLoc, 80000) )
                    JustinaNavigation::getClose(tableLoc, 80000); 
                JustinaHRI::waitAfterSay("I have reached the table", 4000, MIN_DELAY_AFTER_SAY);
                
                /*
                JustinaHRI::waitAfterSay("I'm looking for the table", 4000, MIN_DELAY_AFTER_SAY);
                centroids.clear();
                findSeat = JustinaTasks::turnAndRecognizeYolo(idsSeatTable, JustinaTasks::NONE, 0.0f, 0.1f, 0.0f, -0.2f, -0.2f, -0.3f, 0.1f, 0.1f, 9.0, centroids, "kitchen");
                if(!findSeat)
                {
                    findSeatCount++;
                    JustinaHRI::waitAfterSay("I'm going to find the table", 5000);
                    break;
                }
                centroid = centroids[0];
                JustinaHRI::waitAfterSay("Please wait", 4000, MIN_DELAY_AFTER_SAY);
                JustinaTools::transformPoint("/base_link", centroid(0, 0), centroid(1, 0) , centroid(2, 0), "/map", gx_w, gy_w, gz_w);
                JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                JustinaKnowledge::addUpdateKnownLoc("guest", gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                goalx = gx_w;
                goaly = gy_w;
                guest_z = gz_w;
                JustinaTasks::closeToGoalWithDistanceTHR(goalx, goaly, 0.3, 30000);
                withLeft = true;
                */
                state = SM_PLACE_BOWL;
                
            break;

            case SM_PLACE_BOWL:
                JustinaHRI::waitAfterSay("I'm going to place the bowl", 4000, MIN_DELAY_AFTER_SAY);
                
                alignWithTable();

                if(!JustinaTasks::placeObject(!withLeft))
                    state = SM_PLACE_BOWL;
                else
                {
                    state = SM_PLACE_SPOON;
                    //withLeft = false;
                }

                JustinaNavigation::moveDistAngle(-0.3, 0, 2000);
                JustinaManip::startTorsoGoTo(0.10, 0, 0);
                JustinaManip::waitForTorsoGoalReached(3000);

                
            break;

            case SM_PLACE_SPOON:
                JustinaHRI::waitAfterSay("I'm going to place the spoon", 4000, MIN_DELAY_AFTER_SAY);
                
                alignWithTable();

                JustinaManip::hdGoTo(0, -.8, 2000);
                if(!JustinaVision::getObjectSeg(my_cutlery))
                {
                        std::cout << ".-> Can not detect any object" << std::endl;
                        state = SM_PLACE_SPOON;
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
                                ros::Duration(2.0).sleep();
                                state = SM_TAKE_OBJECT;
                                break;
                            }
                        } 
                    }



                if(!placeSpoon(pose.position.x, pose.position.y, pose.position.z, withLeft, id_cutlery, true) )
                    state = SM_PLACE_SPOON;
                else
                {
                    state = SM_GO_FOR_CEREAL;
                    withLeft = false;
                }

                JustinaNavigation::moveDistAngle(-0.3, 0, 2000);
                

            break;

            case SM_GO_FOR_CEREAL:

                JustinaHRI::waitAfterSay("I'm going to the cupboard", 4000, MIN_DELAY_AFTER_SAY);
                
                if(!JustinaNavigation::getClose(cutleryLoc, 80000) )
                    JustinaNavigation::getClose(cutleryLoc, 80000); 

                JustinaHRI::waitAfterSay("I have reached cupboard", 4000, MIN_DELAY_AFTER_SAY);
                
                state = SM_TAKE_CEREAL;
            break;

            case SM_TAKE_CEREAL:

                //alignWithTable();
                //JustinaHRI::waitAfterSay("I'm going to take the cereal", 4000, MIN_DELAY_AFTER_SAY);
                
                //if(JustinaTasks::findObject(idObjectGrasp, poseCereal, withLeft) )
                //{
                //    state = SM_RETURN_TO_TABLE;
                 //   JustinaTasks::graspObject(poseCereal.position.x, poseCereal.position.y, poseCereal.position.z, withLeft, idObjectGrasp, true, false);
                //}else
                //{
                    state = SM_TAKE_CEREAL;
                //}

                if (!JustinaManip::isRaInPredefPos("navigation"))
                    JustinaManip::startRaGoTo("navigation");
                else
                    std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;

                JustinaManip::startRaOpenGripper(0.3);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

                JustinaHRI::waitAfterSay("Human please put the cereals in my gripper", 4000, MIN_DELAY_AFTER_SAY);
                boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
                JustinaHRI::waitAfterSay("Thank you.", 4000, MIN_DELAY_AFTER_SAY);
                JustinaManip::startRaCloseGripper(0.5);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

                
                JustinaManip::startTorsoGoTo(0.10, 0, 0);
                JustinaManip::waitForTorsoGoalReached(3000);

            break;
            case SM_RETURN_TO_TABLE:


                JustinaHRI::waitAfterSay("I'm going to the table", 4000, MIN_DELAY_AFTER_SAY);
                
                if(!JustinaNavigation::getClose(tableLoc, 80000) )
                    JustinaNavigation::getClose(tableLoc, 80000); 

                //alignWithTable();

                //JustinaHRI::waitAfterSay("I'm going to pouring  the  cereals inside the bowl", 4000, MIN_DELAY_AFTER_SAY);
                
                state = SM_SEARCH_BOWL;
            break;

            case SM_SEARCH_BOWL:

                std::cout << ".-> trying to detect the objects" << std::endl;

                graspObjectID = BOWL;
                
                alignWithTable();

                JustinaManip::hdGoTo(0, -.8, 2000);
                if(!JustinaVision::getObjectSeg(my_cutlery))
                {
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
                            if(my_cutlery.ObjectList[i].graspable == true && my_cutlery.ObjectList[i].type_object == graspObjectID )
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

                //withLeft = (pose.position.y > 0 ? true : false);
                 
                countGraspAttemps = 0;
                
                JustinaHRI::waitAfterSay("I am going to pour the cereal carefully inside the bowl.", 6000, MIN_DELAY_AFTER_SAY);
                while(countGraspAttemps++ <= MAX_ATTEMPTS_GRASP )
                {
                    if(!pouringCereal(pose.position.x, pose.position.y, pose.position.z, withLeft, id_cutlery, true))
                        std::cout << ".-> cannot take the object" << std::endl;
                    else
                        break;
                }
                state = SM_LEAVE_CEREAL;
            break;

            case SM_LEAVE_CEREAL:
                
                alignWithTable();
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

    std::stringstream ss;
    ss.str("");

    bool objectInHand = false;
    float idealX = 0.50;
    float idealY = withLeftArm ? 0.225 : -0.255; //It is the distance from the center of the robot, to the center of the arm
    float idealZ = 0.6; //It is the ideal height for taking an object when torso is at zero height.
    float missingZ = 0.0;


    float minTorso = 0.08;
    float maxTorso = 0.294;

    float lastTorso ;
    float currentTorso ;

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

    if (goalTorso < minTorso){
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
    //JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
    JustinaNavigation::getRobotPoseFromOdom(lastRobotX, lastRobotY, lastRobotTheta);
    
    //if (usingTorse)
    //    JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
    JustinaNavigation::moveLateral(movLateral, 3000);
    JustinaNavigation::moveDist(movFrontal, 3000);

    //if (usingTorse)
    //    JustinaManip::waitForTorsoGoalReached(waitTime);

    //JustinaManip::startTorsoGoTo(torsoSpine+.02, 0, 0);
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

    

    if (found && objects.ObjectList[0].graspable) {
        std::cout << "The object was found again, update the new coordinates." << std::endl;
        typeCutlery = objects.ObjectList.at(0).type_object;
        
        JustinaHardware::getTorsoCurrentPose(lastTorso, torsoWaist, torsoShoulders);
        JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
        JustinaManip::waitForTorsoGoalReached(waitTime);
        JustinaHardware::getTorsoCurrentPose(currentTorso, torsoWaist, torsoShoulders);

        switch (typeCutlery) {
            //Cutlery objects
            case 0:
                objToGraspX = objects.ObjectList.at(0).pose.position.x;
                objToGraspY = objects.ObjectList.at(0).pose.position.y;
                objToGraspZ = objects.ObjectList.at(0).minPoint.z  + .3 ;//+ currentTorso - lastTorso ;
                dz = minTorso;
                break;
            // This to the bowls
            case 1:
                objToGraspX = objects.ObjectList.at(0).pose.position.x;
                if (withLeftArm)
                    objToGraspY = objects.ObjectList.at(0).pose.position.y;//maxPoint.y;
                else
                    objToGraspY = objects.ObjectList.at(0).pose.position.y;//minPoint.y;

                objToGraspZ = objects.ObjectList.at(0).minPoint.z + 0.1;// + currentTorso -lastTorso;
                dz = minTorso;
                break;
            default:
                break;
        }
        std::cout << "MaxPoint en z:" << objToGraspZ << std::endl;
    } else if (!found && colorObject.compare("") == 0) {
        std::cout << "The object was not found again, update new coordinates with the motion of robot." << std::endl;
        float robotX, robotY, robotTheta;
        //JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
        JustinaNavigation::getRobotPoseFromOdom(robotX, robotY, robotTheta);
        //Adjust the object position according to the new robot pose
        //I don't request again the object position due to the possibility of not recognizing it again
        float dxa = (robotX - lastRobotX);
        float dya = (robotY - lastRobotY);
        float dxr = dxa * cos(robotTheta) + dya * sin(robotTheta);
        float dyr = -dxa * sin(robotTheta) + dya * cos(robotTheta);

        objToGraspX -= dxr;
        objToGraspY -= dyr;
        std::cout << "lastRobotX:" << lastRobotX << ",lastRobotY:" << lastRobotY << ",lastRobotTheta:" << lastRobotTheta << std::endl;
        std::cout << "robotX:" << robotX << ",robotY:" << robotY << ",robotTheta:" << robotTheta << std::endl;
        std::cout << "objToGraspX:" << objToGraspX << ",objToGraspY:" << objToGraspY << ",objToGraspZ:" << objToGraspZ << std::endl;
    } else if (!found && colorObject.compare("") != 0 || !objects.ObjectList[0].graspable) {
        JustinaNavigation::moveDist(-0.2, 2000);
        return false;
    }

    //The position it is adjusted and converted to coords wrt to the corresponding arm
    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";
    if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY, objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) {
        std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
        return false;
    }

    std::cout << "JustinaTasks.->Moving ";
    if (withLeftArm)
        std::cout << "left arm";
    else
        std::cout << "right arm";
    std::cout << " to " << objToGraspX << "  " << objToGraspY << "  " << objToGraspZ << std::endl;

    
    if( withLeftArm ) {
        if ( typeCutlery == 0) 
        {
            JustinaManip::laGoToCartesianTraj(objToGraspX + 0.03, objToGraspY - 0.02, objToGraspZ, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

            JustinaManip::startLaOpenGripper(0.3);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
            
            JustinaManip::laGoToCartesian(objToGraspX + 0.00, objToGraspY + 0.0, objToGraspZ,objects.ObjectList[0].roll, objects.ObjectList[0].pitch,objects.ObjectList[0].yaw, 0.1, 5000);
            
            JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
            if ( usingTorse )
                JustinaManip::startTorsoGoTo(torsoSpine-.13, 0, 0);
                JustinaManip::waitForTorsoGoalReached(waitTime);

            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

            JustinaManip::startLaCloseGripper(0.5);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

            JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
            if ( usingTorse )
                JustinaManip::startTorsoGoTo(torsoSpine+.1, 0, 0);
            JustinaNavigation::moveDist(-.3, 2000);
            if ( usingTorse )
            JustinaManip::waitForTorsoGoalReached(waitTime);

        }
        else if (typeCutlery == 1 ) 
        {
            std::vector<float> articular;            
            
            if(JustinaManip::inverseKinematics(objToGraspX - 0.12, objToGraspY - 0.25, objToGraspZ, articular))
                JustinaManip::startLaGoToArticular(articular);
            else
                std::cout << "JustinaTask.->Can not calculate inverse kinematics." << std::endl;
            
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
            if(JustinaManip::inverseKinematics(objToGraspX - 0.08, objToGraspY - 0.0, objToGraspZ, articular)){
                JustinaManip::waitForLaGoalReached(3500);
                JustinaManip::startLaGoToArticular(articular);
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            }
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        ros::spinOnce();

        JustinaManip::startLaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        
        //for (int i = 0; i < 3; i++) 
        //{
            //if (usingTorse) 
            //{
            //    float torsoAdjust = 0.08;    
            //    JustinaManip::startTorsoGoTo(goalTorso + torsoAdjust, 0, 0);
            //    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            //    JustinaManip::waitForTorsoGoalReached(20000);
               
            //} else
            //    JustinaManip::laGoToCartesian(objToGraspX - 0.13, objToGraspY + 0.04, objToGraspZ, 0, 0, 1.5708, 0, 5000);
            
            //if (i == 0) {
            JustinaManip::laGoTo("navigation", 3500);
            //}

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
                JustinaManip::startTorsoGoTo(torsoSpine-.13, 0, 0);
            JustinaManip::waitForTorsoGoalReached(waitTime);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

            JustinaManip::startRaCloseGripper(0.5);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));


            JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
            if ( usingTorse )
                JustinaManip::startTorsoGoTo(torsoSpine+.1, 0, 0);
            JustinaNavigation::moveDist(-.3, 3000);
            if ( usingTorse )
            JustinaManip::waitForTorsoGoalReached(waitTime);

        
        }
        else if (typeCutlery == 1 ) 
        {
            std::vector<float> articular;            
            
            if(JustinaManip::inverseKinematics(objToGraspX - 0.12, objToGraspY - 0.25, objToGraspZ, articular))
                JustinaManip::startRaGoToArticular(articular);
            else
                std::cout << "JustinaTask.->Can not calculate inverse kinematics." << std::endl;
            
            boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            JustinaManip::startRaOpenGripper(0.5);

            articular.clear();
            if(JustinaManip::inverseKinematics(objToGraspX - 0.06, objToGraspY - 0.15, objToGraspZ, articular)){
                JustinaManip::waitForRaGoalReached(2500);
                JustinaManip::startRaGoToArticular(articular);
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            }

            articular.clear();
            if(JustinaManip::inverseKinematics(objToGraspX - 0.06, objToGraspY , objToGraspZ, articular)){
                JustinaManip::waitForRaGoalReached(2500);
                JustinaManip::startRaGoToArticular(articular);
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            }

            articular.clear();
            if(JustinaManip::inverseKinematics(objToGraspX - 0.08, objToGraspY - 0.0, objToGraspZ, articular)){
                JustinaManip::waitForRaGoalReached(3500);
                JustinaManip::startRaGoToArticular(articular);
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            }

            JustinaManip::waitForRaGoalReached(2500);

        }
        
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        ros::spinOnce();
        
        JustinaManip::startRaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        
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
    
        JustinaManip::raGoToCartesian(objToGraspX-.04, objToGraspY, objToGraspZ, 0.0, 0.0, 0, 0.0, 5000);
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
        found = JustinaVision::getObjectSeg(objects);
        std::cout << "GET OBJECTS: " << found << std::endl;
    }

    if (found && objects.ObjectList[0].graspable) 
    {
        std::cout << "The object was found again, update the new coordinates." << std::endl;
        typeCutlery = objects.ObjectList.at(0).type_object;
        
        objToGraspX = objects.ObjectList.at(0).pose.position.x;
        if (withLeftArm)
            objToGraspY = objects.ObjectList.at(0).maxPoint.y-.1;
        else
            objToGraspY = objects.ObjectList.at(0).minPoint.y+.1;

        objToGraspZ = objects.ObjectList.at(0).minPoint.z + 0.2;
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
        
        JustinaManip::laGoToCartesian(objToGraspX -.15, objToGraspY , objToGraspZ ,0.0, 0.0, 1.5707, 0.1, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(400));
        std::vector<float> currPose;
        JustinaManip::getLaCurrentPos(currPose);
        if (currPose.size() == 7) 
        {
            while( (currPose[6] += 0.6 ) < 2.5)
            {
                JustinaManip::laGoToArticular(currPose, 2000);
                //JustinaManip::waitForLaGoalReached(900);   
            }
            while( (currPose[6] -= 0.6) > 0)
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


        JustinaManip::raGoToCartesian(objToGraspX -.15, objToGraspY , objToGraspZ+.08 ,0.0, 0.0, 1.5707, 0.1, 5000);
        std::vector<float> currPose;
        JustinaManip::getRaCurrentPos(currPose);
        if (currPose.size() == 7) 
        {
            while( (currPose[6] -= 0.6) > -2.5)
            {
                JustinaManip::raGoToArticular(currPose,2000);   
            }
            
            while( (currPose[6] += 0.6) < 0)
            {
                JustinaManip::raGoToArticular(currPose,2000);   
            }
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