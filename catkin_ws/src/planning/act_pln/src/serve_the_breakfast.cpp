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
    SM_GO_FOR_CEREAL
};

std::string lastRecoSpeech;
std::string lastInteSpeech;

std::string test("serve the breakfast");

int main(int argc, char **argv){

    ros::init(argc, argv, "serve_the_breakfast_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    bool findSeat = false;
    bool doorOpenFlag = false;
    bool success = false;
    bool withLeft = false;
    int attempsDoorOpend = 0;
    int findSeatCount = 0;
    geometry_msgs::Pose pose;
    std::string id_cutlery;
    int type;

    std::string param, typeOrder;
    std::string lastName, lastDrink;
    std::vector<std::string> names;
    std::vector<std::string> drinks;
    std::string grammarCommandsID = "serve_the_breakfastCommands";
    std::string grammarDrinksID = "serve_the_breakfastDrinks";
    std::string grammarNamesID = "receptionistNames";
    
    std::string recogLoc = "kitchen";
    std::string cutleryLoc = "table_location";
    std::string graspObject = " bowl ";

    int graspObjectID = BOWL;

    Eigen::Vector3d centroid;
    std::vector<Eigen::Vector3d> centroids;
    
    std::stringstream ss;
    
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

    std::vector<std::string> idsSeat;
    idsSeat.push_back("chair");
    
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
    STATE state = SM_INIT;



    vision_msgs::VisionObjectList my_cutlery;     
    my_cutlery.ObjectList.resize(6);  

    my_cutlery.ObjectList[0].id="red";
    my_cutlery.ObjectList[1].id="green";
    my_cutlery.ObjectList[2].id="blue";
    my_cutlery.ObjectList[3].id="purple";
    my_cutlery.ObjectList[4].id="yellow";
    my_cutlery.ObjectList[5].id="orange";

    while(ros::ok() && !success){

        switch(state){
            case SM_INIT:
                
                std::cout << test << ".-> State SM_INIT: Init the test." << std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaHRI::waitAfterSay("I am ready for the serve the breakfast test", 6000, MIN_DELAY_AFTER_SAY);
                state = SM_WAIT_FOR_OPEN;
                
                break;

            case SM_WAIT_FOR_OPEN:
                std::cout << test << ".-> State SM_WAIT_FOR_OPEN: Wait for open the door." << std::endl;
                
                JustinaHRI::waitAfterSay("Human, can you open the door please", 6000, MIN_DELAY_AFTER_SAY);
                if( JustinaNavigation::doorIsOpen(0.9, 2000) || attempsDoorOpend >= MAX_ATTEMPTS_DOOR )
                {
                    state = SM_NAVIGATE_TO_TABLEWARE;
                    JustinaHRI::waitAfterSay("Thank you, I will navigate to the kitchen table", 4000, MIN_DELAY_AFTER_SAY);
                }
                else
                    attempsDoorOpend++;

                break;
            case SM_NAVIGATE_TO_TABLEWARE:
                std::cout << test << ".-> State SM_NAVIGATE_TO_KITCHEN: Navigate to the kitchen." << std::endl;
                if(!JustinaNavigation::getClose(cutleryLoc, 80000) )
                    JustinaNavigation::getClose(cutleryLoc, 80000); 
                JustinaHRI::waitAfterSay("I have reached the kitchen", 4000, MIN_DELAY_AFTER_SAY);
                state = SM_FIND_OBJECTS_ON_TABLE;       
                break;

            case SM_FIND_OBJECTS_ON_TABLE:

                std::cout << ".-> inspecting the objets on the table" << std::endl;
                
                while(!JustinaTasks::alignWithTable(0.42))
                {
                    std::cout << ".-> Can not align with table." << std::endl;
                }
                
                std::cout << ".-> trying to detect the objects" << std::endl;

                ss.str("");
                ss << "I'm looking for a" << graspObject << "on the table";
                JustinaHRI::say(ss.str());

                ros::Duration(2.0).sleep();

                if(!JustinaVision::getObjectSeg(my_cutlery))
                {
                    if(!JustinaVision::getObjectSeg(my_cutlery))
                    {
                        std::cout << ".-> Can not detect any object" << std::endl;
                        state = SM_InspectTheObjetcs;
                    }
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
                                ss << "I've found a" << graspObject << "on the table";
                                JustinaHRI::say(ss.str());
                                ros::Duration(2.0).sleep();
                                state = SM_TAKE_OBJECT;
                                break;
                            }
                        } 
                    }
                break;
            case SM_TAKE_OBJECT:
                if(!JustinaTasks::graspObjectColorFeedback(pose.position.x, pose.position.y, pose.position.z, withLeft, id_cutlery, true)){
                    std::cout << ".-> cannot take the object" << std::endl;
                    std::cout << ".-> trying again" << std::endl;
                }
                if( !withLeft )
                {
                    state = SM_FIND_OBJECTS_ON_TABLE;
                    graspObjectID = CUTLERY;
                    graspObject = " spoon ";
                    withLeft = true;

                }
                else
                    state= SM_GO_TO_KITCHEN;
                
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
                JustinaHRI::waitAfterSay("I'm looking for the table", 4000, MIN_DELAY_AFTER_SAY);
                centroids.clear();
                findSeat = JustinaTasks::turnAndRecognizeYolo(idsSeat, JustinaTasks::NONE, 0.0f, 0.1f, 0.0f, -0.2f, -0.2f, -0.3f, 0.1f, 0.1f, 9.0, centroids, "kitchen");
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
                state = SM_PLACE_BOWL;
                    
            break;

            case SM_PLACE_BOWL:
                JustinaHRI::waitAfterSay("I'm going to place the bowl", 4000, MIN_DELAY_AFTER_SAY);
                
                while(!JustinaTasks::alignWithTable(0.42))
                {
                    std::cout << ".-> Can not align with table." << std::endl;
                }

                if(!JustinaTasks::placeObject(withLeft))
                    state = SM_PLACE_BOWL;
                else
                {
                    state = SM_PLACE_SPOON;
                    withLeft = false;
                }
                
            break;

            case SM_PLACE_SPOON:
                JustinaHRI::waitAfterSay("I'm going to place the spoon", 4000, MIN_DELAY_AFTER_SAY);
                
                while(!JustinaTasks::alignWithTable(0.42))
                {
                    std::cout << ".-> Can not align with table." << std::endl;
                }

                if(!JustinaTasks::placeObject(withLeft))
                    state = SM_PLACE_SPOON;
                else
                {
                    state = SM_GO_FOR_CEREAL;
                    withLeft = false;
                }

            break;

            case SM_GO_FOR_CEREAL:

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