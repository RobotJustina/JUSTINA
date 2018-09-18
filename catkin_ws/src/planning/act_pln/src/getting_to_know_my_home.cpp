#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaRepresentation.h"

enum STATE{
    SM_INIT,
    SM_SAY_WAIT_FOR_DOOR,
    SM_WAIT_FOR_DOOR,
    SM_ENTRANCE,
    SM_NAVIGATE_ENTRANCE_DOOR,
    SM_EXPLORING_DOOR,
    SM_GET_DOOR_LOCATION,
    SM_GET_CLOSE_LOCATION,
    SM_FIND_OBJECTS,
    SM_CREATE_SEMANTIC_MAP,
    SM_FINISH_TEST
};

STATE state;

std::string task("getting to know my home");

std::stringstream ss;

int minDelayAfterSay = 0;
int maxDelayAfterSay = 300;

bool startSignalSM = true;
bool fail = false, success = false;

std::string furnituresLocations [5] = {"entrance_door", "desk", "center_table", "coffee_table", "dining_table"};
std::string roomToVisitDummy [5][3] = {{"hallway", "", ""}, {"hallway", "living_room", "bedroom"}, {"bedroom", "living_room", ""}, {"living_room", "", ""}, {"living_room", "dining_room", ""}};
int sizeRoomToVisitDummy [5] = {1, 3, 2, 1, 2};
std::vector<std::string> locations;
std::string robotLocation = "";
float currX, currY, currTheta;
int locationMaxAttemps = 4;
int locationsAttemps = 0;
int currFurnitureLocation = 0;
int currLocation = 0;

bool door_isopen=false;
int countDoorIsOpen = 0;
int maxCountDoorIsOpen = 0;
int attempsCountDoorIsOpen = 20;
int maxAttempsCountDoorIsOpen = 12;
            
std::vector<std::string> tokens_items;
std::vector<std::string>::iterator tokens_items_it;
std::size_t found;

int closed_doors = 0;
int max_closed_doors = 1;

int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
float laser_l=0;

sensor_msgs::Image image;
std::vector<vision_msgs::VisionObject> recoObjList;
bool alignWithTable = true;
// This is for attemps to find objects on the table
int attempsFindObjects = 0;
// This is for the max attemps to find Object table
int maxAttempsFindObjects = 2;

bool funCompNearestVisionObject(vision_msgs::VisionObject obj1, vision_msgs::VisionObject obj2){
    return (obj1.confidence < obj2.confidence);
}

void Callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    range=msg->ranges.size();
    range_c=range/2;
    range_i=range_c-(range/10);
    range_f=range_c+(range/10);
    //std::cout<<"Range Size: "<< range << "\n ";
    //std::cout<<"Range Central: "<< range_c << "\n ";
    //std::cout<<"Range Initial: "<< range_i << "\n ";
    //std::cout<<"Range Final: "<< range_f << "\n ";

    cont_laser=0;
    laser_l=0;
    for(int i=range_c-(range/10); i < range_c+(range/10); i++)
    {
        if(msg->ranges[i] > 0 && msg->ranges[i] < 4){ 
            laser_l=laser_l+msg->ranges[i];    
            cont_laser++;
        }
    }
    std::cout<<"Laser promedio: "<< laser_l/cont_laser << std::endl;    
    if(laser_l/cont_laser > 2.0){
        door_isopen=true;
    }
    else{
        door_isopen=false;
    }
}

int main(int argc, char ** argv)
{
    
    std::cout << "INITIALIZING VISIT MY HOME TEST ..." << std::endl;
    ros::init(argc, argv, "visit_my_home_test");
    ros::NodeHandle n;
    ros::Rate rate(10);
        
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
    JustinaRepresentation::setNodeHandle(&n);
    
    ros::Subscriber laser_subscriber = n.subscribe<sensor_msgs::LaserScan>("/hardware/scan", 1, Callback_laser);
    
    JustinaRepresentation::initKDB("/virbot_iros/speechTest.dat", true, 20000);

    while(ros::ok() && !fail && !success)
    {
        switch(state)
        {
            case SM_INIT:
                std::cout << task << " state machine: SM_INIT" << std::endl;
                if (startSignalSM) {
                    JustinaHRI::waitAfterSay("I am ready for the getting to know my home test", 5000, minDelayAfterSay);
                    JustinaHRI::loadGrammarSpeechRecognized("visit_my_home.xml");//load the grammar
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    // JustinaTools::pdfAppend("HelpMeCarry_Plans", "Starting the Help me Carry Test");
                    state = SM_SAY_WAIT_FOR_DOOR;
                }
                break;
            case SM_SAY_WAIT_FOR_DOOR:
                JustinaHRI::waitAfterSay("I am waiting for the door to be open", 4000, minDelayAfterSay);
                state = SM_WAIT_FOR_DOOR;
                break;
            case SM_WAIT_FOR_DOOR:
                std::cout << task << " state machine: SM_WAIT_OPEN_DOOR" << std::endl;
                if (!JustinaNavigation::obstacleInFront()){
                    JustinaHRI::waitAfterSay("Now I can see that the door is open", 4000);
                    JustinaNavigation::moveDist(1.0, 4000);
                    // state = SM_ENTRANCE;
                    currFurnitureLocation = 0;
                    state = SM_ENTRANCE;
                }
                break;
            case SM_ENTRANCE:
                std::cout << task << " state machine: SM_ENTRANCE" << std::endl;
                if(!JustinaNavigation::getClose("arena", 120000))
                {
                    JustinaNavigation::getClose("arena", 120000);
                }
                currFurnitureLocation = 0;
                state = SM_GET_DOOR_LOCATION;
                break;
            case SM_EXPLORING_DOOR:
                std::cout << task << " state machine: SM_EXPLORING_DOOR" << std::endl;
                attempsCountDoorIsOpen++;
                if(attempsCountDoorIsOpen > maxAttempsCountDoorIsOpen)
                {
                    if(countDoorIsOpen > maxCountDoorIsOpen)
                    {
                        JustinaHRI::waitAfterSay("The door is open", 4000);
                        door_isopen = true;
                        state = SM_GET_CLOSE_LOCATION;
                    }
                    else
                    {
                        std::string location = furnituresLocations[currFurnitureLocation];
                        boost::replace_all(location, "door_", "");
                        boost::algorithm::split(tokens_items, location, boost::algorithm::is_any_of("_"));
                        if(tokens_items.size() == 2)
                            JustinaRepresentation::updateStateDoor(tokens_items[0], tokens_items[1], false, 0);
                        door_isopen = false;
                        closed_doors++;
                        JustinaHRI::waitAfterSay("The door is close", 4000);
                        currFurnitureLocation++;
                        state = SM_GET_DOOR_LOCATION;
                    }
                    attempsCountDoorIsOpen = 0;
                    countDoorIsOpen = 0;
                }
                else if(door_isopen)
                    countDoorIsOpen++;
                break;
            case SM_GET_DOOR_LOCATION:
                std::cout << task << " state machine: SM_GET_DOOR_LOCATION" << std::endl;
                if(currFurnitureLocation >= (sizeof(furnituresLocations)/sizeof(*furnituresLocations))){
                    state = SM_CREATE_SEMANTIC_MAP;
                    break;
                }
                //JustinaNavigation::getRobotPoseRoom(robotLocation);
                JustinaNavigation::getRobotPose(currX, currY, currTheta);
                locations.clear();
                if(closed_doors < max_closed_doors){
                    // TODO CORRECT IN THE COMPETITION
                    //locations = JustinaKnowledge::getRoomsFromPath(currX, currY, furnituresLocations[currFurnitureLocation]);
                    locations = std::vector<std::string>(roomToVisitDummy[currFurnitureLocation], roomToVisitDummy[currFurnitureLocation] + sizeRoomToVisitDummy[currFurnitureLocation]);
                    if(locations.size() > 1){
                        JustinaRepresentation::getDoorsPath(locations, locations, 1000);
                        locations.push_back(furnituresLocations[currFurnitureLocation]);
                    }
                    else{
                        locations.clear();
                        locations.push_back(furnituresLocations[currFurnitureLocation]);
                    }
                }
                else{
                    locations.clear();
                    locations.push_back(furnituresLocations[currFurnitureLocation]);
                }
                currLocation = 0;
                locationsAttemps = 1;
                state = SM_GET_CLOSE_LOCATION;
                break;
            case SM_GET_CLOSE_LOCATION:
                std::cout << task << " state machine: SM_GET_CLOSE_LOCATION" << std::endl;
                if(currLocation >= locations.size()){
                    state = SM_FIND_OBJECTS;
                    break;
                }
                if(!(locationsAttemps > locationMaxAttemps)){
                    std::string location = locations[currLocation];
                    found = locations[currLocation].find("door");
                    ss.str("");
                    if(found != std::string::npos)
                        ss << "I will navigate to the door";
                    else
                        ss << "I will navigate to the " << locations[currLocation];
                    //std::cout << task << " state machine: SM_GET_CLOSE_LOCATION.->" << ss.str() << std::endl;
                    JustinaHRI::waitAfterSay(ss.str(), 3000, minDelayAfterSay);
                    if(!JustinaNavigation::getClose(locations[currLocation], 240000)){
                        if(!JustinaNavigation::getStopWaitGlobalGoalReached()){
                            locationsAttemps++;
                            break;
                        }
                    }
                    ss.str("");
                    ss << "I have reached the " << locations[currLocation];
                }
                else{
                    ss.str("");
                    ss << "I can not reached the " << locations[currLocation];
                }
                JustinaHRI::waitAfterSay(ss.str(), 3000, minDelayAfterSay);
                currLocation++;
                locationsAttemps = 1;
                if(currFurnitureLocation == 0 || (locations.size() > 1 && currLocation < locations.size())){
                    //currFurnitureLocation++;
                    attempsCountDoorIsOpen = 0;
                    countDoorIsOpen = 0;
                    state = SM_EXPLORING_DOOR;
                }
                break;
            case SM_FIND_OBJECTS:
                std::cout << task << " state machine: SM_FIND_OBJECTS" << std::endl;
                attempsFindObjects++;
                if(attempsFindObjects <= maxAttempsFindObjects){
                    if(attempsFindObjects == 1 && alignWithTable){
                        //JustinaTools::pdfAppend(name_test, fnd_objs_tbl);
                        if(!JustinaTasks::alignWithTable(0.35)){
                            JustinaNavigation::moveDist(0.10, 3000);
                            if(!JustinaTasks::alignWithTable(0.35)){
                                std::cout << "I can´t alignWithTable... :'(" << std::endl;
                                JustinaNavigation::moveDist(-0.15, 3000);
                                alignWithTable = false;
                                break;
                            }
                        }
                    }
                    recoObjList.clear();
                    //categories_tabl.clear();
                    if(!JustinaVision::detectAllObjectsVot(recoObjList, image, 5)){
                        std::cout << "I  can't detect anything" << std::endl;
                        state = SM_FIND_OBJECTS;
                    }else{
                        for(int i = 0; i < recoObjList.size(); i++){
                            std::size_t found = recoObjList[i].id.find("unkown");
                            if(found == std::string::npos)
                                recoObjList.erase(recoObjList.begin() + i);
                        }
                        if(recoObjList.size() > 0){
                            std::string name;
                            int id;
                            std::string location;
                            bool isObjectInDefaultLocation = false;
                            if(recoObjList.size() > 1){
                                std::sort(recoObjList.begin(), recoObjList.end(),  funCompNearestVisionObject);
                                recoObjList.erase(recoObjList.begin() + 1, recoObjList.end());
                            }
                            name = recoObjList[0].id;
                            id = 0;
                            location = furnituresLocations[currFurnitureLocation++]; 
                            JustinaRepresentation::isObjectInDefaultLocation(name, id, location, isObjectInDefaultLocation, 0);
                            if(!isObjectInDefaultLocation)
                                JustinaRepresentation::updateFurnitureFromObject(name, id, "", location, 0);
                            //temp.str("");
                            //temp << "/home/biorobotica/objs/table" << countFindObjectsOnTable++ << "/"; 
                            //JustinaTools::saveImageVisionObject(recoObjList, image, temp.str());
                            state = SM_GET_DOOR_LOCATION;
                        }
                        /*JustinaTools::pdfAppend(name_test, justinaSay.str());
                          JustinaTools::getCategoriesFromVisionObject(recoObjList, categories_tabl);
                          JustinaTools::pdfAppend(name_test, " - Categories found on the table: ");
                          for(int i = 0; i < categories_tabl.size(); i++){
                          std::cout << "Category_" << i << ":  " << categories_tabl[i] << std::endl;
                          temp.str( std::string() );
                          temp << "      - " << categories_tabl[i];
                          JustinaTools::pdfAppend(name_test, temp.str());
                          }*/
                        //JustinaTools::pdfImageStopRec(name_test,"/home/$USER/objs/");
                    }
                }
                else{
                    currFurnitureLocation++;
                    state = SM_GET_DOOR_LOCATION;
                }
                break;
            case SM_CREATE_SEMANTIC_MAP:
                std::cout << task << " state machine: SM_CREATE_SEMANTIC_MAP" << std::endl;
                JustinaRepresentation::getSemanticMap(0); 
                state = SM_FINISH_TEST;
                break;
            case SM_FINISH_TEST:
                std::cout << task << " state machine: SM_FINISH_TEST" << std::endl;
                JustinaHRI::waitAfterSay("I have finished the test", 2500);
                //JustinaTools::pdfStop("HelpMeCarry_Plans");
                success = true;
                break;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 1;
}
