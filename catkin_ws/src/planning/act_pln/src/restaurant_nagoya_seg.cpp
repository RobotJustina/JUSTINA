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
#define SM_FIRST_ORDER 40
#define SM_FIRST_ORDER_CONFIRM 50
#define SM_RETURN_BAR 60

#define SM_FOLLOWING_PHASE 300
#define SM_FOLLOWING_PAUSE 400
#define SM_FOLLOWING_TABLE_1 500
#define SM_FOLLOWING_TABLE_2 600
#define SM_FOLLOWING_TABLE_3 70
#define SM_FOLLOWING_RETURN_KITCHEN 80
#define SM_FOLLOWING_RETURN_PAUSE 90
#define SM_ORDERING_PHASE 100
#define SM_FIRST_ORDER_WHICH_TABLE 110
#define SM_FIRST_ORDER_TABLE_A 120
#define SM_FIRST_ORDER_TABLE_B 130
#define SM_DELIVERING_PHASE 140
#define SM_DELIVERING_TAKING_ORDER 150
#define SM_DELIVERING_BEVERAGE 160
#define SM_DELIVERING_RETURN_KITCHEN 170
#define SM_FIRST_ORDER_RETURN_KITCHEN 180
#define SM_DELIVERING_PUT_ORDER 190
#define SM_FINAL_STATE 200
#define SM_WAIT_FOR_LEGS_FOUND 210
#define SM_WAIT_FOR_LEGS 220

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
   
    ros::Rate loop(10);
    
    std::string reco_sentence;
    std::vector<std::string> validItems;

    validItems.push_back("soda");
    validItems.push_back("milk");
    validItems.push_back("tea");
    validItems.push_back("beer");
    validItems.push_back("wine");
    validItems.push_back("chips");
    validItems.push_back("egg");
    validItems.push_back("eggs");
    validItems.push_back("candy");
    validItems.push_back("candies");
    validItems.push_back("paprika");
    validItems.push_back("apple");
    validItems.push_back("pumper");

    int nextState = 0;
    bool fail = false;
    bool success = false;
    bool stop=false;
    int contador_order=0;
    vision_msgs::VisionRect rectWav;
    bool find;

    int indexToClose = 0;
    std::map<int, std::vector<float> > mapToClose;
    std::map<int, std::vector<float> >::iterator it;
    std::vector<float> vectorPos;

    std::string lastRecoSpeech;

    float robot_y,robot_x,robot_a;    

    std::vector<std::string> startCommands;
    startCommands.push_back("justina start");

    std::vector<std::string> confirmCommands;
    confirmCommands.push_back("justina yes");
    confirmCommands.push_back("justina no");

    std::stringstream ss;
    vision_msgs::VisionFaceObjects faces;

    bool userConfirmation;

    std::string bar_search="";

    while(ros::ok() && !fail && !success){
        switch(nextState){
            case SM_INIT:
                std::cout << "State machine: SM_INIT" << std::endl;	
                JustinaHRI::waitAfterSay("I'm ready for the restaurant test", 10000);
                nextState = SM_WAIT_FOR_INIT_COMMAND;
                break;

            case SM_WAIT_FOR_INIT_COMMAND:
                std::cout << "State machine: SM_WAIT_FOR_INIT_COMMAND" << std::endl;
                if(JustinaHRI::waitForSpecificSentence(startCommands, lastRecoSpeech, 10000)){
                    if(lastRecoSpeech.find("justina start") != std::string::npos){
                        JustinaHRI::waitAfterSay("I will search the bar", 3500);
                        nextState = SM_SEARCH_BAR;
                    }
                    else
                        nextState = SM_WAIT_FOR_INIT_COMMAND;
                }
                break;

            case SM_SEARCH_BAR:
                std::cout << "State machine: SM_SERACH_BAR" << std::endl;
                JustinaTasks::findTable(bar_search);  
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                if (bar_search.compare("center") == 0){
                    JustinaHRI::waitAfterSay("I see the bar in front of me", 10000);
                    //JustinaKnowledge::addUpdateKnownLoc("car_location");	
                }
                else if (bar_search.compare("right") == 0){
                    JustinaHRI::waitAfterSay("I see the bar in my right side", 10000);
                }else if (bar_search.compare("left") == 0){
                    JustinaHRI::waitAfterSay("I see the bar in my left side", 10000);
                }else{
                    std::cout << "SM_SERACH_BAR: Bar default" << std::endl;
                    JustinaHRI::waitAfterSay("I see the bar in my left side", 10000);       
                }
                nextState = SM_SEARCH_WAVING;     
                break;

            case SM_SEARCH_WAVING:
                std::cout << "State machine: SM_SEARCH_WAVING" << std::endl;
                //find = JustinaTasks::findWaving(-0.5, 0.55, 0.5, -0.1, -0.2, -0.4, 500, rectWav);
                find = JustinaTasks::findWaving(0, 0, 0, -0.1, -0.2, 0, 500, rectWav);
                if(find){
                    nextState = SM_ALIGN_WAVING;
                }
                else
                    nextState = SM_SEARCH_WAVING;
                break;

            case SM_ALIGN_WAVING:
                std::cout << "State machine: SM_ALIGN_WAVING" << std::endl;
                find = JustinaTasks::alignWithWaving(rectWav);
                if(find){
                    nextState = SM_WAIT_FOR_TAKE_ORDER;
                    JustinaHRI::waitAfterSay("Semeone asked for my service", 10000);
                    JustinaHRI::waitAfterSay("Do you want me take the order", 10000);
                    JustinaHRI::waitAfterSay("Tell me Justina yes for confirm", 10000);
                    JustinaHRI::waitAfterSay("Tell me Justina no for no attend", 10000);
                }else
                    nextState = SM_SEARCH_WAVING;
                break;

            case SM_FIND_PERSONS:
                std::cout << "State machine: SM_FIND_PERSONS" << std::endl;
                std::cout << "Curr pan position .->" << JustinaHardware::getHeadCurrentPan() << std::endl;
                JustinaManip::startHdGoTo(0.0, JustinaHardware::getHeadCurrentTilt());
                JustinaNavigation::moveDistAngle(0.0, JustinaHardware::getHeadCurrentPan(), 4000);
                JustinaManip::waitForHdGoalReached(3000);
                faces = JustinaVision::getFaces("");
                find = false;
                mapToClose.clear();
                for(int i = 0; i < faces.recog_faces.size(); i++){
                    vision_msgs::VisionFaceObject face = faces.recog_faces[i];
                    float fx_k, fy_k, fz_k, fx_w, fy_w, fz_w;
                    JustinaTools::transformPoint("/base_link", face.face_centroid.x, face.face_centroid.y, face.face_centroid.z, "/kinect_link", fx_k, fy_k, fz_k);
                    JustinaTools::transformPoint("/base_link", face.face_centroid.x, face.face_centroid.y, face.face_centroid.z, "/map", fx_w, fy_w, fz_w);
                    std::cout << "Restaurant SM.->fx_k:" << fx_k << ",fy_k:" << fy_k << ",fz_k" << fz_k << std::endl;
                    std::cout << "Restaurant SM.->fx_w:" << fx_w << ",fy_w:" << fy_w << ",fz_w" << fz_w << std::endl;
                    if(fabs(fx_k) <= 0.5){
                        std::vector<float> pos;
                        pos.push_back(fx_w);
                        pos.push_back(fy_w);
                        pos.push_back(fz_w);
                        ss.str("");
                        ss << "person_" << mapToClose.size();
                        mapToClose[mapToClose.size()] = pos;
                        find = true;
                    }
                }
                if(find){
                    nextState = SM_CLOSE_TO_CLIENT;
                }
                else{
                    float currx, curry, currtheta, nextx, nexty;
                    JustinaNavigation::getRobotPose(currx, curry, currtheta);
                    nextx = currx + 1.5 * cos(currtheta);
                    nexty = curry + 1.5 * sin(currtheta);
                    JustinaNavigation::getClose(nextx, nexty, currtheta, 60000);
                    nextState = SM_FIND_PERSONS;
                }
                break;

            case SM_WAIT_FOR_TAKE_ORDER:
                std::cout << "State machine: SM_WAIT_FOR_TAKE_ORDER" << std::endl;
                if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, 10000)){
                    if(lastRecoSpeech.find("justina yes") != std::string::npos){
                        JustinaHRI::waitAfterSay("Ok, I am getting close to the client", 6000);
                        nextState = SM_FIND_PERSONS;
                    }
                    else if(lastRecoSpeech.find("justina no") != std::string::npos)
                        nextState = SM_SEARCH_WAVING;
                }
                break;

            case SM_CLOSE_TO_CLIENT:
                std::cout << "State machine: SM_CLOSE_TO_CLIENT" << std::endl;
                float currx, curry, currtheta;
                float dist_to_head;
                ss.str("");
                ss << "person_" << indexToClose;
                it = mapToClose.find(indexToClose);
                vectorPos = it->second;
                JustinaNavigation::getRobotPose(currx, curry, currtheta);
                JustinaTasks::closeToGoalWithDistanceTHR(vectorPos[0], vectorPos[1], 1.5, 120000);
                dist_to_head = sqrt( pow(vectorPos[0], 2) + pow(vectorPos[1], 2));
                JustinaManip::startHdGoTo(atan2(vectorPos[1], vectorPos[0]) - currtheta, atan2(vectorPos[3] - 1.6, dist_to_head)); 

                JustinaHRI::waitAfterSay("Hi, I am Justina, I'm going to take you order", 10000);

                nextState = SM_FIRST_ORDER;
                
                /*locations = JustinaKnowledge::getKnownLocations();
                it_locations = locations.find(ss.str());

                if(it_locations != locations.end()){
                    it_locations->second[0];
                    JustinaTasks::closeToGoalWithDistanceTHR();
                }else
                    nextState = SM_FIRST_ORDER;*/
                break;

            case SM_FIRST_ORDER:
                std::cout << "State machine: SM_FIRST_ORDER" << std::endl;
                JustinaHRI::waitAfterSay("What is your order", 10000);	
                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                if(JustinaHRI::waitForSpecificSentence(startCommands, lastRecoSpeech, 10000)){
                    JustinaHRI::waitAfterSay("Do you want me", 10000);
                    JustinaHRI::waitAfterSay(lastRecoSpeech, 10000);
                    nextState=SM_FIRST_ORDER_CONFIRM;           
                }
                else{
                    contador_order++;
                    JustinaHRI::waitAfterSay("I am sorry, i can not understand you", 10000);
                    if(contador_order>4){
                        JustinaHRI::waitAfterSay("Ok, i will go to the bar location and i will be back with your order", 10000);
                        contador_order=0;
                        nextState=SM_FIRST_ORDER_CONFIRM;
                    }
                }
                break;

            case SM_FIRST_ORDER_CONFIRM:
                std::cout << "State machine: SM_FIRST_ORDER_CONFIRM" << std::endl;
                JustinaHRI::waitForUserConfirmation(userConfirmation, 7000);
                if(userConfirmation){
                    JustinaHRI::waitAfterSay("Ok, i will go to the bar location and i will be back with your order", 10000);
                    nextState = SM_RETURN_BAR;
                }
                else {
                     JustinaHRI::waitAfterSay("Ok", 10000);
                     SM_FIRST_ORDER;
                }
                break;	

            case SM_RETURN_BAR:
                std::cout << "State machine: SM_RETURN_BAR" << std::endl;
                if(!JustinaNavigation::getClose(0,0,0, 200000))
                    if(!JustinaNavigation::getClose(0,0,0, 200000))
                        JustinaNavigation::getClose(0,0,0, 200000);
                JustinaHRI::waitAfterSay("I arrived to bar location", 2000);
                JustinaHRI::waitAfterSay("I waiting for the next client", 2000);
                nextState=SM_SEARCH_WAVING;
            break;    
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

