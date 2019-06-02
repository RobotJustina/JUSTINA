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
//#include "string"
#include <sensor_msgs/LaserScan.h>

enum STATE{
    SM_INIT,
    SM_FIND_PERSON,
    SM_FIND_BAG,
    SM_GET_CLOSE_BAG,
    SM_WAIT_FOR_THE_BAG,
    SM_TAKE_BAG,
    SM_INSTRUCTIONS,
    SM_WAIT_FOR_OPERATOR,
    SM_MEMORIZING_OPERATOR,
    SM_WAIT_FOR_LEGS_FOUND,
    SM_FOLLOWING_PHASE,
    SM_BRING_GROCERIES,
    SM_BRING_GROCERIES_CONF,
    SM_BRING_GROCERIES_TAKE,
    SM_BAG_DELIVERY,
    SM_BAG_DELIVERY_PLACE,
    SM_LOOKING_HELP,
    SM_GUIDING_ASK,
    SM_GUIDING_HELP,
    SM_GUIDING_MEMORIZING_OPERATOR,
    SM_GUIDING_MEMORIZING_OPERATOR_ELF,
    SM_GUIDING_MEMORIZING_OPERATOR_SAY,
    SM_GUIDING_PHASE,
    SM_GUIDING_STOP,
    SM_GUIDING_CAR,
    SM_OPEN_DOOR,
    SM_FINAL_STATE,
    SM_RETURN_HOME,
    SM_HOKUYO_TEST
};


#define MAX_ATTEMPTS_RECOG 3
#define MAX_ATTEMPTS_CONF 3

sensor_msgs::LaserScan laser;
std::vector<float> laser_ranges;
bool door_isopen=false;
bool door_loc=false;
int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
float laser_l=0;


vision_msgs::VisionFaceObjects recognizeFaces (float timeOut, bool &recognized)
{
    recognized = false;
    int previousSize = 20;
    int sameValue = 0;
    boost::posix_time::ptime curr;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff;
    vision_msgs::VisionFaceObjects lastRecognizedFaces;

    do
    {
        lastRecognizedFaces = JustinaVision::getFaces();
        
        if(previousSize == 1)
            sameValue ++;
        
        if (sameValue == 3)
            recognized = true;

        else
        {
            previousSize = lastRecognizedFaces.recog_faces.size();
            recognized = false;
        }

        curr = boost::posix_time::second_clock::local_time();
        ros::spinOnce();
    }while(ros::ok() && (curr - prev).total_milliseconds()< timeOut && !recognized);

    std::cout << "recognized:" << recognized << std::endl;
    return lastRecognizedFaces;
}

void Callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    range=msg->ranges.size();
    range_c=range/2;
    range_i=range_c-(range/10);
    range_f=range_c+(range/10);
    std::cout<<"Range Size: "<< range << "\n ";
    std::cout<<"Range Central: "<< range_c << "\n ";
    std::cout<<"Range Initial: "<< range_i << "\n ";
    std::cout<<"Range Final: "<< range_f << "\n ";

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

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING HELP ME CARRY TEST..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    std::cout << system("pacmd set-default-source alsa_input.pci-0000_00_1f.3.analog-stereo") << std::endl;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    boost::posix_time::ptime prev;
    boost::posix_time::ptime curr;

    //int c_point=0,i=1;
    bool is_location;
    int nextState = SM_INIT;
    bool fail = false;
    bool success = false;
    bool giveToHuman = true;
    float x, y ,z;
    std::stringstream ss;
    std::vector<std::string> tokens;
    int attemptsRecogLoc = 0;
    int attemptsConfLoc = 0;
    int attemptsWaitContinue = 0;
    int maxAttemptsWaitContinue = 3;

    float robot_x, robot_y, robot_a;
    std::vector<std::string> yoloIds;
    yoloIds.push_back("person");

    std::string lastRecoSpeech;
    std::string location="";
    std::string room = "kitchen";
    std::vector<std::string> validCommandsStop;
    std::vector<std::string> validCommandsTake;
    validCommandsStop.push_back("here is the car");
    validCommandsStop.push_back("stop follow me");
    JustinaTasks::POSE poseRecog;
     
    int minDelayAfterSay = 0;
    int maxDelayAfterSay = 300;

    /*
    //places
    validCommandsTake.push_back("take this bag to the bedroom");
    validCommandsTake.push_back("get this bag to the bedroom");
    location="bedroom";
    
    validCommandsTake.push_back("take this bag to the bed");
    validCommandsTake.push_back("get this bag to the bed");
    location="bed";

    validCommandsTake.push_back("take this bag to the desk");
    validCommandsTake.push_back("get this bag to the desk");
    location="desk";

    validCommandsTake.push_back("take this bag to the side table");
    validCommandsTake.push_back("get this bag to the side table");
    location="side_table";
    
    validCommandsTake.push_back("take this bag to the living room");
    validCommandsTake.push_back("get this bag to the living room");
    location="living_room";

    validCommandsTake.push_back("take this bag to the couch");
    validCommandsTake.push_back("get this bag to the couch");
    location="couch";

    validCommandsTake.push_back("take this bag to the end table");
    validCommandsTake.push_back("get this bag to the end table");
    location="end_table";
    
    validCommandsTake.push_back("take this bag to the bookcase");
    validCommandsTake.push_back("get this bag to the bookcase");
    location="bookcase";

    validCommandsTake.push_back("take this bag to the dining room");
    validCommandsTake.push_back("get this bag to the dining room");
    location="dining_room";

    validCommandsTake.push_back("take this bag to the dining table");
    validCommandsTake.push_back("get this bag to the dining table");
    location="dining_table";

    validCommandsTake.push_back("take this bag to the kitchen");
    validCommandsTake.push_back("get this bag to the kitchen");
    location="kitchen";

    validCommandsTake.push_back("take this bag to the sink");
    validCommandsTake.push_back("get this bag to the sink");
    location="sink";

    validCommandsTake.push_back("take this bag to the dishwasher");
    validCommandsTake.push_back("get this bag to the dishwasher");
    location="dishwasher";

    validCommandsTake.push_back("take this bag to the counter");
    validCommandsTake.push_back("get this bag to the counter");
    location="counter";

    validCommandsTake.push_back("take this bag to the storage table");
    validCommandsTake.push_back("get this bag to the storage table");
    location="storage_table";

    validCommandsTake.push_back("take this bag to the cupboard");
    validCommandsTake.push_back("get this bag to the cupboard");
    location="cupboard";

    validCommandsTake.push_back("take this bag to the entrance");
    validCommandsTake.push_back("get this bag to the entrance");
    location="entrance";
    
    validCommandsTake.push_back("take this bag to the corridor");
    validCommandsTake.push_back("get this bag to the corridor");
    location="corridor";*/

    ros::Subscriber laser_subscriber;
    //laser_subscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, Callback_laser);  

    bool hokuyoRear = false;
    bool userConfirmation = false;
    bool follow_start=false;
    bool alig_to_place=true;
    int cont_z=3;

    vision_msgs::VisionFaceObjects faces;
    bool recog =false;
    int contChances=0;
    bool withLeftArm = true;

    //JustinaHRI::setInputDevice(JustinaHRI::RODE);
    //JustinaHRI::setOutputDevice(JustinaHRI::USB);
    //JustinaHRI::setVolumenInputDevice(JustinaHRI::RODE, 65000);
    JustinaHRI::setVolumenOutputDevice(JustinaHRI::USB, 80000);

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {  

            case SM_INIT:
                std::cout << "State machine: SM_INIT" << std::endl;	
                JustinaManip::startHdGoTo(0.0, 0.0);
                JustinaHRI::waitAfterSay("I am ready for the carry my luggage test", 2000, minDelayAfterSay);
                JustinaHRI::loadGrammarSpeechRecognized("CarryMyLuggage.xml");//load the grammar
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                nextState = SM_FIND_PERSON;
                break;

            case SM_FIND_PERSON:
                std::cout << "State machine: SM_LOOKING_HELP" << std::endl;
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                if(JustinaTasks::findYolo(yoloIds, poseRecog, JustinaTasks::STANDING, room)){
                    JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                    location = "person_loc";
                    JustinaKnowledge::addUpdateKnownLoc("person_loc", robot_x, robot_y, robot_a);
                    JustinaHRI::waitAfterSay("Hello, my name is justina, and i will help you to carry the luggage", 5000, minDelayAfterSay);
                    JustinaHRI::waitAfterSay("Please take the bag and say, justina continue", 5000, maxDelayAfterSay);
                    attemptsWaitContinue = 0;
                    nextState=SM_WAIT_FOR_THE_BAG;
                }else
                    JustinaHRI::waitAfterSay("I did not find anyone", 3000); 
                JustinaHRI::enableSpeechRecognized(true);
                break;

            case SM_FIND_BAG:
                std::cout << "State machine: SM_FIND_BAG" << std::endl;
                nextState = SM_GET_CLOSE_BAG;
                break;

            case SM_GET_CLOSE_BAG:
                std::cout << "State machine: SM_GET_CLOSE_BAG" << std::endl;
                nextState = SM_TAKE_BAG;
                break;

            case SM_WAIT_FOR_THE_BAG:
                std::cout << "State machine: SM_WAIT_FOR_THE_BAG" << std::endl;
                if(attemptsWaitContinue < maxAttemptsWaitContinue){
                    if(JustinaHRI::waitForSpecificSentence("justina continue", 5000))
                        nextState = SM_TAKE_BAG; 
                    else{
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::waitAfterSay("Please take the bag and say, justina continue", 5000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);
                        nextState = SM_WAIT_FOR_THE_BAG;
                    }
                    attemptsWaitContinue++;
                }
                else{
                    attemptsWaitContinue = 0;
                    nextState = SM_TAKE_BAG; 
                }
                break;

            case SM_TAKE_BAG:
                std::cout << "State machine: SM_GET_TAKE_BAG" << std::endl;
                JustinaHRI::say("i can not take the bag, but i will take the bag if you put the bag in my gripper");
                JustinaTasks::detectObjectInGripper("bag", withLeftArm, 10000);
                JustinaHRI::say("Tank you");
                JustinaManip::hdGoTo(0.0, 0.0, 3000);
                nextState = SM_INSTRUCTIONS;
                break;
            
            case SM_INSTRUCTIONS:
                std::cout << "State machine: SM_INSTRUCTIONS" << std::endl;
                JustinaHRI::waitAfterSay("Tell me, here is the car, when we reached the car location, please tell me, follow me, for start following you", 12000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                cont_z=0;
                nextState=SM_WAIT_FOR_OPERATOR;
                break;

            case SM_WAIT_FOR_OPERATOR:
                std::cout << "State machine: SM_WAIT_FOR_OPERATOR" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("follow me" , 15000))
                    nextState = SM_MEMORIZING_OPERATOR;
                else                    
                    cont_z++;    		

                if(cont_z>3){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("Please repeat the command", 5000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    cont_z=0;
                }
                break;

            case SM_MEMORIZING_OPERATOR:
                std::cout << "State machine: SM_MEMORIZING_OPERATOR" << std::endl;
                if(!follow_start){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("Human, please put in front of me", 3000, minDelayAfterSay);
                    JustinaHRI::enableLegFinder(true);
                }
                else
                    JustinaHRI::enableLegFinder(true);    
                nextState=SM_WAIT_FOR_LEGS_FOUND;
                break;

            case SM_WAIT_FOR_LEGS_FOUND:
                std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
                if(JustinaHRI::frontalLegsFound()){
                    if(follow_start){
                        std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("I found you, please walk", 4000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        JustinaHRI::startFollowHuman();
                        ros::spinOnce();
                        loop.sleep();
                        JustinaHRI::startFollowHuman();
                        nextState = SM_FOLLOWING_PHASE;
                    }
                    else{
                        std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("I found you, i will start to follow you human, please walk", 10000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//disable recognized speech
                        JustinaHRI::startFollowHuman();
                        follow_start=true;
                        nextState = SM_FOLLOWING_PHASE;
                    }
                }
                break;

            case SM_FOLLOWING_PHASE:
                std::cout << "State machine: SM_FOLLOWING_PHASE" << std::endl;
                if(JustinaHRI::waitForSpecificSentence(validCommandsStop, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find("here is the car") != std::string::npos || lastRecoSpeech.find("stop follow me") != std::string::npos){
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("is it the car location, please tell me robot yes, or robot no", 10000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        JustinaHRI::waitForUserConfirmation(userConfirmation, 5000);
                        if(userConfirmation){
                            JustinaHRI::stopFollowHuman();
                            JustinaHRI::enableLegFinder(false);
                            JustinaKnowledge::addUpdateKnownLoc("car_location");	
                            JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                            JustinaHRI::waitAfterSay("I stopped", 2000, minDelayAfterSay);
                            JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                            alig_to_place=false;
                            nextState = SM_BAG_DELIVERY_PLACE;
                            cont_z=8;
                            break;
                        }
                        else{
                            JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                            JustinaHRI::waitAfterSay("Ok, please walk", 3000, maxDelayAfterSay);
                            JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        }
                    }
                }
                if(!JustinaHRI::frontalLegsFound()){
                    std::cout << "State machine: SM_FOLLOWING_PHASE -> Lost human!" << std::endl;
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("I lost you, please put in front of me again", 5500, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    JustinaHRI::stopFollowHuman();
                    JustinaHRI::enableLegFinder(false);
                    nextState=SM_MEMORIZING_OPERATOR;
                }        
                break;

            /*case SM_BRING_GROCERIES:
                std::cout << "State machine: SM_BRING_GROCERIES" << std::endl; 
                if(cont_z > 3){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("I am ready to help you, Please tell me, take this bag to some location", 7000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    cont_z=0;
                }
                cont_z++;
                if(JustinaHRI::waitForSpecificSentence(validCommandsTake, lastRecoSpeech, 7000)){
                    attemptsRecogLoc++;
 
                    if(lastRecoSpeech.find("this bag to the bedroom") != std::string::npos){
                        location="bedroom";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the bed") != std::string::npos){
                        location="bed";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the desk") != std::string::npos){
                        location="desk";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the side table") != std::string::npos){
                        location="side_table";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the living room") != std::string::npos){
                        location="living_room";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the end table") != std::string::npos){
                        location="end_table";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the bookcase") != std::string::npos){
                        location="bookcase";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    
                    else if(lastRecoSpeech.find("this bag to the couch") != std::string::npos){
                        location="couch";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the dining room") != std::string::npos){
                        location="dining_room";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the dining table") != std::string::npos){
                        location="dining_table";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(lastRecoSpeech.find("this bag to the kitchen") != std::string::npos){
                        location="kitchen";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }


                    else if(lastRecoSpeech.find("this bag to the sink") != std::string::npos){
                        location="sink";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
 
                    else if(lastRecoSpeech.find("this bag to the dishwasher") != std::string::npos){
                        location="dishwasher";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
 
                    else if(lastRecoSpeech.find("this bag to the counter") != std::string::npos){
                        location="counter";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    
                    else if(lastRecoSpeech.find("this bag to the storage table") != std::string::npos){
                        location="storage_table";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    
                    else if(lastRecoSpeech.find("this bag to the cupboard") != std::string::npos){
                        location="cupboard";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    
                    else if(lastRecoSpeech.find("this bag to the entrance") != std::string::npos){
                        location="entrance";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    
                    else if(lastRecoSpeech.find("this bag to the corridor") != std::string::npos){
                        location="corridor";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }

                    else if(attemptsRecogLoc >= MAX_ATTEMPTS_RECOG){
                        location = "counter";
                        alig_to_place=true;
                        nextState = SM_BRING_GROCERIES_TAKE;
                    } 
                    if(location.compare("") != 0 && nextState == SM_BRING_GROCERIES_CONF){
                        ss.str("");
                        ss << "Do you want me take this bag to the "; 
                        tokens.clear();
                        boost::algorithm::split(tokens, location, boost::algorithm::is_any_of("_"));
                        for(int i = 0; i < tokens.size(); i++)
                            ss << tokens[i] << " ";
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay(ss.str(), 5000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//disable recognized speech
                    }

                }
                break;

            case SM_BRING_GROCERIES_CONF:
                std::cout << "State machine: SM_BRING_GROCERIES_CONF" << std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                JustinaHRI::waitForUserConfirmation(userConfirmation, 7000);
                attemptsConfLoc++;
                if(userConfirmation)
                    nextState = SM_BRING_GROCERIES_TAKE;
                else if(attemptsConfLoc < MAX_ATTEMPTS_CONF){
                    nextState = SM_BRING_GROCERIES;
                    cont_z = 8;
                }
                else
                    nextState = SM_BRING_GROCERIES_TAKE;
                break;

            case SM_BRING_GROCERIES_TAKE:    
                std::cout << "State machine: SM_BRING_GROCERIES_TAKE" << std::endl;
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                JustinaManip::startHdGoTo(0.0, 0.0);
                JustinaHRI::waitAfterSay("Please put in front of me to see your face", 3000);
                ros::Duration(1.0).sleep();
                while(!recog && contChances < 3)
                {
                    faces = recognizeFaces (10000, recog);
                    JustinaVision::startFaceRecognition(false);
                    contChances++;
                }

                if(faces.recog_faces.size()==0)
                {
                    JustinaHRI::say("Sorry, I cannot see anybody in front of me");
                    ros::Duration(1.5).sleep();
                    JustinaHRI::say("i can not take the bag form your hand but i will take the bag if you put the bag in my gripper");
                    ros::Duration(1.0).sleep();
                    JustinaTasks::detectObjectInGripper("bag", true, 20000);
                    withLeftArm = true;
                    ros::Duration(1.0).sleep();
                }
                else{
                    JustinaManip::startHdGoTo(0.0, -0.4);
                    JustinaHRI::say("Ready, now wait for the next instruction");
                    JustinaHRI::say("Please put your hand with the bag in front of me, in midle hight ");
                    if(JustinaTasks::graspObjectFromHand(faces.recog_faces[0].face_centroid, withLeftArm))
                        std::cout << "test succesfully" << std::endl;
                    else
                    {
                        JustinaHRI::say("sorry i can not see your hand");
                        ros::Duration(1.0).sleep();
                        JustinaHRI::say("i can not take the bag form your hand but i will take the bag if you put the bag in my gripper");
                        ros::Duration(1.0).sleep();
                        JustinaTasks::detectObjectInGripper("bag", true, 7000);
                        withLeftArm = true;
                        ros::Duration(1.0).sleep();
                    }
                }

                //JustinaTasks::detectBagInFront(true, 20000);

                ss.str("");
                ss << "Ok human, I will go to the "; 
                tokens.clear();
                boost::algorithm::split(tokens, location, boost::algorithm::is_any_of("_"));
                for(int i = 0; i < tokens.size(); i++)
                    ss << tokens[i] << " ";
                ss << "and i will be back to the car";
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                //JustinaManip::startTorsoGoTo(0.3, 0.0, 0.0);
                nextState=SM_BAG_DELIVERY;     
                break;*/

            case SM_BAG_DELIVERY:
                std::cout << "State machine: SM_BAG_DELIVERY" << std::endl;
                
                if(!JustinaKnowledge::existKnownLocation(location)){
                    std::cout << "SM_BAG_DELIVERY: NO LOCATION!" << std::endl;
                    location="counter";
                    alig_to_place=true;
                }
                
                std::cout << "Location -> " << location << std::endl;
                if(!JustinaNavigation::getClose(location, 200000))
                    if(!JustinaNavigation::getClose(location, 200000))
                        JustinaNavigation::getClose(location, 200000);
                JustinaHRI::waitAfterSay("I arrived", 2000);
                nextState=SM_BAG_DELIVERY_PLACE;

                break;

            case SM_BAG_DELIVERY_PLACE:
                std::cout << "State machine: SM_BAG_DELIVERY_PLACE" << std::endl;
                if(!giveToHuman){
                    JustinaHRI::waitAfterSay("I will delivery the bag", 3000);
                    if(alig_to_place){
                        if(!JustinaTasks::alignWithTable(0.35)){
                            JustinaNavigation::moveDist(0.15, 3000);
                            if(!JustinaTasks::alignWithTable(0.35)){
                                JustinaNavigation::moveDist(0.15, 3000);
                                JustinaTasks::alignWithTable(0.35);   
                            }
                        }
                        if(!JustinaTasks::placeObject(withLeftArm, 0.35, true)){
                            if(!JustinaTasks::placeObject(withLeftArm, 0.35, true))
                                if(!JustinaTasks::placeObject(withLeftArm, 0.35, true))
                                {
                                    if(withLeftArm){
                                        JustinaManip::laGoTo("place_bag_floor", 4000);
                                        JustinaManip::startLaOpenGripper(0.7);
                                        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                                        JustinaManip::laGoTo("home", 4000);
                                        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                                        JustinaManip::startLaOpenGripper(0);
                                    }
                                    else{
                                        JustinaManip::raGoTo("place_bag_floor", 4000);
                                        JustinaManip::startRaOpenGripper(0.7);
                                        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                                        JustinaManip::raGoTo("home", 4000);
                                        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                                        JustinaManip::startRaOpenGripper(0);
                                    }

                                }
                        } 
                        if(withLeftArm){   
                            JustinaManip::laGoTo("home", 4000);
                            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                            JustinaManip::startLaOpenGripper(0);
                        }
                        else{
                            JustinaManip::raGoTo("home", 4000);
                            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                            JustinaManip::startRaOpenGripper(0);
                        }
                    }
                    else{
                        if(withLeftArm){
                            JustinaManip::laGoTo("place_bag_floor", 4000);
                            JustinaManip::startLaOpenGripper(0.7);
                            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                            JustinaManip::laGoTo("home", 4000);
                            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));  
                            JustinaManip::startLaOpenGripper(0);
                        }   
                        else{
                            JustinaManip::raGoTo("place_bag_floor", 4000);
                            JustinaManip::startRaOpenGripper(0.7);
                            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                            JustinaManip::raGoTo("home", 4000);
                            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));  
                            JustinaManip::startRaOpenGripper(0);
                        }
                    }    

                    JustinaNavigation::moveDistAngle(-0.2, 0.0, 1000);
                    nextState=SM_RETURN_HOME;
                }
                else{
                    JustinaTasks::dropObject("lugagge", withLeftArm, 8000);
                    nextState=SM_RETURN_HOME;
                }

                break;
            
            case SM_RETURN_HOME:
                std::cout << "State machine: SM_BAG_DELIVERY" << std::endl;
                if(!JustinaKnowledge::existKnownLocation(location)){
                    std::cout << "SM_BAG_DELIVERY: NO LOCATION!" << std::endl;
                    location=room;
                }
                
                std::cout << "Location -> " << location << std::endl;
                if(!JustinaNavigation::getClose(location, 200000))
                    if(!JustinaNavigation::getClose(location, 200000))
                        JustinaNavigation::getClose(location, 200000);
                JustinaHRI::waitAfterSay("I arrived", 2000);
                nextState=SM_FINAL_STATE;
                break;

            /*case SM_LOOKING_HELP:
                std::cout << "State machine: SM_LOOKING_HELP" << std::endl;
                
                JustinaHRI::waitAfterSay("I will look for help", 3000);
                if(JustinaTasks::findPerson("", -1, JustinaTasks::STANDING, false, location))
                    nextState=SM_GUIDING_ASK;
                else
                    JustinaHRI::waitAfterSay("I did not find anyone", 3000); 

                break;

            case SM_GUIDING_ASK:
                std::cout << "State machine: SM_GUIDING_ASK" << std::endl;
                JustinaHRI::waitAfterSay("Human, can you help me bring some bags please, please tell me robot yes, or robot no", 12000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                JustinaHRI::waitForUserConfirmation(userConfirmation, 15000);
                if(userConfirmation){
                    nextState = SM_GUIDING_MEMORIZING_OPERATOR_SAY;
                }
                else {
                    nextState = SM_LOOKING_HELP;
                    JustinaNavigation::moveDistAngle(0.0, 1.5708, 2000);
                }	    
                break;        

            case SM_GUIDING_MEMORIZING_OPERATOR_SAY:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR_SAY" << std::endl;
                JustinaHRI::waitAfterSay("I will guide you to the car location", 4000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                location="entrance_door";
                if(!JustinaKnowledge::existKnownLocation(location)){
                    std::cout << "SM_BAG_DELIVERY: NO LOCATION!" << std::endl;
                    location="car_location";
                }
                cont_z=0;
                nextState=SM_GUIDING_MEMORIZING_OPERATOR_ELF;
                break;

            case SM_GUIDING_MEMORIZING_OPERATOR_ELF:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR_ELF" << std::endl;
                JustinaHRI::enableLegFinderRear(true); ////igcdkjgdhghksd
                nextState = SM_GUIDING_MEMORIZING_OPERATOR;

                break;

            case SM_GUIDING_MEMORIZING_OPERATOR:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR" << std::endl;
                hokuyoRear = JustinaHRI::rearLegsFound();
                if(hokuyoRear){
                    JustinaHRI::waitAfterSay("Ok, let us go", 2500);
                    nextState=SM_GUIDING_PHASE;
                    JustinaNavigation::startGetClose(location);
                    cont_z=0;
                }
                else{
                    if(cont_z>3){
                        JustinaHRI::waitAfterSay("Human, stand behind me", 3000);
                        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                        cont_z=0;
                    }
                    cont_z++;
                }

                break;    

            case SM_GUIDING_PHASE:
                std::cout << "State machine: SM_GUIDING_PHASE" << std::endl;
                std::cout << "Location -> " << location << std::endl;
                hokuyoRear = JustinaHRI::rearLegsFound();
                std::cout << "hokuyoRear -> " << hokuyoRear << std::endl;

                if(!hokuyoRear)
                    nextState=SM_GUIDING_STOP;

                if(JustinaNavigation::isGlobalGoalReached()){
                    laser_subscriber = n.subscribe<sensor_msgs::LaserScan>("/hardware/scan", 1, Callback_laser);
                    ros::spinOnce();
                    loop.sleep();
                    nextState=SM_HOKUYO_TEST;
                }

                break;

            case SM_GUIDING_STOP:
                std::cout << "State machine: SM_GUIDING_STOP" << std::endl;
                JustinaHardware::stopRobot();
                JustinaHardware::stopRobot();
                JustinaHardware::stopRobot();
                ros::spinOnce();
                JustinaHRI::waitAfterSay("I lost you", 1500);
                JustinaHRI::enableLegFinderRear(false);
                JustinaHRI::waitAfterSay("Human, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                nextState=SM_GUIDING_MEMORIZING_OPERATOR_ELF;
                break;

            case SM_HOKUYO_TEST:    
                std::cout << "State machine: SM_HOKUYO_TEST" << std::endl;

                if(cont_z>5)
                    nextState=SM_GUIDING_CAR;
                else
                    cont_z++;    
                break;

            case SM_GUIDING_CAR:
                std::cout << "State machine: SM_GUIDING_CAR" << std::endl;
                if(!door_loc){
                    //JustinaHRI::waitAfterSay("Here is the door", 2500);
                    std::cout << "Here is the door" << std::endl;
                    JustinaHRI::enableLegFinderRear(false);
                    if(door_isopen){
                        JustinaHRI::waitAfterSay("The door is open", 2500);
                        std::cout << "The door is open" << std::endl;
                        location="car_location";
                        door_loc=true;
                        laser_subscriber.shutdown();
                        door_loc=true;
                        nextState=SM_GUIDING_MEMORIZING_OPERATOR_ELF;
                    }
                    else{
                        std::cout << "the door is close" << std::endl;
                        cont_z=10; 
                        nextState=SM_OPEN_DOOR;
                    }

                }
                else{
                    std::cout << "State machine: SM_GUIDING_CAR" << std::endl;
                    JustinaHRI::waitAfterSay("Here is the car, please help us", 2500);
                    JustinaHRI::waitAfterSay("I have finished the test", 2500);
                    JustinaHRI::enableLegFinderRear(false);
                    nextState=SM_FINAL_STATE;
                }        
                break;

            case SM_OPEN_DOOR:
                std::cout << "State machine: SM_OPEN_DOOR" << std::endl;
                if(door_isopen){
                    JustinaHRI::waitAfterSay("Thank you", 2500);
                    std::cout << "Tank You" << std::endl;
                    location="car_location";
                    laser_subscriber.shutdown();
                    door_loc=true;
                    nextState= SM_GUIDING_MEMORIZING_OPERATOR_ELF;
                }

                else{
                    if(cont_z>5){
                        std::cout << "Huma Open the door" << std::endl;
                        JustinaHRI::waitAfterSay("Human, can you open the door please", 4500);
                        //JustinaHRI::waitAfterSay("Please move, i will move backwards", 10000);
                        sleep(1.0);
                        //JustinaNavigation::moveDist(-0.4, 4000);
                        cont_z=0;
                    }
                    std::cout << "Open the door time" << std::endl;
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000)); 
                    cont_z++;        
                }

                break;*/

            case SM_FINAL_STATE:
                std::cout << "State machine: SM_FINAL_STATE" << std::endl;
                JustinaHRI::say("I have finished the test");
                success = true;
                break;

        }

        ros::spinOnce();
        loop.sleep();
    }



    return 1;
}

