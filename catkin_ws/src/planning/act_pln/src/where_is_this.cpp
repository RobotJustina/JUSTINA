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
#define GRAMMAR_QUESTIONS "where_is_this.xml"
#define GRAMMAR_PLACES "incomplete_place.xml"
#define GRAMMAR_COMMANDS "commands.xml"

/**
ss.str("")
ss << "(assert (where_is_this " <<  nameObject << " 1))";
std::string query;
JustinaRepresentation::strQueryKDB(ss.str(),query,1000);
std::cout << ".-> SM_TALK_TO_OPERATOR" << std::endl;
**/

bool graspObjectColorCupBoardFeedback2(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);
bool pouringCereal(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);
bool placeSpoon(float x, float y, float z, bool withLeftArm, std::string colorObject, bool usingTorse);

enum TYPE_AREA{OFFICE, BEDROOM, LIVINGROOM,KITCHEN };

enum STATE{
    SM_INIT,
    SM_WAIT_FOR_OPEN,
    SM_NAVIGATE_TO_START,
    SM_TALK_TO_OPERATOR,
    SM_INTERMEDIATE,
    SM_GO_TO_PLACE,
    SM_EXPLAIN,
    SM_RETURN,
    SM_FINISH_TEST,
    SM_GO_TO_INFO_POINT
};

std::string lastRecoSpeech;
std::string lastInteSpeech;

std::string test("serve the breakfast");

void ArmLiving1()
{
    
    std::vector<float> currPose;
    JustinaManip::getRaCurrentPos(currPose);
    if (currPose.size() == 7) 
    {
        currPose[0] =0.9;
        currPose[1] =-0.2;
        currPose[2] =0;
        currPose[3] =1.5;
        currPose[4] =0;
        currPose[5] =-1.0;
        currPose[6] =0;
            
        JustinaManip::raGoToArticular(currPose, 3000);
    }   
            
}

void ArmLiving2()
{
    
   
    std::vector<float> currPose;
    JustinaManip::getLaCurrentPos(currPose);
    if (currPose.size() == 7) 
    {
        currPose[0] =1.1;
        currPose[1] =-0.7;
        currPose[2] =0;
        currPose[3] =0.9;
        currPose[4] =0;
        currPose[5] =-0.6;
        currPose[6] =0;
        
        JustinaManip::laGoToArticular(currPose, 3000);
    }   
            
}


void here()
{
    
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    std::vector<float> currPose;
    JustinaManip::getRaCurrentPos(currPose);
    if (currPose.size() == 7) 
    {
        currPose[0] =-0.2;
        currPose[1] =0.0;
        currPose[2] =-0.3;
        currPose[3] =1.8;
        currPose[4] =0;
        currPose[5] =0.0;
        currPose[6] =0;
            
        JustinaManip::raGoToArticular(currPose, 3000);
    }   
            
}

bool pointingInOffice(int n)
{
    switch(n){

        case 0:
        JustinaNavigation::moveDistAngle(0,0.5,2000);
        ArmLiving2();
        JustinaHRI::waitAfterSay("Here is the  office, there is a desk and a chair on my left ", 4000, MIN_DELAY_AFTER_SAY);
        
        break;
        case 1:
            JustinaNavigation::moveDistAngle(0,-0.5,2000); 
            ArmLiving1();
            JustinaHRI::waitAfterSay("Here, there is the coat hanger next to the entrance door",4000,MIN_DELAY_AFTER_SAY);
            std::cout << "Here, there is the coat hanger next to the entrance door" << std::endl;
        break;
        case 2:
            //JustinaNavigation::moveDistAngle(0,-0.5,2000); 
            ArmLiving1();
            JustinaHRI::waitAfterSay("The entrance is in this room, also a coat hanger",4000,MIN_DELAY_AFTER_SAY);
        break;
        case 3:
            //JustinaNavigation::moveDistAngle(0,0.5,2000); 
            ArmLiving2();
            JustinaHRI::waitAfterSay("On my left there is the desk, next to it there is the shoe rack ",4000,MIN_DELAY_AFTER_SAY);
        break;
        case 4:
            JustinaNavigation::moveDistAngle(0,-0.5,2000); 
            ArmLiving1();
            JustinaHRI::waitAfterSay("In this place are a one desk, a shoe rack and  the  entrance door",4000,MIN_DELAY_AFTER_SAY);
            std::cout << "Here, there is the coat hanger next to the entrance door" << std::endl;
        break;
        default:
            JustinaNavigation::moveDistAngle(0,0.5,2000);
            ArmLiving1();
            JustinaHRI::waitAfterSay("Here is the  office, there is a desk and a chair on my left ", 4000, MIN_DELAY_AFTER_SAY);
            return false;
        break;
    }
    JustinaManip::startLaGoTo("navigation");
    JustinaManip::startRaGoTo("navigation");
    JustinaManip::waitForLaGoalReached(2000);
    JustinaManip::waitForRaGoalReached(2000);
    JustinaHRI::waitAfterSay("Please follow me ,", 4000, MIN_DELAY_AFTER_SAY);
    return true;
}

bool pointingInLivingRoom(int n)
{
        //JustinaManip::startLaGoTo("navigation");
        

        switch(n){

        case 0:
            
            ArmLiving1();
            JustinaHRI::waitAfterSay("Here is the living room you can rest on the sofa", 4000, MIN_DELAY_AFTER_SAY);
            JustinaManip::startRaGoTo("navigation");
            JustinaManip::startLaGoTo("navigation");
                          
            
        break;
        case 1:
            ArmLiving2();
            JustinaHRI::waitAfterSay("Here is the living room, You can watch a movie on this tv.", 4000, MIN_DELAY_AFTER_SAY);
            JustinaManip::startRaGoTo("navigation");
            
        break;
        case 2:
            here();
            JustinaHRI::waitAfterSay("Here is the living in front there is the coffe table", 4000, MIN_DELAY_AFTER_SAY);
            JustinaManip::startRaGoTo("navigation");
            
        break;
        case 3:
            here();
            JustinaHRI::waitAfterSay("Here is the living in front there is the couch", 4000, MIN_DELAY_AFTER_SAY);
            JustinaManip::startRaGoTo("navigation");
        break;
        default:
            JustinaNavigation::moveDistAngle(0,0.5,3000);

            JustinaManip::startLaGoTo("navigation");
            ArmLiving1();
            JustinaHRI::waitAfterSay("here is the living room and there is an armchair  and a coffe table.", 4000, MIN_DELAY_AFTER_SAY); 
            return false;
        break;
    }

    JustinaManip::startLaGoTo("navigation");
    JustinaManip::startRaGoTo("navigation");
    JustinaManip::waitForLaGoalReached(2000);
    JustinaManip::waitForRaGoalReached(2000);

    JustinaHRI::waitAfterSay("Please follow me ,", 4000, MIN_DELAY_AFTER_SAY);
    return true;
}


bool pointingInKitchen(int n)
{
        //JustinaManip::startLaGoTo("navigation");
        //boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

        switch(n){

        case 0:
            
            here();
            JustinaHRI::waitAfterSay("Here is the kitchen in front there is the  table", 4000, MIN_DELAY_AFTER_SAY);
            JustinaManip::startRaGoTo("navigation");            
            
        break;
        case 1:
            here();
            JustinaHRI::waitAfterSay("Here is the kitchen in front at the corner there is the cabinet", 4000, MIN_DELAY_AFTER_SAY);
            JustinaManip::startRaGoTo("navigation");
            
        break;
        case 2:
            ArmLiving2();
            JustinaHRI::waitAfterSay("Here is the kitchen on the left there is the sink and the dishwasher", 4000, MIN_DELAY_AFTER_SAY);
            JustinaManip::startRaGoTo("navigation");
        break;
        default:
           ArmLiving2();
            JustinaHRI::waitAfterSay("Here is the kitchen on the left there is the fridge", 4000, MIN_DELAY_AFTER_SAY);
            JustinaManip::startRaGoTo("navigation");
            return false;
        break;
    }

    JustinaManip::startLaGoTo("navigation");
    JustinaManip::startRaGoTo("navigation");
    JustinaManip::waitForLaGoalReached(2500);
    JustinaManip::waitForRaGoalReached(2500);
    JustinaHRI::waitAfterSay("Please follow me ,", 4000, MIN_DELAY_AFTER_SAY);
    return true;
}



bool pointingInBedRoom(int n)
{
        //JustinaManip::startLaGoTo("navigation");
        //boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

        switch(n){

        case 0:
            
            here();
            JustinaHRI::waitAfterSay("Here is the bedroom in front of me there is the bed", 4000, MIN_DELAY_AFTER_SAY);
            JustinaManip::startRaGoTo("navigation");           
            
        break;
        case 1:
            ArmLiving1();
            JustinaHRI::waitAfterSay("Here is the bedroom, on the right at the corner there is the shelf", 4000, MIN_DELAY_AFTER_SAY);
            JustinaManip::startRaGoTo("navigation");
            
        break;
        default:
           ArmLiving1();
            JustinaHRI::waitAfterSay("Here is the kitchen on the right there is the sidetable", 4000, MIN_DELAY_AFTER_SAY);
            JustinaManip::startRaGoTo("navigation");
            
            return false;
        break;
    }

    JustinaManip::startLaGoTo("navigation");
    JustinaManip::startRaGoTo("navigation");
    JustinaManip::waitForLaGoalReached(2500);
    JustinaManip::waitForRaGoalReached(2500);
    JustinaHRI::waitAfterSay("Please follow me ,", 4000, MIN_DELAY_AFTER_SAY);
    return true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "serve_the_breakfast_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    //FLAGS
    bool gui = true;
    bool flagOnce = true;
    bool success = false;
    bool withLeft = false;
    bool findSeat = false;
    bool doorOpenFlag = false;
    bool flagNoBowl= false;

    bool onlyBowl = false;
    bool first = true;

    //COUNTERS
    int countGraspAttemps = 0;
    int findSeatCount = 0;
    int attempsDoorOpend = 0;
    
    int kitchenCounter=0;
    int livingRoomCounter=0;
    int officeCounter=0;
    int bedRoomCounter=0;  

    
    //SPEACH AND GRAMAR
    std::string param, typeOrder;
    std::vector<std::string> names;
    std::vector<std::string> drinks;
    std::string lastName, lastDrink;
    std::string grammarNamesID = "receptionistNames";
    std::string grammarDrinksID = "serve_the_breakfastDrinks";
    std::string grammarCommandsID = "serve_the_breakfastCommands";
    
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

    JustinaHRI::usePocketSphinx = false;


    std::string placeLoc = "tv";
    std::string place = " tv ";
    std::string roomPlace=" living room "; 
    std::string insidePlace="in front of the sofas";
    std::string stringLocation="";

        //LOCATIONS
    std::string startLoc = "bed";// "start_point";
    int whereIm=BEDROOM;

    std::string query;

    int infopointAttemps =0;
    int placeAttemps =0;

    STATE state = SM_INIT;// SM_NAVIGATE_TO_START;//////SM_PLACE_MILK;//SM_LEAVE_CEREAL;//SM_GO_FOR_CEREAL;//SM_LOOK_FOR_TABLE;//SM_INIT;//SM_SEARCH_BOWL;//SM_PLACE_SPOON;//SM_GO_TO_KITCHEN;//
   
    JustinaRepresentation::initKDB("",false,20000);

    while(ros::ok() && !success){

        switch(state){
            case SM_INIT:

                //JustinaNavigation::getRaCurrentPos()
                //JustinaKnowledge::addOrUpdateLocation()
                
                std::cout << test << ".-> State SM_INIT: Init the test." << std::endl;
                
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaManip::startTorsoGoTo(0.10, 0, 0);
                JustinaManip::waitForTorsoGoalReached(3000);
                JustinaHRI::waitAfterSay("I am ready for the test", 6000, MIN_DELAY_AFTER_SAY);
                state = SM_WAIT_FOR_OPEN;
                
                break;

            case SM_WAIT_FOR_OPEN:
                
                std::cout << test << ".-> State SM_WAIT_FOR_OPEN: Wait for open the door." << std::endl;
                
                JustinaHRI::waitAfterSay("Human, can you open the door please", 6000, MIN_DELAY_AFTER_SAY);
                if( JustinaNavigation::doorIsOpen(0.9, 2000) || attempsDoorOpend >= MAX_ATTEMPTS_DOOR )
                {
                    state = SM_NAVIGATE_TO_START;
                    
                    JustinaHRI::waitAfterSay("Thank you", 4000, MIN_DELAY_AFTER_SAY);
                    JustinaNavigation::moveDist(2.5,5000);
                }
                else
                    attempsDoorOpend++;

            break;


            case SM_NAVIGATE_TO_START:
                
                std::cout << test << ".-> State SM_NAVIGATE_TO_START: Navigate to the start point " << std::endl;
                
                JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_PLACES);
                JustinaHRI::waitAfterSay("Please tell me where is information point, for example  at the coffe table ", 8000, MIN_DELAY_AFTER_SAY);
            
                JustinaHRI::enableSpeechRecognized(true);
                

                if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, TIMEOUT_SPEECH))
                {
                    if(JustinaRepresentation::stringInterpretation(lastRecoSpeech, lastInteSpeech))
                    {
                        JustinaHRI::enableSpeechRecognized(false);
                        
                        std::cout << "UUUNOO" << std::endl;

                        startLoc = lastInteSpeech;

                        if( lastInteSpeech=="living_room" )
                            startLoc="living_room_";

                        boost::replace_all(lastInteSpeech,"_"," ");

                        
                        ss.str("");
                        ss << "Is the information point at the " << lastInteSpeech << ", say robot yes or robot no.";
                        

                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_COMMANDS);
                        JustinaHRI::waitAfterSay(ss.str(), 10000, MAX_DELAY_AFTER_SAY);
                        
                        JustinaHRI::enableSpeechRecognized(true);

                        if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, TIMEOUT_SPEECH))
                        {
                            if(lastRecoSpeech.find("yes") != std::string::npos ||  infopointAttemps >= 2)
                            {
                                JustinaHRI::enableSpeechRecognized(false);
                                ss.str("");
                                ss << "Ok, I am going to the " << lastInteSpeech << " information point ";
                                JustinaHRI::waitAfterSay(ss.str(), 6000, MAX_DELAY_AFTER_SAY);  
                                state = SM_GO_TO_INFO_POINT;          
                            }
                            else
                            {
                                std::cout << "DOSSS" << std::endl;
                                JustinaHRI::waitAfterSay(" I did not understand you.", 3000, MAX_DELAY_AFTER_SAY);
                                infopointAttemps++;
                            }
                        }
                        else
                        {
                            std::cout << "TRESS" << std::endl;
                            JustinaHRI::waitAfterSay(" I did not understand you.", 3000, MAX_DELAY_AFTER_SAY);
                             infopointAttemps++;
                        }
                   
                    }
                }else{
                    std::cout << "CCCUAAATRRROO" << std::endl;
                    JustinaHRI::waitAfterSay(" I did not understand you.", 3000, MAX_DELAY_AFTER_SAY);
                     infopointAttemps++;
                }

                std::cout << " CIIIINCOO " << std::endl;
                

            break;


            case SM_GO_TO_INFO_POINT:

                if(!JustinaNavigation::getClose(startLoc, 80000) )
                    JustinaNavigation::getClose(startLoc, 80000); 
                

                JustinaHRI::waitAfterSay("I have reached to the information point. ", 4000, MIN_DELAY_AFTER_SAY);

                JustinaManip::startLaGoTo("navigation");
                JustinaManip::startRaGoTo("navigation");
                JustinaManip::waitForLaGoalReached(2500);
                JustinaManip::waitForRaGoalReached(2500);

                if( lastInteSpeech == "tv"  || lastInteSpeech == "couch"  || lastInteSpeech == "armchair" || lastInteSpeech == "coffe table" || lastInteSpeech == "trash bin" || lastInteSpeech == "sideboard" || lastInteSpeech == "display cabinet" || lastInteSpeech == "living room" ) 
                    whereIm = LIVINGROOM;
                else if ( lastInteSpeech == "shoe rack" || lastInteSpeech == "safe"|| lastInteSpeech == "coat hanger"|| lastInteSpeech == "desk"|| lastInteSpeech == "office" ) 
                    whereIm = OFFICE;
                else if ( lastInteSpeech =="bed" || lastInteSpeech =="bedroom chest"  || lastInteSpeech =="sidetable" || lastInteSpeech =="shelf" || lastInteSpeech =="bedroom" )
                    whereIm = BEDROOM;
                else if ( lastInteSpeech == "sink" || lastInteSpeech == "dishwasher"  || lastInteSpeech == "fridge"  || lastInteSpeech == "kitchen table"  || lastInteSpeech == "kitchen cabinet" || lastInteSpeech == "island"  || lastInteSpeech == "kitchen" )
                    whereIm = KITCHEN;
            
                std::cout << "/////////////// Last int speech " << lastInteSpeech << "ID " << whereIm << std::endl;

                state = SM_TALK_TO_OPERATOR;       
            break;


            case SM_TALK_TO_OPERATOR:
                                
                JustinaHRI::enableSpeechRecognized(false);
                JustinaHRI::waitAfterSay("Im ready, please tell me for example  where is the bed ", 4000, MIN_DELAY_AFTER_SAY);
                JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_QUESTIONS);
                JustinaHRI::enableSpeechRecognized(true);
                
                
                //lastRecoSpeech="where is the kitchen cabinet";

                if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, TIMEOUT_SPEECH))
                //if(1)
                {
                  if(JustinaRepresentation::stringInterpretation(lastRecoSpeech, lastInteSpeech))
                    //if(1)
                    {
                        std::cout << ":::::::: " << lastInteSpeech;

                        //coat_hanger   trash bitn     
                        if( lastInteSpeech == "tv" ) { placeLoc = "tv"; place=" tv "; roomPlace=" living room "; insidePlace="in front of the sofas" ;  }
                        else if ( lastInteSpeech == "couch")  { placeLoc = "couch"; place=" couch "; roomPlace=" living room "; insidePlace=""; }
                        else if ( lastInteSpeech == "armchair") { placeLoc = "armchair"; place=" armchair "; roomPlace=" living room "; insidePlace="in fron of the coffe table";}
                        else if ( lastInteSpeech == "coffe_table") { placeLoc = "coffe_table"; place=" coffe table "; roomPlace=" living room "; insidePlace="in front of the sofas"; }
                        else if ( lastInteSpeech == "trash_bin") { placeLoc = "trash_bin"; place=" trash bin "; roomPlace=" living room "; insidePlace=""; }
                        else if ( lastInteSpeech == "sideboard") { placeLoc = "sideboard"; place=" side board "; roomPlace=" living room ";  insidePlace="";}
                        else if ( lastInteSpeech == "display_cabinet") { placeLoc = "display_cabinet"; place=" display cabinet "; roomPlace=" living room "; insidePlace="and tv is on it."; }
                        else if ( lastInteSpeech == "living_room") { placeLoc = "living_room"; place=" living room "; roomPlace=" is the room between the kitchen and the office ";insidePlace=""; }

                        else if ( lastInteSpeech == "shoe_rack") { placeLoc = "shoe_rack"; place=" shoe rack "; roomPlace=" office "; insidePlace="near to the entrance"; }
                        else if ( lastInteSpeech == "safe") { placeLoc = "safe"; place=" safe "; roomPlace=" office "; insidePlace=""; }
                        else if ( lastInteSpeech == "coat_hanger") { placeLoc = "coat_hanger_2"; place=" coat hanger "; roomPlace=" office "; insidePlace="near to the shoe rack"; }
                        else if ( lastInteSpeech == "desk") { placeLoc = "desk"; place=" desk "; roomPlace=" office "; insidePlace="near to the door"; }
                        else if ( lastInteSpeech == "office") { placeLoc = "office "; place=" office "; roomPlace=" the first room at the entrance "; insidePlace=""; }
                        
                        else if ( lastInteSpeech =="bed") { placeLoc = "bed"; place=" bed "; roomPlace=" bedroom "; insidePlace="in front of shelf";  }
                        else if ( lastInteSpeech =="bedroom_chest") { placeLoc = "bedroom_chest"; place=" bedroom chest "; roomPlace=" bedroom "; insidePlace=" in a the corner"; }
                        else if ( lastInteSpeech =="sidetable") { placeLoc = "sidetable"; place=" side table "; roomPlace=" bedroom ";insidePlace="next to the shelf "; }
                        else if ( lastInteSpeech =="shelf") { placeLoc = "shelf"; place=" shelf "; roomPlace=" bedroom "; insidePlace="next to the side table"; }
                        else if ( lastInteSpeech =="bedroom") { placeLoc = "bedroom "; place=" bedroom " ; roomPlace=" room between the kitchen and the office ";insidePlace="";}

                        else if ( lastInteSpeech == "sink") { placeLoc = "sink"; place=" sink "; roomPlace=" kitchen "; insidePlace="next to the dish washer"; }
                        else if ( lastInteSpeech == "dishwasher") { placeLoc = "dishwasher"; place=" dishwasher "; roomPlace=" kitchen "; insidePlace="between the sink and the fridge";}
                        else if ( lastInteSpeech == "fridge") { placeLoc = "fridge"; place=" fridge "; roomPlace=" kitchen "; insidePlace="in the corner next to the dish washer"; }
                        else if ( lastInteSpeech == "kitchen_table") { placeLoc = "kitchen_table"; place=" kitchen table "; roomPlace=" kitchen "; insidePlace="in the center of the room"; }
                        else if ( lastInteSpeech == "kitchen_cabinet") { placeLoc = "kitchen_cabinet"; place=" kitchen cabinet "; roomPlace=" kitchen "; insidePlace="in the corner near to the exit door"; }
                        else if ( lastInteSpeech == "island") { placeLoc = "island"; place=" island "; roomPlace=" kitchen "; insidePlace=" nex to the exit door"; }
                        else if ( lastInteSpeech == "kitchen") { placeLoc = "kitchen"; place=" kitchen "; roomPlace=" room between the living room and the bedroom "; insidePlace=""; }
                        
                        else if ( lastInteSpeech == "exit") { placeLoc = "exit"; place=" exit "; roomPlace=" kitchen "; insidePlace=""; }
                        else if ( lastInteSpeech == "entrance") { placeLoc = "entrance"; place=" entrance "; roomPlace=" office "; insidePlace=""; }
                        
                        
                        
                    }
                    else{}
                }
                else{}

                ss.str("");
                ss << "Do you want information for the" << place << ", please tell me robot yes or robot no.";     

                JustinaHRI::enableSpeechRecognized(false);
                JustinaHRI::loadGrammarSpeechRecognized(GRAMMAR_COMMANDS);
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                JustinaHRI::waitAfterSay(ss.str(), 10000, MAX_DELAY_AFTER_SAY);
                
                JustinaHRI::enableSpeechRecognized(true);

                if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, TIMEOUT_SPEECH))
                {
                    if(lastRecoSpeech.find("yes") != std::string::npos || placeAttemps >= 2)
                    {
                        JustinaHRI::enableSpeechRecognized(false);
                        ss.str("");
                        ss << "Ok ";
                        JustinaHRI::waitAfterSay(ss.str(), 6000, MAX_DELAY_AFTER_SAY);  
                        state = SM_INTERMEDIATE;    
                        placeAttemps=0;      
                    }
                    else
                    {
                        std::cout << "DOSSS" << std::endl;
                        JustinaHRI::waitAfterSay(" I did not understand you.", 3000, MAX_DELAY_AFTER_SAY);
                        placeAttemps++;
                    }
                }else{placeAttemps++;}
                
            break;


            case SM_INTERMEDIATE:

                ss.str("");
                ss << "(assert (where_is_this_place " <<  placeLoc << " 1))";
                
                JustinaRepresentation::strQueryKDB(ss.str(),query ,1000);
                std::cout << "-----------the " << place << "is in " << query <<std::endl;

                std::cout << ".-> SM_TALK_TO_OPERATOR" << std::endl;

                ss.str("");
                ss << "The " << place << "is in the" << roomPlace << " " << insidePlace  <<std::endl;
                JustinaHRI::say(ss.str());
                ros::Duration(4.0).sleep();


                std::cout << "Im here ---> " << whereIm << std::endl;

                if( whereIm == OFFICE && roomPlace == " office ")
                {}
                if( whereIm == OFFICE && roomPlace == " bedroom ")
                {
                    if(!gui)
                        JustinaTasks::guideAPerson("office_", 90000, 1.75);
                    else
                        JustinaNavigation::getClose("office_", 80000);

                    if(!pointingInOffice(officeCounter++))
                        officeCounter=0;
                }
                if( whereIm == OFFICE && roomPlace == " living room ")
                {
                    if(!gui)
                        JustinaTasks::guideAPerson("office_", 90000, 1.75);
                    else
                        JustinaNavigation::getClose("office_", 80000);

                    if(!pointingInOffice(officeCounter++))
                        officeCounter=0;
                }
                if( whereIm == OFFICE && roomPlace == " kitchen ")
                {
                    
                    if(!gui) 
                        JustinaTasks::guideAPerson("office_", 90000, 1.75); 
                    else
                        JustinaNavigation::getClose("office_", 80000);

                    if(!pointingInOffice(officeCounter++))
                        officeCounter=0;

                    if(!gui) 
                        JustinaTasks::guideAPerson("living_room", 90000, 1.75); 
                    else
                        JustinaNavigation::getClose("living_room", 80000);

                    
                    if(!pointingInLivingRoom(livingRoomCounter++))
                        livingRoomCounter=0;

                }


                if( whereIm == BEDROOM && roomPlace == " office ")
                {
                    if(!gui)
                        JustinaTasks::guideAPerson("bedroom", 90000, 1.75);
                    else
                        JustinaNavigation::getClose("bedroom", 80000);

                    if(!pointingInBedRoom(bedRoomCounter++))
                        bedRoomCounter=0;
                }
                if( whereIm == BEDROOM && roomPlace == " bedroom ")
                {}
                if( whereIm == BEDROOM && roomPlace == " living room ")
                {
                    if(!gui)
                        JustinaTasks::guideAPerson("bedroom", 90000, 1.75);
                    else
                        JustinaNavigation::getClose("bedroom", 80000);

                    if(!pointingInBedRoom(bedRoomCounter++))
                        bedRoomCounter=0;

                    if(!gui)
                        JustinaTasks::guideAPerson("office_", 90000, 1.75);
                    else
                        JustinaNavigation::getClose("office_", 80000);

                    if(!pointingInOffice(officeCounter++))
                        officeCounter=0;
                    
                }
                if( whereIm == BEDROOM && roomPlace == " kitchen ")
                {
                    if(!gui)
                        JustinaTasks::guideAPerson("office_", 90000, 1.75);
                    else
                        JustinaNavigation::getClose("office_", 80000);

                    if(!pointingInOffice(officeCounter++))
                        officeCounter=0;


                    if(!gui) 
                        JustinaTasks::guideAPerson("living_room", 90000, 1.75); 
                    else
                        JustinaNavigation::getClose("living_room", 80000); 

                    if(!pointingInLivingRoom(livingRoomCounter++))
                        livingRoomCounter=0;

                }

                if( whereIm == LIVINGROOM && roomPlace == " office ")
                {
                    if(!gui)
                        JustinaTasks::guideAPerson("living_room", 90000, 1.75);
                    else
                        JustinaNavigation::getClose("living_room", 80000);

                    if(!pointingInLivingRoom(livingRoomCounter++))
                        livingRoomCounter=0;
                }
                if( whereIm == LIVINGROOM && roomPlace == " bedroom ")
                {
                    if(!gui)
                        JustinaTasks::guideAPerson("office_", 90000, 1.75);
                    else
                        JustinaNavigation::getClose("office_", 80000);
                    
                    if(!pointingInOffice(officeCounter++))
                        officeCounter=0;

                }
                if( whereIm == LIVINGROOM && roomPlace == " living room ")
                {}
                if( whereIm == LIVINGROOM && roomPlace == " kitchen ")
                {
                    if(!gui)
                        JustinaTasks::guideAPerson("living_room_", 90000, 1.75);
                    else
                        JustinaNavigation::getClose("living_room_", 80000);

                    if(!pointingInLivingRoom(livingRoomCounter++))
                        livingRoomCounter=0;  
                }

                if( whereIm == KITCHEN && roomPlace == " office ")
                {
                    
                    if(!gui) 
                        JustinaTasks::guideAPerson("living_room", 90000, 1.75); 
                    else
                        JustinaNavigation::getClose("living_room", 80000); 
                    
                    if(!pointingInLivingRoom(livingRoomCounter++))
                        livingRoomCounter=0;

                }
                if( whereIm == KITCHEN && roomPlace == " bedroom ")
                {
                    if(!gui) 
                        JustinaTasks::guideAPerson("living_room", 90000, 1.75); 
                    else
                        JustinaNavigation::getClose("living_room", 80000);
                    
                    if(!pointingInLivingRoom(livingRoomCounter++))
                        livingRoomCounter=0;

                    if(!gui)
                        JustinaTasks::guideAPerson("office_", 90000, 1.75);
                    else
                        JustinaNavigation::getClose("office_", 80000);
                    if(!pointingInOffice(officeCounter++))
                        officeCounter=0;



                }
                if( whereIm == KITCHEN && roomPlace == " living room ")
                {
                    if(!gui)
                        JustinaTasks::guideAPerson("kitchen", 90000, 1.75);
                    else
                        JustinaNavigation::getClose("kitchen", 80000);

                    if(!pointingInKitchen(kitchenCounter++))
                        kitchenCounter=0;
                }
                if( whereIm == KITCHEN && roomPlace == " kitchen ")
                {
                   
                }

                state = SM_GO_TO_PLACE;
            break;

            case SM_GO_TO_PLACE:

                

                if(!gui)
                        JustinaTasks::guideAPerson(placeLoc, 90000, 1.75);
                else
                        JustinaNavigation::getClose(placeLoc, 80000);

                state = SM_EXPLAIN;

            break;
    
            
            case SM_EXPLAIN:
                
                JustinaHRI::waitAfterSay(" Whe have reached to the location", 4000, MIN_DELAY_AFTER_SAY);
                std::cout << "Whe have reached to the location" << std::endl;
                ss.str("");
                ss << "We are in the " << roomPlace << " and here is the " << place ;
                
                JustinaManip::startRaGoTo("navigation");
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                here();
                JustinaHRI::waitAfterSay(ss.str(), 4000, MIN_DELAY_AFTER_SAY);
                std::cout << ss.str() << std::endl;
                JustinaManip::startRaGoTo("navigation");
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                
                state = SM_RETURN;             
            break;

            case SM_RETURN:
                JustinaHRI::waitAfterSay("I am going back to the start point", 4000, MIN_DELAY_AFTER_SAY);
                std::cout << "I am going back to the start point" << std::endl;
                if(!JustinaNavigation::getClose(startLoc, 80000) )
                    JustinaNavigation::getClose(startLoc, 80000); 
                
                JustinaHRI::waitAfterSay("I have reached to the starting point", 4000, MIN_DELAY_AFTER_SAY);
                std::cout << "I have reached to the starting point" << std::endl;
                state = SM_TALK_TO_OPERATOR;
                
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
