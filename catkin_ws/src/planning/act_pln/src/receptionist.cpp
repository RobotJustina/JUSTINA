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

#define MAX_FIND_PERSON_COUNT 6
#define MAX_FIND_PERSON_ATTEMPS 3


enum STATE{
    SM_INIT,
    SM_SAY_OPEN_DOOR,
    SM_WAIT_FOR_OPEN_DOOR,
    SM_WAIT_FOR_PERSON_ENTRANCE,
    SM_INTRO_GUEST,
    SM_FINISH
};

int minDelayAfterSay = 0;
int maxDelayAfterSay = 300;

std::string test("receptionist");

int main(int argc, char **argv){

    ros::init(argc, argv, "receptionist_test");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    bool opened = false;
    bool success = false;
    bool findPerson = false;
    int findPersonCount = 0;
    int findPersonAttemps = 0;
    Eigen::Vector3d centroidPerson;

    STATE state = SM_INIT;
    
    std::vector<vision_msgs::VisionObject> yoloObjects;

    JustinaHRI::setNodeHandle(&nh);
    JustinaNavigation::setNodeHandle(&nh);
    JustinaVision::setNodeHandle(&nh);
    JustinaManip::setNodeHandle(&nh);
    JustinaTasks::setNodeHandle(&nh);
    JustinaKnowledge::setNodeHandle(&nh);
    JustinaRepresentation::setNodeHandle(&nh);
    JustinaTools::setNodeHandle(&nh);

    while(ros::ok() && !success){

        switch(state){
            case SM_INIT:
                std::cout << test << ".-> State SM_INIT: Init the test." << std::endl;
                JustinaHRI::waitAfterSay("I am ready for the receptionist test", 6000, minDelayAfterSay);
                JustinaHRI::loadGrammarSpeechRecognized("receptionist.xml");//load the grammar
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                state = SM_SAY_OPEN_DOOR;
                break;

            case SM_SAY_OPEN_DOOR:
                std::cout << test << ".-> State SM_SAY_OPEN_DOOR: Saying open the door." << std::endl;
                JustinaHRI::waitAfterSay("Human, can you open the door please", 6000, minDelayAfterSay);
                state = SM_WAIT_FOR_OPEN_DOOR;
                break;

            case SM_WAIT_FOR_OPEN_DOOR:
                std::cout << test << ".-> State SM_WAIT_FOR_OPEN_DOOR: Wait for the open the door." << std::endl;
                opened = JustinaNavigation::doorIsOpen(0.9, 2000);
                state = SM_SAY_OPEN_DOOR;
                if(opened){
                    JustinaHRI::waitAfterSay("Hello human, can you entrance in the house please", 6000, minDelayAfterSay);
                    state = SM_WAIT_FOR_PERSON_ENTRANCE;
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                }
                else
                    ros::Duration(3).sleep();
                break;
            case SM_WAIT_FOR_PERSON_ENTRANCE:
                std::cout << test << ".-> State SM_INTRO_GUEST: Intro Guest." << std::endl;
                
                if(findPersonAttemps < MAX_FIND_PERSON_ATTEMPS){
                    findPerson = JustinaTasks::turnAndRecognizeYoloPerson(JustinaTasks::NONE, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0f, 9.0, centroidPerson, "entrance");
                    if(findPerson)
                        findPersonCount++;
                    if(findPersonCount > MAX_FIND_PERSON_COUNT){
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        state = SM_INTRO_GUEST;
                    }
                }else{
                    
                }
                JustinaHRI::waitAfterSay("Hello human, can you entrance in the house please", 6000, minDelayAfterSay);
                state = SM_WAIT_FOR_OPEN_DOOR;
                break;
            case SM_INTRO_GUEST:
                JustinaHRI::waitAfterSay("Hello, my name is Justina", 6000, minDelayAfterSay);
                std::cout << test << ".-> State SM_INTRO_GUEST: Intro Guest." << std::endl;
                break;

            case SM_FINISH:
                std::cout << test << ".-> State SM_FINISH: Finish the test." << std::endl;
                JustinaHRI::waitAfterSay("I have finished the test", 6000, minDelayAfterSay);
                success = true;
                break;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 1;
}
