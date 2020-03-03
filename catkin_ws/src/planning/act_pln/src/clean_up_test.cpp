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


#define SM_SAY_WAIT_FOR_DOOR 0
#define SM_WAIT_FOR_DOOR 10
#define SM_INIT 20
#define SM_FINAL_STATE 100


#define GRAMMAR_POCKET_COMMANDS "grammars/pre_sydney/commands.jsgf"
#define GRAMMAR_POCKET_FOLLOW "/grammars/pre_sydney/gpsr/follow_me.jsgf"
#define GRAMMAR_POCKET_NAMES "grammars/pre_sydney/people_names.jsgf"
#define GRAMMAR_COMMANDS "commands.xml"
#define GRAMMAR_FOLLOW "follow_taxi.xml"
#define GRAMMAR_NAMES "people_names.xml"
#define TIMEOUT_SPEECH 10000
#define MAX_ATTEMPTS_WAIT_CONFIRMATION 2
#define MAX_DELAY_AFTER_SAY 300
#define MAX_ATTEMPTS_SPEECH_INT 3
#define MIN_DELAY_AFTER_SAY 0
#define MAX_ATTEMPTS_SPEECH_RECO 3


void switchSpeechReco(bool pocket, std::string grammarPocket, std::string grammarMicrosoft, std::string speech){
    if (pocket){
        //use pocket sphinx
        JustinaHRI::usePocketSphinx = true;
        JustinaHRI::enableGrammarSpeechRecognized(grammarPocket, 0);
	    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
        JustinaHRI::enableSpeechRecognized(false);
	    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
        JustinaHRI::waitAfterSay(speech,5000);
        JustinaHRI::enableSpeechRecognized(true);
    }

    else{
        //use speech recognition of microsoft
        JustinaHRI::usePocketSphinx = false;
        JustinaHRI::loadGrammarSpeechRecognized(grammarMicrosoft);
        JustinaHRI::waitAfterSay(speech,5000);
        JustinaHRI::enableSpeechRecognized(true);
    }
}

int main(int argc, char** argv)
{
	std::cout << "Initializing Clean Up Test..." << std::endl;
  	ros::init(argc, argv, "act_pln");
  	ros::NodeHandle n;
  	JustinaHardware::setNodeHandle(&n);
  	JustinaHRI::setNodeHandle(&n);
  	JustinaManip::setNodeHandle(&n);
  	JustinaNavigation::setNodeHandle(&n);
  	JustinaTools::setNodeHandle(&n);
  	JustinaVision::setNodeHandle(&n);
	JustinaAudio::setNodeHandle(&n);
	JustinaRepresentation::setNodeHandle(&n);
	JustinaTasks::setNodeHandle(&n);
	JustinaKnowledge::setNodeHandle(&n);//knowledge
    JustinaRepresentation::setNodeHandle(&n);

    std::string grammarCommandsID = "farewellCommands";
    std::string grammarNamesID = "farewellNames";
    std::string grammarFollowID = "farewellFollow";

    JustinaHRI::usePocketSphinx = false;
    
    
	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech


    int nextState = 0;
    bool fail = false;
    bool success = false;

    JustinaHRI::setInputDevice(JustinaHRI::KINECT);

    ros::Rate loop(10);

    while(ros::ok() && !fail && !success)
  	{
        switch(nextState)
    	{
            case SM_SAY_WAIT_FOR_DOOR:
                std::cout << "Clean Up Test...-> start Clean Up test" << std::endl;
                JustinaHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_COMMANDS);
                ros::spinOnce();
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaHRI::loadGrammarSpeechRecognized(grammarFollowID, GRAMMAR_POCKET_FOLLOW);
                ros::spinOnce();
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
        		JustinaManip::startHdGoTo(0.0, 0.0);
        		JustinaHRI::say("I am ready for the Clean Up test");
        		ros::Duration(2.0).sleep();
				JustinaHRI::waitAfterSay("I am waiting for the door to be open", 4000);
				nextState = SM_WAIT_FOR_DOOR;
            break;

            case SM_WAIT_FOR_DOOR:
				if (!JustinaNavigation::obstacleInFront())
					nextState = SM_INIT;
			break;

            case SM_INIT:
                std::cout << "Clean Up Test...->navigate to initial point " << std::endl;
                JustinaHRI::waitAfterSay("Now I can see that the door is open",4000);
				std::cout << "Clean Up Test...->First attempt to move" << std::endl;
            	JustinaNavigation::moveDist(1.0, 4000);

                /*std::cout << "Farewell Test...-> start Farewell test" << std::endl;
                JustinaHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_COMMANDS);
                ros::spinOnce();
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaHRI::loadGrammarSpeechRecognized(grammarFollowID, GRAMMAR_POCKET_FOLLOW);
                ros::spinOnce();
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
        		JustinaManip::startHdGoTo(0.0, 0.0);
        		JustinaHRI::say("I am ready for the Farewell test");
        		ros::Duration(2.0).sleep();*/

                JustinaManip::hdGoTo(0.0, 0.0, 2000);
                if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
					std::cout << "Clean Up Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
						std::cout << "Clean Up Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
							std::cout << "Clean Up...->moving to the initial point" << std::endl;
						}
					} 
				}
                //JustinaHRI::waitAfterSay("I'm ready for the farewell test, tell me, justina start, to performing the test", timeoutspeech, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(false);
                //nextState = SM_SEARCH_WAVING;
                break;

            case SM_FINAL_STATE:
                std::cout << "Clean UP Test...-> SM_FINAL_STATE" << std::endl;
                JustinaManip::hdGoTo(0.0, 0.0, 2000);
                JustinaHRI::say("I have finished the test");
                ros::Duration(1.0).sleep();
                success=true;
            break;

        }

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}