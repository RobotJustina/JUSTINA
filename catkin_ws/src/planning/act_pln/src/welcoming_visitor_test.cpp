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

#define SM_InitialState 0
#define	SM_WaitingDoorBell 10
#define SM_NAVIGATE_TO_THE_DOOR 20
#define SM_OpenTheDoor 30
#define	SM_RecognizeVisitor 40
#define SM_WaitBlindGame 50
#define SM_BlindGame 60
#define SM_BlindGameRepeatQ 70
#define	SM_FinalState 80



int main(int argc, char** argv)
{
	std::cout << "Initializing Speech and Person Recognition Test..." << std::endl;
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
	std::stringstream auxAudio;
	std::string str1;

	JustinaHRI::loadGrammarSpeechRecognized("speechandperson.xml");//load the grammar
	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
	JustinaRepresentation::initKDB("", true, 20000);

  	//ros::Rate loop(10);

	bool fail = false;
	bool success = false;

  	//int nextState = SM_WaitBlindGame;
  	int nextState = 0;
  	bool recog=false;
  	int numQuestion = 1;
  	std::string answer;
	std::stringstream ss;

    int maxDelayAfterSay = 300;
    int cont_z;
	
	int contChances=0;
	str1 = "/home/biorobotica/Script/stop_arecord.sh ";

	//vector para almacenar los rostros encontrados
	//std::vector<vision_msgs::VisionFaceObject> dFaces;

	//load the predifined questions
  	//JustinaKnowledge::getPredQuestions(questionList);

  	//set the KINECT as the input device 
  	JustinaHRI::setInputDevice(JustinaHRI::RODE);

  	//almacena los rostros detectados por el servicio
  	vision_msgs::VisionFaceObjects dFaces;
  	//alamcena los gestos detectados
  	std::vector<vision_msgs::GestureSkeleton> gestures;



  	while(ros::ok() && !fail && !success)
  	{
		ros::Rate loop(10);
  		switch(nextState)
    	{

    		case SM_InitialState:
      			std::cout << "Welcome visitor Test...->start WELCOME VISITORS test" << std::endl;
        		JustinaManip::startHdGoTo(0.0, 0.0);
        		JustinaHRI::say("I am ready for the welcom visitors test");
        		ros::Duration(1.0).sleep();
        		JustinaHRI::say("I'm waiting for the door bell");
        		ros::Duration(1.0).sleep();
        		nextState = SM_WaitingDoorBell;
      		break;

            case SM_WaitingDoorBell:
                std::cout << "Welcome visitor Test...->waiting door bell.." << std::endl;
                JustinaHRI::waitAfterSay("Tell me, justina start, in order to attend the door bell", 12000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                cont_z=0;
                std::cout << "Welcome visitor Test...-> SM_WAIT_FOR_COMMAND" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("justina start", 15000)){
                    nextState = SM_NAVIGATE_TO_THE_DOOR;
                }
                else                    
                    cont_z++;    		

                if(cont_z>3){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("Please repeat the command", 5000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    cont_z=0;
                }
            break; 

            case SM_NAVIGATE_TO_THE_DOOR:
                std::cout << "Welcome visitor Test...->navigate to the door.." << std::endl;
                JustinaHRI::say("I've noticed that the door bell is ringing");
        		ros::Duration(1.0).sleep();
                if (!JustinaTasks::sayAndSyncNavigateToLoc("entrance", 120000)) {
					std::cout << "Welcome visitor Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("entrance", 120000)) {
						std::cout << "Welcome visitor Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("entrance", 120000)) {
							nextState = SM_OpenTheDoor;
						}
					} 
					else{
						nextState = SM_OpenTheDoor;
					}
				} 
				else {
					nextState = SM_OpenTheDoor;
				}
            break;

            case SM_OpenTheDoor:
                std::cout << "Welcome visitor Test...->open the door.." << std::endl;
                JustinaHRI::say("I've noticed that there is someone at the door, but i can not open the door");
        		ros::Duration(2.0).sleep();
                JustinaHRI::say("Human, please open the door");
        		ros::Duration(4.0).sleep();
                nextState = SM_RecognizeVisitor;
            break;

            case SM_RecognizeVisitor:
                std::cout << "Welcome visitor Test...->recognizing visitor.." << std::endl;
                nextState = SM_FinalState;
            break;

			case SM_FinalState:
				std::cout <<"Welcome visitor Test...->finalState reached" << std::endl;
				JustinaHRI::say("I have finished the WELCOME VISITORS test");
				ros::Duration(2.0).sleep();
				success=true;
			break;

    }
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
