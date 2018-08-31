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
#define SM_GreetingDoctor 50
#define SM_ReceiveMail 60
#define SM_InterrogatePerson 70
#define	SM_FinalState 80
#define SM_GuidingDoctor 90
#define SM_GuideMemorize 100
#define SM_GUIDING_STOP 110
#define SM_WaitDoctor 120
#define SM_FOLLOW_TO_THE_DOOR 130



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
    std::string id = "doctor";
    std::string location;

	JustinaHRI::loadGrammarSpeechRecognized("speechandperson.xml");//load the grammar
	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
    //JustinaRepresentation::initKDB("", true, 20000);

  	//ros::Rate loop(10);

	bool fail = false;
	bool success = false;
    bool hokuyoRear = false;

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
      			std::cout << "Welcoming visitor Test...->start WELCOMING VISITORS test" << std::endl;
        		JustinaManip::startHdGoTo(0.0, 0.0);
        		JustinaHRI::say("I am ready for the welcoming visitors test");
        		ros::Duration(1.0).sleep();
        		JustinaHRI::say("I'm waiting for the door bell");
        		ros::Duration(1.0).sleep();
        		nextState = SM_WaitingDoorBell;
      		break;

            case SM_WaitingDoorBell:
                std::cout << "Welcoming visitor Test...->waiting door bell.." << std::endl;
                JustinaHRI::waitAfterSay("Tell me, justina start, in order to attend the door bell", 12000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                cont_z=0;
                std::cout << "Welcoming visitor Test...-> SM_WAIT_FOR_COMMAND" << std::endl;
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
                std::cout << "Welcoming visitor Test...->navigate to the door.." << std::endl;
                JustinaHRI::say("I've noticed that the door bell is ringing");
        		ros::Duration(1.0).sleep();
                if (!JustinaTasks::sayAndSyncNavigateToLoc("arena", 120000)) {
					std::cout << "Welcoming visitor Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("arena", 120000)) {
						std::cout << "Welcoming visitor Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("arena", 120000)) {
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
                std::cout << "Welcoming visitor Test...->open the door.." << std::endl;
                JustinaHRI::say("I've noticed that there is someone at the door, but i can not open the door");
        		ros::Duration(2.0).sleep();
                JustinaHRI::say("Human, please open the door");
        		ros::Duration(4.0).sleep();
                nextState = SM_RecognizeVisitor;
            break;

            case SM_RecognizeVisitor:
                std::cout << "Welcoming visitor Test...->recognizing visitor.." << std::endl;
                if(id == "doctor")
                {
                    std::cout << "Welcoming visitor Test...->doctor recognized.." << std::endl;
                    nextState = SM_GreetingDoctor;
                }
                else if(id == "postman")
                {
                    std::cout << "Welcoming visitor Test...->postman recognized.." << std::endl;
                    nextState = SM_ReceiveMail;
                }
                else if(id == "unknown")
                {
                    std::cout << "Welcoming visitor Test...->unknown recognized.." << std::endl;
                    nextState = SM_InterrogatePerson;
                }
                else
                    nextState = SM_RecognizeVisitor;
            break;

            case SM_GreetingDoctor:
                std::cout << "Welcoming visitor Test...->greeting doctor.." << std::endl;
                JustinaHRI::say("Hello Doctor Kimble, we are waiting for you");
        	    ros::Duration(1.0).sleep();
                JustinaHRI::say("Please let me guide you to Annies bedroom");
        	    ros::Duration(1.0).sleep();
                nextState = SM_GuidingDoctor;
            break;


            case SM_GuidingDoctor:
                std::cout << "Welcoming visitor Test...->guiding doctor.." << std::endl;
                JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                //location="bed";
                
                //cont_z=0;
                //JustinaHRI::enableLegFinderRear(true);
                //nextState = SM_GuideMemorize;
                JustinaTasks::guideAPerson("bedroom", 50000000);
                JustinaHRI::waitAfterSay("Here is the annies bedroom, i will waiting for you here", 2500);
                nextState = SM_WaitDoctor;


            break;

            case SM_GuideMemorize:
                hokuyoRear = JustinaHRI::rearLegsFound();
                if(hokuyoRear){
                    JustinaHRI::waitAfterSay("Ok, let us go", 2500);
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

                hokuyoRear = JustinaHRI::rearLegsFound();
                std::cout << "hokuyoRear -> " << hokuyoRear << std::endl;

                if(!hokuyoRear)
                    nextState=SM_GUIDING_STOP;

                if(JustinaNavigation::isGlobalGoalReached()){
                    JustinaHRI::waitAfterSay("Here is the annies bedroom, i will waiting for you here", 2500);
                    JustinaHRI::enableLegFinderRear(false);
                    nextState = SM_WaitDoctor;
                }

            break;

            case SM_GUIDING_STOP:
                std::cout << "Welcoming visitor Test...->SM_GUIDING_STOP" << std::endl;

                JustinaHardware::stopRobot();
                JustinaHardware::stopRobot();
                JustinaHardware::stopRobot();
                ros::spinOnce();
                JustinaHRI::waitAfterSay("I lost you", 1500);
                JustinaHRI::enableLegFinderRear(false);
                JustinaHRI::waitAfterSay("Human, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                nextState=SM_GuideMemorize;
            break;

            case SM_WaitDoctor:
                std::cout << "Welcoming visitor Test...->SM_Waiting doctor" << std::endl;
                JustinaHRI::waitAfterSay("Tell me, justina continue, in order to follow you to the exit", 12000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                cont_z=0;
                std::cout << "Welcoming visitor Test...-> SM_WAIT_FOR_COMMAND" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("justina continue", 15000)){
                    nextState = SM_FOLLOW_TO_THE_DOOR;
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

            case SM_FOLLOW_TO_THE_DOOR:
                std::cout << "Welcoming visitor Test...->SM_Follow to the door" << std::endl;
                if(!JustinaTasks::findAndFollowPersonToLoc("arena"))
                    if(!JustinaTasks::findAndFollowPersonToLoc("arena"))
                        if(!JustinaTasks::findAndFollowPersonToLoc("arena"))
                            nextState = SM_FOLLOW_TO_THE_DOOR;
                std::cout << "Welcoming visitor Test...->Follow to the door successfully" << std::endl;

                JustinaHRI::say("Thank you for your visit, see you soon");
        		ros::Duration(2.0).sleep();
                nextState = SM_FinalState;
            break;
			
            case SM_FinalState:
				std::cout <<"Welcoming visitor Test...->finalState reached" << std::endl;
				JustinaHRI::say("I have finished the WELCOMING VISITORS test");
				ros::Duration(2.0).sleep();
				success=true;
			break;

    }
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
