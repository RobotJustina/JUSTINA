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
#include "justina_tools/JustinaIROS.h"
#include "std_msgs/Bool.h"
#include "string"

#define SM_WatingPrepare 0
#define SM_InitialState 1
#define	SM_WaitingDoorBell 10
#define SM_NAVIGATE_TO_THE_DOOR 20
#define SM_OpenTheDoor 30
#define	SM_RecognizeVisitor 40
#define SM_GreetingDoctor 50
#define SM_ReceiveMail 60
#define SM_InterrogatePerson 70
#define	SM_FinalState 80
#define SM_WaitVisitor 90
#define SM_GuidingDoctor 100
#define SM_FOLLOW_TO_THE_DOOR 110
#define SM_GreetingPostman 120
#define SM_NavigateToInicialPoint 130
#define SM_DeliverMailToAnnie 140
#define SM_DeliverPost 150 
#define SM_IdentityConfirm 160
#define SM_GreetingDeliman 170
#define SM_GreetingPlumber 180
#define SM_ConfirmLocation 190
#define SM_GuidingPlumber 200
#define SM_ConfirmLocationD 210
#define SM_GuidingDeliman 220


#define MAX_ATTEMPTS_RECOG 3
#define MAX_ATTEMPTS_CONF 3


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
        lastRecognizedFaces = JustinaVision::getFaces("");
        
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

std::string identifyVisitor(float timeOut, bool &recognized){
    recognized = false;
    int previousSize = 20;
    int sameValue = 0;
    int contDoctor =0;
    int contPostman = 0;
    int contUnknown =0;
    boost::posix_time::ptime curr;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff;
    vision_msgs::VisionFaceObjects lastRecognizedFaces;


    do
    {
        lastRecognizedFaces = JustinaVision::getFacenet2D("");

        for(int i=0; i<lastRecognizedFaces.recog_faces.size(); i++){
        if(lastRecognizedFaces.recog_faces[i].id=="doctor")
            contDoctor++;
        else if(lastRecognizedFaces.recog_faces[i].id=="postman")
            contPostman++;
        else if(lastRecognizedFaces.recog_faces[i].id=="Unknown")
            contUnknown++; 
        }
        
        if(previousSize == 1)
            sameValue ++;
        
        if (sameValue == 4)
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
    std::cout << "cont doctor:" << contDoctor << std::endl;
    std::cout << "cont postman:" << contPostman << std::endl;
    std::cout << "cont Unknown:" << contUnknown << std::endl;
    
    

    if(contDoctor >= contPostman && contDoctor >= contUnknown)
        return "doctor";
    else if(contPostman >= contDoctor && contPostman >= contUnknown)
        return "postman";
    else if(contUnknown >= contDoctor && contUnknown >= contPostman)
        return "Unknown";
}



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
    JustinaIROS::setNodeHandle(&n);
	std::stringstream auxAudio;
	std::string str1;
    std::string id = "doctor";
    std::string location;

	JustinaHRI::loadGrammarSpeechRecognized("welcoming_visitors.xml");//load the grammar
	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
    //JustinaRepresentation::initKDB("", true, 20000);

  	//ros::Rate loop(10);

	bool followV = false;
    bool fail = false;
	bool success = false;
    bool hokuyoRear = false;


  	//int nextState = SM_WaitBlindGame;
  	int nextState = 0;
  	bool recog=false;
    bool recogMail = false;  
    bool withLeftArm=false;  
    bool validatePlumber = false;
    bool isPlumber = false;
    bool userConfirmation = false;
    bool opened = false;

  	int numQuestion = 1;
  	std::string answer;
	std::stringstream ss;
    std::string lastRecoSpeech;
    std::string fileDirectory;
    int timeoutspeech = 10000;

    int attempsSpeechReco = 1;
    int attempsSpeechInt = 1;
    int attempsConfirmation = 1;
    int attempsWaitConfirmation = 1;
    int maxAttempsConfirmation = 3;
    int maxAttempsWaitConfirmation = 3;

    int maxDelayAfterSay = 300;
    int cont_z;
    int contO=0;
    int attemptsRecogLoc = 0;
    int attemptsConfLoc = 0;
    int contU = 0;
    int contK = 0;
    int contVisitor = 0;
    bool door =true;
    int contD =0;
    int contUP = 0;

	
	int contChances=0;
    int contAttempts = 0;
	str1 = "/home/biorobotica/Script/stop_arecord.sh";
    std::string grammarPlumber = "welcome_plumber.xml";
    std::string grammarDeliman = "welcome_deliman.xml";

    std::vector<std::string> confirmCommands;
    confirmCommands.push_back("justina yes");
    confirmCommands.push_back("justina no");

    std::vector<std::string> validCommandsVisit;
    validCommandsVisit.push_back("i want to visit the kitchen");
    validCommandsVisit.push_back("i want to visit the bedroom");
    validCommandsVisit.push_back("i want to visit the bathroom");

    std::vector<std::string>plumberNotAllowed;
    plumberNotAllowed.push_back("bedroom");
    std::vector<std::string>delimanNotAllowed;
    delimanNotAllowed.push_back("bathroom");
    delimanNotAllowed.push_back("bedroom");
    

	//vector para almacenar los rostros encontrados
	//std::vector<vision_msgs::VisionFaceObject> dFaces;

	//load the predifined questions
  	//JustinaKnowledge::getPredQuestions(questionList);

  	//set the KINECT as the input device 
  	JustinaHRI::setInputDevice(JustinaHRI::KINECT);

  	//almacena los rostros detectados por el servicio
  	vision_msgs::VisionFaceObjects faces;
  	//alamcena los gestos detectados
  	std::vector<vision_msgs::GestureSkeleton> gestures;
    
    ros::Time timeTest = ros::Time::now();
    JustinaTools::startGlobalRecordRosbag("ERL Consumer", "Welcoming Visitors", timeTest);
    JustinaTools::startTestRecordRosbag("ERL Consumer","Welcoming Visitors", timeTest);
    ros::Rate loop(10);

    ros::Publisher pubstartExecuting = n.advertise<std_msgs::Empty>("/planning/start_executing", 1);

  	while(ros::ok() && !fail && !success)
  	{
        if(JustinaTasks::tasksStop())
        {
            JustinaTasks::sayAndSyncNavigateToLoc("exitdoor", 240000, true);
            break;
        }
  		switch(nextState)
    	{
            case SM_WatingPrepare:
      			std::cout << "Welcoming visitor Test...->wating WELCOMING VISITORS test" << std::endl;
                if(JustinaIROS::getLastBenchmarkState() == roah_rsbb_comm_ros::BenchmarkState::PREPARE)
                {
                    JustinaIROS::end_prepare();
                    pubstartExecuting.publish(std_msgs::Empty());
                    nextState = SM_InitialState;
                }
            break;
    		case SM_InitialState:
      			std::cout << "Welcoming visitor Test...->start WELCOMING VISITORS test" << std::endl;
                if(JustinaIROS::getLastBenchmarkState() == roah_rsbb_comm_ros::BenchmarkState::EXECUTE)
                {
                    JustinaManip::startHdGoTo(0.0, 0.0);
                    JustinaHRI::say("I am ready for the welcoming visitors test");
                    ros::Duration(1.0).sleep();
                    //JustinaNavigation::moveDist(1.0, 4000);
                    //JustinaHRI::say("I'm waiting for the door bell");
                    //ros::Duration(1.0).sleep();
                    nextState = SM_WaitingDoorBell;
                }
      		break;

            case SM_WaitingDoorBell:
                if(contVisitor<4)
                {
                    /*
                    JustinaHRI::enableSpeechRecognized(false);//enable recognized speech
                    std::cout << "Welcoming visitor Test...->waiting door bell.." << std::endl;
                    JustinaHRI::waitAfterSay("Tell me, justina start, in order to attend the door bell", 12000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    cont_z=0;
                    std::cout << "Welcoming visitor Test...-> SM_WAIT_FOR_COMMAND" << std::endl;
                    if(JustinaHRI::waitForSpecificSentence("justina start", 15000)){
                        contVisitor++;
                        nextState = SM_NAVIGATE_TO_THE_DOOR;
                    }
                    else                    
                        cont_z++;    		

                    if(cont_z>3){
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("Please repeat the command", 5000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        cont_z=0;
                    }*/
                    std::cout << "Welcoming visitor Test...->waiting door bell.." << std::endl;
                    if(JustinaIROS::getBellState() != -1){
                        contVisitor++;
                        JustinaIROS::loggingCommand("door bell ringing");
                        nextState = SM_NAVIGATE_TO_THE_DOOR;
                    }
                }
                else
                    nextState = SM_FinalState;
            break; 

            case SM_NAVIGATE_TO_THE_DOOR:
                std::cout << "Welcoming visitor Test...->navigate to the door.." << std::endl;
                JustinaHRI::say("I've noticed that the door bell is ringing");
        		ros::Duration(1.0).sleep();
                if (!JustinaTasks::sayAndSyncNavigateToLoc("entrance_door", 120000)) {
					std::cout << "Welcoming visitor Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("entrance_door", 120000)) {
						std::cout << "Welcoming visitor Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("entrance_door", 120000)) {
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
        		ros::Duration(1.0).sleep();
                while(!opened || contO ==3){
                    //opened = JustinaTasks::visitorOpenDoor(500000);
                    opened = JustinaNavigation::doorIsOpen(0.9, 2000);
                    contO++;
                    std::cout << "open: " << opened << std::endl;
                }
                if(opened || contChances ==3){
                    opened = false;
                    contO = 0;
                    nextState = SM_RecognizeVisitor;
                    contChances =0;
                    JustinaIROS::loggingCommand("the door has been opened");
                    JustinaHRI::say("the door is opened");
        		    ros::Duration(1.0).sleep();
                    JustinaNavigation::moveDistAngle(0.45, 0.0, 10000);
                    ros::Duration(1.0).sleep();
                }
                else{
                    nextState = SM_OpenTheDoor;
                    contChances++;
                }
            break;

            case SM_RecognizeVisitor:
                JustinaHRI::say("please, look at me to try to identify you");
        	    ros::Duration(1.0).sleep();

                if(contK <2)
                    id = identifyVisitor(10000, recog);
                else
                    id = "Unknown";

                std::cout << "Welcoming visitor Test...->recognizing visitor.." << std::endl;
                if(id == "doctor")
                {
                    std::cout << "Welcoming visitor Test...->doctor recognized.." << std::endl;
                    /*fileDirectory = JustinaTools::startRecordSpeach("ERL Consumer", "Welcoming Visitors");
                    JustinaHRI::say("I think you are the doctor kimble");
        	        ros::Duration(1.0).sleep();
                    JustinaHRI::say("please tell me justina yes or justina no to confirm your identity");
        	        ros::Duration(1.0).sleep();
                    JustinaHRI::enableSpeechRecognized(true);
                    JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, timeoutspeech);
                    if(lastRecoSpeech.find("yes") != std::string::npos || attempsConfirmation == 3){
                        attempsSpeechReco = 1;
                        attempsSpeechInt = 1;
                        attempsConfirmation = 1;
                        attempsWaitConfirmation = 1;
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaTools::stopRecordSpeach();
                        std::cout << "audio saved in: " << fileDirectory << std::endl;
                        contK++;
                        nextState = SM_GreetingDoctor;
                    }
                    else{
                         JustinaHRI::enableSpeechRecognized(false);
                         JustinaTools::stopRecordSpeach();
                         std::cout << "audio saved in: " << fileDirectory << std::endl;
                         attempsWaitConfirmation++;
                         nextState = SM_RecognizeVisitor;
                    }*/
                    nextState = SM_GreetingDoctor;
                }
                else if(id == "postman")
                {
                    std::cout << "Welcoming visitor Test...->postman recognized.." << std::endl;
                    /*fileDirectory = JustinaTools::startRecordSpeach("ERL Consumer", "Welcoming Visitors");
                    JustinaHRI::say("I think you are the postman");
        	        ros::Duration(1.0).sleep();
                    JustinaHRI::say("please tell me justina yes or justina no to confirm your identity");
        	        ros::Duration(1.0).sleep();
                    JustinaHRI::enableSpeechRecognized(true);
                    JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, timeoutspeech);
                    if(lastRecoSpeech.find("yes") != std::string::npos || attempsConfirmation == 3){
                        attempsSpeechReco = 1;
                        attempsSpeechInt = 1;
                        attempsConfirmation = 1;
                        attempsWaitConfirmation = 1;
                        JustinaHRI::enableSpeechRecognized(false);
                        JustinaTools::stopRecordSpeach();
                        std::cout << "audio saved in: " << fileDirectory << std::endl;
                        contK++;
                        nextState = SM_GreetingPostman;
                    }
                    else{
                         JustinaHRI::enableSpeechRecognized(false);
                         JustinaTools::stopRecordSpeach();
                         std::cout << "audio saved in: " << fileDirectory << std::endl;
                         attempsWaitConfirmation++;
                         nextState = SM_RecognizeVisitor;
                    }*/
                    nextState = SM_GreetingPostman;
                }
                else if(id == "Unknown")
                {
                    std::cout << "Welcoming visitor Test...->unknown recognized.." << std::endl;
                    nextState = SM_InterrogatePerson;
                }
                else
                    nextState = SM_RecognizeVisitor;
            break;

            case SM_InterrogatePerson:
                std::cout << "Welcoming visitor Test...->interrogate person.." << std::endl;
                if(contUP == 2){
                    JustinaHRI::say("Sorry, but i can not recognize you, i will try it again");
        	        ros::Duration(1.0).sleep();
                    contUP = 0;
                    nextState = SM_RecognizeVisitor;   
                }
                else{
                    fileDirectory = JustinaTools::startRecordSpeach("ERL Consumer", "Welcoming Visitors");
                    JustinaHRI::say("Sorry, but i can not recognize you");
        	        ros::Duration(1.0).sleep();
                    JustinaHRI::say("I need to ask you some questions to try to identify you");
        	        ros::Duration(1.0).sleep();
                    JustinaHRI::enableSpeechRecognized(false);
                    if(validatePlumber){
                        JustinaHRI::waitAfterSay("Do you want to repair something in the house, please tell me justina yes or justina no", 12000, maxDelayAfterSay);
                        contUP++;
                    }
                    else{ 
                        JustinaHRI::waitAfterSay("Do you want to deliver something, please tell me justina yes or justina no", 11000, maxDelayAfterSay);
                        contUP++;
                    }
                    JustinaHRI::enableSpeechRecognized(true);
                    nextState = SM_IdentityConfirm;
                }
                
            break;

     
            case SM_IdentityConfirm:
                std::cout << "State machine: Confirm Identity" << std::endl;
                if(JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, timeoutspeech)){
                    if(lastRecoSpeech.find("yes") != std::string::npos){
                        attempsSpeechReco = 1;
                        attempsSpeechInt = 1;
                        attempsConfirmation = 1;
                        attempsWaitConfirmation = 1;
                        isPlumber = validatePlumber;
                        JustinaHRI::enableSpeechRecognized(false);
                        if(isPlumber){
                            std::cout << "Welcoming visitor Test...->plumber recognized.." << std::endl;
                            /*JustinaHRI::say("I think you are the plumber");
        	                ros::Duration(1.0).sleep();
                            JustinaHRI::say("please tell me justina yes or justina no to confirm your identity");
        	                ros::Duration(1.0).sleep();
                            JustinaHRI::enableSpeechRecognized(true);
                            JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, timeoutspeech);
                            if(lastRecoSpeech.find("yes") != std::string::npos || attempsConfirmation == 3){
                                attempsSpeechReco = 1;
                                attempsSpeechInt = 1;
                                attempsConfirmation = 1;
                                attempsWaitConfirmation = 1;
                                JustinaHRI::enableSpeechRecognized(false);*/
                                JustinaHRI::say("Hello Plumber, my name is Justina");
        	                    ros::Duration(1.0).sleep();
                                JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want to visit the kitchen", 5000, maxDelayAfterSay);
                                id = "plumber";
                                JustinaIROS::loggingVisitor(id);
                                nextState = SM_GreetingPlumber;
                            /*}
                            else{
                                 JustinaHRI::enableSpeechRecognized(false);
                                 JustinaTools::stopRecordSpeach();
                                 std::cout<< "audio saved in: " << fileDirectory << std::endl;
                                 attempsWaitConfirmation++;
                                 nextState = SM_InterrogatePerson;
                            }*/
                            
                        }
                        else{
                            std::cout << "Welcoming visitor Test...->deliman recognized.." << std::endl;
                            /*JustinaHRI::say("I think you are the deliman");
        	                ros::Duration(1.0).sleep();
                            JustinaHRI::say("please tell me justina yes or justina no to confirm your identity");
        	                ros::Duration(1.0).sleep();
                            JustinaHRI::enableSpeechRecognized(true);
                            JustinaHRI::waitForSpecificSentence(confirmCommands, lastRecoSpeech, timeoutspeech);
                            if(lastRecoSpeech.find("yes") != std::string::npos || attempsConfirmation == 3){
                                attempsSpeechReco = 1;
                                attempsSpeechInt = 1;
                                attempsConfirmation = 1;
                                attempsWaitConfirmation = 1;
                                JustinaHRI::enableSpeechRecognized(false);*/
                                JustinaHRI::say("Hello deliman, my name is Justina");
        	                    ros::Duration(1.0).sleep();
                                //JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want to visit the kitchen", 5000, maxDelayAfterSay);
                                id = "deliman";
                                JustinaIROS::loggingVisitor(id);
                                nextState = SM_GreetingDeliman;
                            /*}
                            else{
                                 JustinaHRI::enableSpeechRecognized(false);
                                 JustinaTools::stopRecordSpeach();
                                 std::cout<< "audio saved in: " << fileDirectory << std::endl;
                                 attempsWaitConfirmation++;
                                 nextState = SM_InterrogatePerson;
                            }*/
                        }
                        JustinaHRI::enableSpeechRecognized(true);
                        //nextState = SM_TAKE_ORDER;
                    }
                    else{
                        if(attempsConfirmation <= maxAttempsConfirmation){
                            attempsConfirmation++;
                            attempsWaitConfirmation = 1;
                            nextState = SM_InterrogatePerson;
                        }
                        else{
                            attempsSpeechReco = 1;
                            attempsSpeechInt = 1;
                            attempsConfirmation = 1;
                            attempsWaitConfirmation = 1;
                            isPlumber = validatePlumber;
                            JustinaHRI::enableSpeechRecognized(false);
                            if(isPlumber){
                                JustinaHRI::say("Hello Plumber, my name is Justina");
        	                    ros::Duration(1.0).sleep();
                                JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want  to visit the kitchen", 5000, maxDelayAfterSay);
                                id = "plumber";
                                JustinaIROS::loggingVisitor(id);
                                nextState = SM_GreetingPlumber;
                            }
                            else{
                                JustinaHRI::say("Hello Deli man, my name is Justina");
        	                    ros::Duration(1.0).sleep();
                                //JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want to visit the kitchen", 5000, maxDelayAfterSay);
                                id = "deliman";
                                JustinaIROS::loggingVisitor(id);
                                nextState = SM_GreetingDeliman;
                            }
                            JustinaHRI::enableSpeechRecognized(true);
                            
                        }
                        if(validatePlumber)
                            validatePlumber = false;
                        else
                            validatePlumber = true;
                    }
                }
                else {
                    if(attempsWaitConfirmation <= maxAttempsWaitConfirmation){
                        attempsWaitConfirmation++;
                        nextState = SM_InterrogatePerson;
                    }
                    else{
                        attempsSpeechReco = 1;
                        attempsSpeechInt = 1;
                        attempsConfirmation = 1;
                        attempsWaitConfirmation = 1;
                        isPlumber = validatePlumber;
                        JustinaHRI::enableSpeechRecognized(false);
                        if(isPlumber){
                            JustinaHRI::say("Hello Plumber, my name is Justina");
        	                ros::Duration(1.0).sleep();
                            id = "plumber";
                            cont_z=8;
                            JustinaIROS::loggingVisitor(id);
                            nextState = SM_GreetingPlumber;
                        }
                        else{
                            JustinaHRI::say("Hello Deli man, my name is Justina");
                            ros::Duration(1.0).sleep();
                            //JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want to visit the kitchen", 5000, maxDelayAfterSay);
                            id = "deliman";
                            JustinaIROS::loggingVisitor(id);
                            nextState = SM_GreetingDeliman;
                        }
                        
                    }
                }
            break;

            case SM_GreetingDeliman:
                std::cout << "Welcoming visitor Test...->greeting deliman.." << std::endl;
                JustinaHRI::say("I am going to guide you to the kitchen");
                ros::Duration(1.0).sleep();
                JustinaTools::stopRecordSpeach();
                std::cout<< "audio saved in: " << fileDirectory << std::endl;
                JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                ros::Duration(1.0).sleep();
                location = "kitchen";
                nextState = SM_GuidingDeliman;
                /*if(cont_z > 3){
                    JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want  to visit the kitchen", 5000, maxDelayAfterSay);
                    //JustinaHRI::loadGrammarSpeechRecognized(grammarPlumber);
                    JustinaHRI::enableSpeechRecognized(true);
                    cont_z=0;
                }
                cont_z++;
                if(JustinaHRI::waitForSpecificSentence(validCommandsVisit, lastRecoSpeech, 7000)){
                    attemptsRecogLoc++;
 
                    if(lastRecoSpeech.find("the bedroom") != std::string::npos)
                        location="bedroom";
                    else if (lastRecoSpeech.find("the kitchen") != std::string::npos)
                        location = "kitchen";
                    else if (lastRecoSpeech.find("the bathroom") != std::string::npos)
                        location = "bathroom";
                     else if(attemptsRecogLoc >= MAX_ATTEMPTS_RECOG){
                        location = "kitchen";
                    } 

                    std::cout << "Welcoming visitor Test...->verify if the person is allowed to enter to the room" << std::endl;
                    
                    if(location=="bedroom"){
                        JustinaHRI::say("Sorry deli man but you are not allowed to visit the bedroom");
                        ros::Duration(1.5).sleep();
                        cont_z = 8;
                        nextState = SM_GreetingDeliman;
                        break; 
                    }

                    else if(location=="kitchen"){
                        JustinaHRI::say("do you want to go to the kitchen");
                        ros::Duration(1.0).sleep();
                        JustinaHRI::say("tell me justina yes or justina no");
                        ros::Duration(1.0).sleep();
                        nextState = SM_ConfirmLocationD;
                        JustinaHRI::enableSpeechRecognized(true);
                    }
                    
                    else if (location=="bathroom"){
                        JustinaHRI::say("Sorry deli man but you are not allowed to visit the bathroom");
                        ros::Duration(1.0).sleep();
                        nextState = SM_GreetingDeliman;
                        JustinaHRI::enableSpeechRecognized(true);
                    }

                } */ 
            break;

            case SM_ConfirmLocationD:
                std::cout << "Welcoming visitor Test...->confirm location" << std::endl;
                //boost::this_thread::sleep(boost::posix_time::milliseconds(500));   
                JustinaHRI::waitForUserConfirmation(userConfirmation, 7000);
                attemptsConfLoc++;
                if(userConfirmation){
                    JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                    ros::Duration(1.0).sleep();
                    nextState = SM_GuidingDeliman;
                }
                else if(attemptsConfLoc < MAX_ATTEMPTS_CONF){
                    nextState = SM_GreetingDeliman;
                    cont_z = 8;
                }
                else{
                    JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                    ros::Duration(1.0).sleep();
                    nextState = SM_GuidingDeliman;
                }
            break;

            case SM_GuidingDeliman:
                std::cout << "Welcoming visitor Test...->go to location" << std::endl;
                std::cout << "Welcoming visitor Test...->guiding deli man.." << std::endl;
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                
                //JustinaTasks::guideAPerson(location, 50000000);
                JustinaTasks::guideAPerson("kitchen", 50000000, 1.5, true, delimanNotAllowed);
                //JustinaNavigation::moveDistAngle(0.0, 3.14159, 10000);
                //ros::Duration(1.0).sleep();
                JustinaHRI::say("Please deliver the breakfast box on the table");
        		ros::Duration(2.0).sleep();
                JustinaHRI::waitAfterSay("i will waiting for you here", 2500);
                nextState = SM_WaitVisitor;
            break;

            case SM_GreetingPlumber:
                std::cout << "Welcoming visitor Test...->greeting plumber.." << std::endl;
                if(cont_z > 3){
                    JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want  to visit the kitchen", 5000, maxDelayAfterSay);
                    //JustinaHRI::loadGrammarSpeechRecognized(grammarPlumber);
                    JustinaHRI::enableSpeechRecognized(true);
                    cont_z=0;
                }
                cont_z++;
                if(JustinaHRI::waitForSpecificSentence(validCommandsVisit, lastRecoSpeech, 7000)){
                    attemptsRecogLoc++;
 
                    if(lastRecoSpeech.find("the bedroom") != std::string::npos)
                        location="bedroom";
                    else if (lastRecoSpeech.find("the kitchen") != std::string::npos)
                        location = "kitchen";
                    else if (lastRecoSpeech.find("the bathroom") != std::string::npos)
                        location = "bathroom";
                     else if(attemptsRecogLoc >= MAX_ATTEMPTS_RECOG){
                        location = "bathroom";
                    } 

                    std::cout << "Welcoming visitor Test...->verify if the person is allowed to enter to the room" << std::endl;
                    
                    if(location=="bedroom"){
                        JustinaHRI::say("Sorry plumber but you are not allowed to visit the bedroom");
                        ros::Duration(1.5).sleep();
                        JustinaTools::stopRecordSpeach();
                        std::cout<< "audio saved in: " << fileDirectory << std::endl;
                        cont_z = 8;
                        nextState = SM_GreetingPlumber;
                        break; 
                    }

                    else if(location=="kitchen"){
                        JustinaHRI::say("do you want to go to the kitchen");
                        ros::Duration(1.0).sleep();
                        JustinaHRI::say("tell me justina yes or justina no");
                        ros::Duration(1.0).sleep();
                        nextState = SM_ConfirmLocation;
                        JustinaHRI::enableSpeechRecognized(true);
                    }
                    
                    else if (location=="bathroom"){
                        JustinaHRI::say("do you want to go to the bathroom");
                        ros::Duration(1.0).sleep();
                        JustinaHRI::say("tell me justina yes or justina no");
                        ros::Duration(1.0).sleep();
                        nextState = SM_ConfirmLocation;
                        JustinaHRI::enableSpeechRecognized(true);
                    }

                }  
            break;

            case SM_ConfirmLocation:
                std::cout << "Welcoming visitor Test...->confirm location" << std::endl;
                //boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                JustinaHRI::waitForUserConfirmation(userConfirmation, 7000);
                attemptsConfLoc++;
                JustinaTools::stopRecordSpeach();
                std::cout<< "audio saved in: " << fileDirectory << std::endl;
                if(userConfirmation){
                    JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                    ros::Duration(1.0).sleep();
                    nextState = SM_GuidingPlumber;
                }
                else if(attemptsConfLoc < MAX_ATTEMPTS_CONF){
                    nextState = SM_GreetingPlumber;
                    cont_z = 8;
                }
                else{
                    JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                    ros::Duration(1.0).sleep();
                    nextState = SM_GuidingPlumber;
                }
            break;

            case SM_GuidingPlumber:
                std::cout << "Welcoming visitor Test...->go to location" << std::endl;
                std::cout << "Welcoming visitor Test...->guiding plumber.." << std::endl;
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                
                //JustinaTasks::guideAPerson(location, 50000000);
                JustinaTasks::guideAPerson(location, 50000000, 1.5, true, plumberNotAllowed);
                //JustinaNavigation::moveDistAngle(0.0, 3.14159, 10000);
                //ros::Duration(1.0).sleep();
                if(followV){
                    JustinaTasks::followVisitor();
                    nextState = SM_FOLLOW_TO_THE_DOOR;
                }
                else{
                    JustinaHRI::waitAfterSay("i will waiting for you here", 2500);
                    nextState = SM_WaitVisitor;
                }

            break;

            case SM_GreetingDoctor:
                std::cout << "Welcoming visitor Test...->greeting doctor.." << std::endl;
                JustinaIROS::loggingVisitor("doctor");
                JustinaHRI::say("Hello Doctor Kimble, we are waiting for you");
        	    ros::Duration(1.0).sleep();
                JustinaHRI::say("Please let me guide you to Annies bedroom");
        	    ros::Duration(1.0).sleep();
                JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                ros::Duration(1.0).sleep();
                nextState = SM_GuidingDoctor;
            break;

            case SM_GreetingPostman:
                std::cout << "Welcoming visitor Test...->greeting postman..." << std::endl;
                JustinaIROS::loggingVisitor("postman");
                JustinaHRI::say("Hi postman, i am coming to get the post mail");
        	    ros::Duration(1.0).sleep();
                JustinaHRI::say("Please wait for the next instruction");
        	    ros::Duration(1.0).sleep();
                //id = "Unknown";
                nextState = SM_ReceiveMail;
            break;


            case SM_GuidingDoctor:
                std::cout << "Welcoming visitor Test...->guiding doctor.." << std::endl;
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                
                JustinaTasks::guideAPerson("bedroom", 50000000);
                JustinaNavigation::moveDistAngle(0.0, 3.14159, 10000);
                ros::Duration(1.0).sleep();
                JustinaHRI::waitAfterSay("Here is the annies bedroom, i will waiting for you here", 2500);
                nextState = SM_WaitVisitor;
            break;


            case SM_WaitVisitor:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
                std::cout << "Welcoming visitor Test...->SM_Waiting doctor" << std::endl;
                fileDirectory = JustinaTools::startRecordSpeach("ERL Consumer", "Welcoming Visitor");
                JustinaHRI::waitAfterSay("Tell me, justina continue, in order to follow you to the exit", 12000, maxDelayAfterSay);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                cont_z=0;
                std::cout << "Welcoming visitor Test...-> SM_WAIT_FOR_COMMAND" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("justina continue", 15000)){
                    JustinaIROS::loggingCommand("justina continue");
                    JustinaTools::stopRecordSpeach();
                    std::cout<< "audio saved in: " << fileDirectory << std::endl;
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
                
                if(id == "plumber"){
                    if(!JustinaTasks::findAndFollowPersonToLoc("entrance_door", 0, true, plumberNotAllowed))
                        if(!JustinaTasks::findAndFollowPersonToLoc("entrance_door",0,  true, plumberNotAllowed))
                            if(!JustinaTasks::findAndFollowPersonToLoc("entrance_door",0, true, plumberNotAllowed))
                                nextState = SM_FOLLOW_TO_THE_DOOR;
                }            
                else if(id == "deliman"){
                    if(!JustinaTasks::findAndFollowPersonToLoc("entrance_door", 0,true, delimanNotAllowed))
                        if(!JustinaTasks::findAndFollowPersonToLoc("entrance_door",0, true, delimanNotAllowed))
                            if(!JustinaTasks::findAndFollowPersonToLoc("entrance_door",0, true, delimanNotAllowed))
                                nextState = SM_FOLLOW_TO_THE_DOOR;
                }
                else{
                    if(!JustinaTasks::findAndFollowPersonToLoc("entrance_door"))
                        if(!JustinaTasks::findAndFollowPersonToLoc("entrance_door"))
                            if(!JustinaTasks::findAndFollowPersonToLoc("entrance_door"))
                                nextState = SM_FOLLOW_TO_THE_DOOR;
                }            
                            
                std::cout << "Welcoming visitor Test...->Follow to the door successfully" << std::endl;

                JustinaHRI::say("Thank you for your visit, see you soon");
        		ros::Duration(1.0).sleep();

                /*if (!JustinaTasks::sayAndSyncNavigateToLoc("entrance_door", 120000)) {
					std::cout << "Welcoming visitor Test...->Second attempt to move" << std::endl;
					JustinaTasks::sayAndSyncNavigateToLoc("entrance_door", 120000);
                }*/    

                JustinaHRI::say("Please close the door");
        		ros::Duration(1.0).sleep();
                JustinaHRI::say("Thank you");
        		ros::Duration(1.0).sleep();
                /*while(door || contD ==3){
                    JustinaHRI::say("Please close the door");
        		    ros::Duration(1.0).sleep();
                    door = JustinaNavigation::doorIsOpen(0.9, 2000);
                    contD++;
                    std::cout << "door: " << door << std::endl;
                }
                contD = 0;
                door = true;
                JustinaHRI::say("the door is closed, Thank you");
        		ros::Duration(1.0).sleep();*/
                JustinaIROS::loggingCommand("the door has been closed");
                nextState = SM_NavigateToInicialPoint;
            break;

            case SM_NavigateToInicialPoint:
            //TODO remplazar la location a initial point 
            std::cout << "Welcoming visitor Test...->navigate to the inicial point.." << std::endl;
                if (!JustinaTasks::sayAndSyncNavigateToLoc("initial_point", 120000)) {
					std::cout << "Welcoming visitor Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("initial_point", 120000)) {
						std::cout << "Welcoming visitor Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("initial_point", 120000)) {
							nextState = SM_WaitingDoorBell;
						}
					} 
					else{
						nextState = SM_WaitingDoorBell;
					}
				} 
				else {
					nextState = SM_WaitingDoorBell;
				}
            break;

            case SM_ReceiveMail:
                std::cout << "Welcoming visitor Test...->receiving the mail" << std::endl;
                JustinaManip::startHdGoTo(0.0, 0.0);
                //JustinaHRI::waitAfterSay("Please put in front of me to see your face", 3000);
                //ros::Duration(1.0).sleep();
                
                /*while(!recogMail && contAttempts < 3)
                {
                    faces = recognizeFaces (10000, recogMail);
                    JustinaVision::stopFaceRecognition();
                    contAttempts++;
                }*/

                /*if(faces.recog_faces.size()==0)
                {
                    JustinaHRI::say("Sorry");
                    ros::Duration(1.0).sleep();
                    JustinaHRI::say("i can not take the mail from your hand but i will take the mail if you put the mail in my gripper");
                    ros::Duration(1.0).sleep();
                    JustinaTasks::detectObjectInGripper("bag", true, 20000);
                    withLeftArm = true;
                    ros::Duration(1.0).sleep();
                }
                else{
                    JustinaManip::startHdGoTo(0.0, -0.4);
                    //JustinaHRI::say("Ready, now wait for the next instruction");
                    //ros::Duration(2.0).sleep();
                    if(JustinaTasks::graspBagHand(faces.recog_faces[0].face_centroid, withLeftArm))
                        std::cout << "test succesfully" << std::endl;
                    else
                    {
                        JustinaHRI::say("sorry i can not see your hand");
                        ros::Duration(1.0).sleep();
                        JustinaHRI::say("i can not take the mail from your hand but i will take the mail if you put the mail in my gripper");
                        ros::Duration(1.0).sleep();
                        JustinaTasks::detectObjectInGripper("bag", true, 7000);
                        withLeftArm = true;
                        ros::Duration(1.0).sleep();
                    }
                }*/
                JustinaHRI::say("sorry i can not take the mail by myself, but i will take the mail if you put the mail in my gripper");
                ros::Duration(1.3).sleep();
                JustinaTasks::detectObjectInGripper("bag", true, 7000);
                withLeftArm = true;
                ros::Duration(1.0).sleep();
                nextState = SM_DeliverMailToAnnie;
            break;

            case SM_DeliverMailToAnnie:
                std::cout <<"Welcoming visitor Test...->saying goodbye to postman" << std::endl;
                JustinaHRI::say("Thank you for your visit post man, see you soon");
        		ros::Duration(1.2).sleep();
                JustinaHRI::say("Please close the door");
        		ros::Duration(1.0).sleep();
                 /*while(door || contD ==3){
                    JustinaHRI::say("Please close the door");
        		    ros::Duration(1.0).sleep();
                    door = JustinaNavigation::doorIsOpen(0.9, 2000);
                    contD++;
                    std::cout << "door: " << door << std::endl;
                //}
                contD = 0;
                door = true;*/
                JustinaHRI::say("Thank you");
        		ros::Duration(1.0).sleep();
                JustinaIROS::loggingCommand("the door has been closed");
                JustinaHRI::say("I am going to deliver the mail to Granny Annie in the bedroom");
        		ros::Duration(3.0).sleep();
                if (!JustinaTasks::sayAndSyncNavigateToLoc("bedroom", 120000)) {
					std::cout << "Welcoming visitor Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("bedroom", 120000)) {
						std::cout << "Welcoming visitor Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("bedroom", 120000)) {
							nextState = SM_DeliverPost;
						}
					} 
					else{
						nextState = SM_DeliverPost;
					}
				} 
				else {
					nextState = SM_DeliverPost;
				}

            break;

            case SM_DeliverPost:
                std::cout <<"Welcoming visitor Test...->delivering post" << std::endl;
                //JustinaTasks::findPerson("granny annie", -1, JustinaTasks::NONE, false, "bedroom");
                JustinaHRI::say("Hi Annie, The postman brought something for you");
				ros::Duration(1.0).sleep();
                JustinaManip::laGoTo("navigation", 3000);
                JustinaTasks::dropObject("mail", withLeftArm, 10000);
                JustinaHRI::say("Bye Annie, maybe someone else will visit us today");
                ros::Duration(1.0).sleep();
                nextState=SM_NavigateToInicialPoint;
            break;
			
            case SM_FinalState:
				std::cout <<"Welcoming visitor Test...->finalState reached" << std::endl;
				JustinaHRI::say("I have finished the welcoming visitor test");
                JustinaIROS::end_execute();
				ros::Duration(2.0).sleep();
				success=true;
			break;

    }
    ros::spinOnce();
    loop.sleep();
  }
  JustinaTools::stopGlobalRecordRosbag();
  JustinaTools::stopTestRecordRosbag();
  return 0;
}
