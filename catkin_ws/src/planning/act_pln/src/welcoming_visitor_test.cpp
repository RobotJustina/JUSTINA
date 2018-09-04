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
    bool withLeftArm=false;  
    bool validatePlumber = false;
    bool isPlumber = false;
    bool userConfirmation = false;

  	int numQuestion = 1;
  	std::string answer;
	std::stringstream ss;
    std::string lastRecoSpeech;
    int timeoutspeech = 10000;

    int attempsSpeechReco = 1;
    int attempsSpeechInt = 1;
    int attempsConfirmation = 1;
    int attempsWaitConfirmation = 1;
    int maxAttempsConfirmation = 3;
    int maxAttempsWaitConfirmation = 3;

    int maxDelayAfterSay = 300;
    int cont_z;
    int attemptsRecogLoc = 0;
    int attemptsConfLoc = 0;

	
	int contChances=0;
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

	//vector para almacenar los rostros encontrados
	//std::vector<vision_msgs::VisionFaceObject> dFaces;

	//load the predifined questions
  	//JustinaKnowledge::getPredQuestions(questionList);

  	//set the KINECT as the input device 
  	JustinaHRI::setInputDevice(JustinaHRI::RODE);

  	//almacena los rostros detectados por el servicio
  	vision_msgs::VisionFaceObjects faces;
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
                    nextState = SM_GreetingPostman;
                }
                else if(id == "unknown")
                {
                    std::cout << "Welcoming visitor Test...->unknown recognized.." << std::endl;
                    nextState = SM_InterrogatePerson;
                }
                else
                    nextState = SM_RecognizeVisitor;
            break;

            case SM_InterrogatePerson:
                std::cout << "Welcoming visitor Test...->interrogate person.." << std::endl;
                JustinaHRI::say("Sorry, but i can not recognize you");
        	    ros::Duration(1.0).sleep();
                JustinaHRI::say("I need to ask you some questions to try to identify you");
        	    ros::Duration(1.0).sleep();
                JustinaHRI::enableSpeechRecognized(false);
                if(validatePlumber)
                    JustinaHRI::waitAfterSay("Do you want to repair something in the house, please tell me justina yes or justina no", 12000, maxDelayAfterSay);
                else 
                    JustinaHRI::waitAfterSay("Do you want to deliver something, please tell me justina yes or justina no", 11000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);
                nextState = SM_IdentityConfirm;
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
                            JustinaHRI::say("Hello Plumber, my name is Justina");
        	                ros::Duration(1.0).sleep();
                            JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want to visit the kitchen", 5000, maxDelayAfterSay);
                            JustinaHRI::loadGrammarSpeechRecognized(grammarPlumber);
                            nextState = SM_GreetingPlumber;
                        }
                        else{
                            JustinaHRI::say("Hello Deli man, my name is Justina");
        	                ros::Duration(1.0).sleep();
                            JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want to visit the kitchen", 5000, maxDelayAfterSay);
                            JustinaHRI::loadGrammarSpeechRecognized(grammarDeliman);
                            nextState = SM_GreetingDeliman;
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
                                JustinaHRI::loadGrammarSpeechRecognized(grammarPlumber);
                                nextState = SM_GreetingPlumber;
                            }
                            else{
                                JustinaHRI::say("Hello Deli man, my name is Justina");
        	                    ros::Duration(1.0).sleep();
                                JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want to visit the kitchen", 5000, maxDelayAfterSay);
                                JustinaHRI::loadGrammarSpeechRecognized(grammarDeliman);
                                nextState = SM_GreetingDeliman;
                            }
                            JustinaHRI::enableSpeechRecognized(true);
                            //nextState = SM_TAKE_ORDER;
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
                            
                            cont_z=8;
                            nextState = SM_GreetingPlumber;
                        }
                        else{
                            JustinaHRI::say("Hello Deli man, my name is Justina");
                            ros::Duration(1.0).sleep();
                            JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want to visit the kitchen", 5000, maxDelayAfterSay);
                            JustinaHRI::loadGrammarSpeechRecognized(grammarDeliman);
                            nextState = SM_GreetingDeliman;
                        }
                        //nextState = SM_TAKE_ORDER;
                    }
                }
            break;

            case SM_GreetingPlumber:
                std::cout << "Welcoming visitor Test...->greeting plumber.." << std::endl;
                if(cont_z > 3){
                    JustinaHRI::waitAfterSay("Please tell me wich room do you want to visit, for example, i want  to visit the kitchen", 5000, maxDelayAfterSay);
                    JustinaHRI::loadGrammarSpeechRecognized(grammarPlumber);
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
                    else if (lastRecoSpeech.find("the bathromm") != std::string::npos)
                        location = "bathroom";
                     else if(attemptsRecogLoc >= MAX_ATTEMPTS_RECOG){
                        location = "bathroom";
                    } 

                    std::cout << "Welcoming visitor Test...->verify if the person is allowed to enter to the room" << std::endl;
                    
                    if(location=="bedroom"){
                        JustinaHRI::say("Sorry plumber but you are not allowed to visit the bedroom");
                        ros::Duration(1.5).sleep();
                        cont_z = 8;
                        nextState = SM_GreetingPlumber;
                        break; 
                    }

                    else if(location=="kitchen"){
                        JustinaHRI::say("do you want to go to the kitchen");
                        ros::Duration(1.0).sleep();
                        nextState = SM_ConfirmLocation;
                        JustinaHRI::enableSpeechRecognized(true);
                    }
                    
                    else if (location=="bathroom"){
                        JustinaHRI::say("do you want to go to the bathroom");
                        ros::Duration(1.0).sleep();
                        nextState = SM_ConfirmLocation;
                        JustinaHRI::enableSpeechRecognized(true);
                    }

                }  
            break;

            case SM_ConfirmLocation:
                std::cout << "Welcoming visitor Test...->confirm location" << std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                JustinaHRI::waitForUserConfirmation(userConfirmation, 7000);
                attemptsConfLoc++;
                if(userConfirmation)
                    nextState = SM_GuidingPlumber;
                else if(attemptsConfLoc < MAX_ATTEMPTS_CONF){
                    nextState = SM_GreetingPlumber;
                    cont_z = 8;
                }
                else
                    nextState = SM_GuidingPlumber;
            break;

            case SM_GuidingPlumber:
                std::cout << "Welcoming visitor Test...->go to location" << std::endl;
                std::cout << "Welcoming visitor Test...->guiding doctor.." << std::endl;
                JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                
                JustinaTasks::guideAPerson(location, 50000000);
                JustinaHRI::waitAfterSay("i will waiting for you here", 2500);
                nextState = SM_WaitVisitor;

            break;

            case SM_GreetingDoctor:
                std::cout << "Welcoming visitor Test...->greeting doctor.." << std::endl;
                JustinaHRI::say("Hello Doctor Kimble, we are waiting for you");
        	    ros::Duration(1.0).sleep();
                JustinaHRI::say("Please let me guide you to Annies bedroom");
        	    ros::Duration(1.0).sleep();
                nextState = SM_GuidingDoctor;
            break;

            case SM_GreetingPostman:
                std::cout << "Welcoming visitor Test...->greeting postman..." << std::endl;
                JustinaHRI::say("Hi postman, i am coming to get the post mail");
        	    ros::Duration(1.0).sleep();
                JustinaHRI::say("Please wait for the next instruction");
        	    ros::Duration(1.0).sleep();
                nextState = SM_ReceiveMail;
            break;


            case SM_GuidingDoctor:
                std::cout << "Welcoming visitor Test...->guiding doctor.." << std::endl;
                JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                
                JustinaTasks::guideAPerson("bedroom", 50000000);
                JustinaHRI::waitAfterSay("Here is the annies bedroom, i will waiting for you here", 2500);
                nextState = SM_WaitVisitor;
            break;


            case SM_WaitVisitor:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
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

            case SM_NavigateToInicialPoint:
            //TODO remplazar la location a initial point 
            std::cout << "Welcoming visitor Test...->navigate to the inicial point.." << std::endl;
                if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
					std::cout << "Welcoming visitor Test...->Second attempt to move" << std::endl;
					if (!JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
						std::cout << "Welcoming visitor Test...->Third attempt to move" << std::endl;
						if (JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000)) {
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
                
                while(!recog && contChances < 3)
                {
                    faces = recognizeFaces (10000, recog);
                    JustinaVision::stopFaceRecognition();
                    contChances++;
                }

                if(faces.recog_faces.size()==0)
                {
                    JustinaHRI::say("Sorry");
                    ros::Duration(1.5).sleep();
                    JustinaHRI::say("i can not take the mail form your hand but i will take the mail if you put the mail in my gripper");
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
                        JustinaHRI::say("i can not take the mail form your hand but i will take the mail if you put the mail in my gripper");
                        ros::Duration(1.0).sleep();
                        JustinaTasks::detectObjectInGripper("bag", true, 7000);
                        withLeftArm = true;
                        ros::Duration(1.0).sleep();
                    }
                }
                nextState = SM_DeliverMailToAnnie;
            break;

            case SM_DeliverMailToAnnie:
                std::cout <<"Welcoming visitor Test...->saying goodbye to postman" << std::endl;
                JustinaHRI::say("Thank you for your visit post man, see you soon");
        		ros::Duration(2.0).sleep();
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
                JustinaHRI::say("Hi Annie, The postman brought something for you");
				ros::Duration(2.0).sleep();
                JustinaManip::laGoTo("navigation", 3000);
                JustinaTasks::dropObject("mail", true, 10000);
                JustinaHRI::say("Bye Annie, maybe someone else will visit us today");
                ros::Duration(1.0).sleep();
                nextState=SM_NavigateToInicialPoint;
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
