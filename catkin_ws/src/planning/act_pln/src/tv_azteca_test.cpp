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
#include "justina_tools/JustinaTasks.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "string"

#define SM_InitialState 0
#define SM_Wait_Initial_Command 10
#define	SM_MeetandGreet 20
#define SM_WAIT_FOR_OPERATOR 30
#define SM_MEMORIZING_OPERATOR 40
#define SM_WAIT_FOR_LEGS_FOUND 50
#define SM_Followme 60
#define SM_Findperson 70
#define	SM_TakeBag 80
#define SM_Goodbye 90
#define	SM_FinalState 100
#define SM_GiveBag 110

bool skip_state = false;

void hardcode_next_state_callback(const std_msgs::Empty::ConstPtr& msg){
	std::cout << "Skip next state." << std::endl;
	skip_state = true;
}


int main(int argc, char** argv)
{
	std::cout << "Initializing TV Azteca Test..." << std::endl;
  ros::init(argc, argv, "act_pln");
  ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/hardware/robot_state_skip_state", 1, hardcode_next_state_callback);

  JustinaHardware::setNodeHandle(&n);
  JustinaHRI::setNodeHandle(&n);
  JustinaManip::setNodeHandle(&n);
  JustinaNavigation::setNodeHandle(&n);
  JustinaTools::setNodeHandle(&n);
  JustinaVision::setNodeHandle(&n);
	JustinaAudio::setNodeHandle(&n);
	JustinaTasks::setNodeHandle(&n);
	JustinaKnowledge::setNodeHandle(&n);//knowledge
  ros::Rate loop(10);

	boost::posix_time::ptime prev;
	boost::posix_time::ptime curr;

	bool fail = false;
	bool success = false;
	int nextState = 0;

	float x, y ,z;

	std::string lastRecoSpeech;

	std::vector<std::string> validCommands;
  validCommands.push_back("justina sigueme");
	validCommands.push_back("justina alto");
	validCommands.push_back("justina busca una persona");
	validCommands.push_back("justina toma esta bolsa");
	validCommands.push_back("justina despidete");
	validCommands.push_back("justina presentate");
	validCommands.push_back("justina entrega la bolsa");


  while(ros::ok() && !fail && !success)
  {

  	switch(nextState)
    {

    	case SM_InitialState:
      	std::cout << "start the TV Azteca test" << std::endl;
        JustinaHardware::setHeadGoalPose(0.0, 0.0);
        ros::Duration(2.0).sleep();
				if(skip_state)
        	nextState = SM_Wait_Initial_Command;
      break;

			case SM_Wait_Initial_Command:

				if(skip_state){
					nextState = SM_MeetandGreet;
					break;
				}
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000)){
						std::cout << "waiting the initial command" << std::endl;
						nextState = SM_Wait_Initial_Command;
				}

				else{
					if(lastRecoSpeech.find("justina presentate") != std::string::npos){
						std::cout << "start the introduction" << std::endl;
						nextState = SM_MeetandGreet;
					}
				}
			break;

      case SM_MeetandGreet:
        std::cout << "Greeting to the audience" << std::endl;
        JustinaHRI::say("Buenas tardes, mi nombre es JUSTINA");
				ros::Duration(1.0).sleep();
        JustinaHRI::say("El nombre de mi equipo es pumas");
        ros::Duration(1.0).sleep();
				JustinaHRI::say("Represento a la facultad de ingenieria de la unam");
				ros::Duration(1.0).sleep();
				JustinaHRI::say("El dia de hoy me encuentro en el noticiero hechos sabado con Mariano Riva Palacio");
				ros::Duration(1.0).sleep();
				JustinaHRI::say("puedo hacer algo mas por ti");
				ros::Duration(1.0).sleep();
				nextState = SM_TakeBag;
      break;

			case SM_TakeBag:
				std::cout << "taking the bag" << std::endl;
				if(skip_state){
					nextState = SM_WAIT_FOR_OPERATOR;
					break;
				}
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000)){
					JustinaHRI::say("por favor repite el comando");
					ros::Duration(1.0).sleep();
				}
				else{
					if(lastRecoSpeech.find("justina toma esta bolsa") != std::string::npos){
						JustinaHRI::say("Por favor pon la bolsa en mi mano");
						ros::Duration(1.0).sleep();
						JustinaManip::laGoTo("take", 10000);
						JustinaManip::startLaOpenGripper(0.6);
						JustinaManip::hdGoTo(0, -0.9, 5000);
						JustinaManip::getLeftHandPosition(x, y, z);
						boost::this_thread::sleep(boost::posix_time::milliseconds(200));
						std::cout << "helMeCarry.->Point(" << x << "," << y << "," << z << ")" << std::endl;
						JustinaVision::startHandDetectBB(x, y, z);
						prev = boost::posix_time::second_clock::local_time();
						curr = prev;
						while(ros::ok() && !JustinaVision::getDetectionHandBB() && (curr - prev).total_milliseconds() < 30000){
							loop.sleep();
							ros::spinOnce();
							curr = boost::posix_time::second_clock::local_time();
						}
						JustinaVision::stopHandDetectBB();
						ros::Duration(1.0).sleep();
						JustinaHRI::say("Gracias");
						ros::Duration(1.0).sleep();
						JustinaManip::hdGoTo(0, 0.0, 5000);
						JustinaManip::startLaCloseGripper(0.4);
						boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
						JustinaManip::laGoTo("navigation", 10000);
						JustinaHRI::say("necesitas algo mas");
						ros::Duration(1.0).sleep();
						nextState = SM_WAIT_FOR_OPERATOR;
					}
				}

			break;


			case SM_WAIT_FOR_OPERATOR:
				std::cout << "waiting for the operator" << std::endl;
				if(skip_state){
					nextState = SM_MEMORIZING_OPERATOR;
					break;
				}
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
          	JustinaHRI::say("por favor repite el comando");
        else{
          	if(lastRecoSpeech.find("justina sigueme") != std::string::npos)
          		nextState = SM_MEMORIZING_OPERATOR;
          	else
          		nextState = SM_WAIT_FOR_OPERATOR;
        }
      break;

      case SM_MEMORIZING_OPERATOR:
				std::cout << "MEMORIZING the OPERATOR" << std::endl;
				JustinaHRI::say("por favor colocate frente a mi");
				ros::Duration(1.0).sleep();
	    	JustinaHRI::enableLegFinder(true);
				nextState=SM_WAIT_FOR_LEGS_FOUND;
			break;

			case SM_WAIT_FOR_LEGS_FOUND:
				std::cout << "finding legs" << std::endl;
				if(skip_state){
					nextState = SM_Followme;
					break;
				}
				if(JustinaHRI::frontalLegsFound())
        {
        	std::cout << "NavigTest.->Frontal legs found!" << std::endl;
          JustinaHRI::say("te encontre");
          ros::Duration(1.0).sleep();
					JustinaHRI::say("Ahora te comenzare a seguir");
        	nextState = SM_Followme;
        }
				else{
					std::cout << "trying again" << std::endl;
					nextState = SM_WAIT_FOR_LEGS_FOUND;
				}
      break;

      case SM_Followme:
				std::cout << "Starting following phase" << std::endl;
				if(skip_state){
					nextState = SM_Findperson;
					break;
				}
				JustinaHRI::startFollowHuman();
				ros::spinOnce();
				if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
					if(lastRecoSpeech.find("justina alto") != std::string::npos)
							JustinaHRI::stopFollowHuman();

					JustinaHRI::say("Ok dejare de seguirte");
					ros::Duration(1.0).sleep();
					JustinaHRI::stopFollowHuman();
					JustinaHRI::stopFollowHuman();
					JustinaHRI::stopFollowHuman();
					ros::spinOnce();
					JustinaHRI::say("estoy lista para realizar otra tarea");
					ros::Duration(1.0).sleep();
					nextState = SM_Findperson;
					break;
				}

				if(!JustinaHRI::frontalLegsFound()){
					std::cout << "SM_FOLLOWING_PHASE -> Lost human!" << std::endl;
									JustinaHRI::say("Te he perdido");
									ros::Duration(3.0).sleep();
				}
      break;

      case SM_Findperson:
				std::cout << "finding a person" << std::endl;
				if(skip_state){
					nextState = SM_Findperson;
					break;
				}
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
						JustinaHRI::say("por favor repite el comando");

				else{
						if(lastRecoSpeech.find("justina busca una persona") != std::string::npos){
							JustinaHRI::say("ok buscare a una persona");
							ros::Duration(1.0).sleep();
							if(JustinaTasks::findPerson()){
								JustinaHRI::say("deseas algo mas");
								ros::Duration(1.0).sleep();
								nextState=SM_GiveBag;
							}
							else
								JustinaHRI::say("No encontre a nadie lo intentare de nuevo");
						}
				}
			break;

			case SM_GiveBag:
			std::cout << "giving the bag" << std::endl;
			if(skip_state){
				nextState = SM_Findperson;
				break;
			}
			if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000)){
				JustinaHRI::say("por favor repite el comando");
				ros::Duration(1.0).sleep();
			}
			else{
				if(lastRecoSpeech.find("justina entrega la bolsa") != std::string::npos){
					JustinaHRI::say("Por favor toma la bolsa de mi mano");
					ros::Duration(1.0).sleep();
					JustinaManip::laGoTo("take", 10000);
					JustinaManip::hdGoTo(0, -0.9, 5000);
					JustinaManip::getLeftHandPosition(x, y, z);
					boost::this_thread::sleep(boost::posix_time::milliseconds(200));
					std::cout << "helMeCarry.->Point(" << x << "," << y << "," << z << ")" << std::endl;
					JustinaVision::startHandDetectBB(x, y, z);
					prev = boost::posix_time::second_clock::local_time();
					curr = prev;
					while(ros::ok() && !JustinaVision::getDetectionHandBB() && (curr - prev).total_milliseconds() < 30000){
						loop.sleep();
						ros::spinOnce();
						curr = boost::posix_time::second_clock::local_time();
					}
					JustinaVision::stopHandDetectBB();
					JustinaManip::startLaOpenGripper(0.6);
					ros::Duration(1.0).sleep();
					JustinaHRI::say("Gracias");
					ros::Duration(1.0).sleep();
					JustinaManip::startLaCloseGripper(0.4);
					boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
					JustinaManip::laGoTo("home", 10000);
					JustinaHRI::say("puedo hacer algo mas por ti");
					ros::Duration(1.0).sleep();
					nextState = SM_Goodbye;
				}
			}

			break;


			case SM_Goodbye:
				if(skip_state){
					nextState = SM_Findperson;
					break;
				}
				JustinaManip::hdGoTo(0, 0, 5000);
				ros::Duration(1.0).sleep();
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000)){
					JustinaHRI::say("por favor repite el comando");
					ros::Duration(1.0).sleep();
					nextState = SM_Goodbye;
				}
				else{
					if(lastRecoSpeech.find("justina despidete") != std::string::npos){
						JustinaHRI::say("Hasta luego televidentes de Hechos sabado, nos vemos en JAPON");
						ros::Duration(1.0).sleep();
						JustinaHRI::say("SAYONARA");
						ros::Duration(1.0).sleep();
					}
				}
				nextState = SM_FinalState;
			break;


			case SM_FinalState:
				std::cout <<"finalState reached" << std::endl;
				success=true;
			break;

    }
		skip_state = false;
    loop.sleep();
		ros::spinOnce();
  }
  return 0;
}
