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
#include "string"

#define SM_InitialState 0
#define	SM_MeetandGreet 10
#define SM_WAIT_FOR_OPERATOR 20
#define SM_MEMORIZING_OPERATOR 30
#define SM_WAIT_FOR_LEGS_FOUND 40
#define SM_Followme 50
#define SM_Findperson 60
#define	SM_TakeBag 70
#define SM_Goodbye 80
#define	SM_FinalState 90


//std::string personName = "operator";
std::map<std::string, std::string> questionsL;
std::vector<std::string> questionList;

std::stringstream contW;
std::stringstream contM;
std::stringstream contU;
std::stringstream profPlace;
std::stringstream genderOperator;
std::stringstream contC;
std::stringstream contStanding;
std::stringstream contSitting;
std::stringstream contLying;


int main(int argc, char** argv)
{
	std::cout << "Initializing TV Azteca Test..." << std::endl;
  ros::init(argc, argv, "act_pln");
  ros::NodeHandle n;
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


  while(ros::ok() && !fail && !success)
  {

  	switch(nextState)
    {

    	case SM_InitialState:
      	std::cout << "start the TV Azteca test" << std::endl;
        JustinaHardware::setHeadGoalPose(0.0, 0.0);
        ros::Duration(2.0).sleep();
        nextState = SM_MeetandGreet;
      break;

      case SM_MeetandGreet:
        std::cout << "Greeting to the audience" << std::endl;
        JustinaHRI::say("Buenas tardes, mi nombre es Justina");
				ros::Duration(1.0).sleep();
        JustinaHRI::say("El nombre de mi equipo es pumas");
        ros::Duration(1.0).sleep();
				JustinaHRI::say("Represento a la facultad de ingenieria de la unam");
				ros::Duration(1.0).sleep();
				JustinaHRI::say("El dia de hoy me encuentro en el noticiero hechos sabado con Mariano Riva Palacio");
				ros::Duration(1.0).sleep();
				JustinaHRI::say("a continucion mostrare algunas de mis habilidades");
				ros::Duration(1.0).sleep();
        nextState = SM_Followme;
      break;

			case SM_WAIT_FOR_OPERATOR:

				std::cout << "waiting for the operator" << std::endl;
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
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
						JustinaHRI::say("por favor repite el comando");

				else{
						if(lastRecoSpeech.find("justina busca una persona") != std::string::npos){
							JustinaHRI::say("ok buscare a una persona");
							ros::Duration(1.0).sleep();
							if(JustinaTasks::findPerson())
								nextState=SM_TakeBag;
							else
								JustinaHRI::say("No encontre a nadie lo intentare de nuevo");
						}
				}
			break;

			case SM_TakeBag:
				std::cout << "taking the bag" << std::endl;
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000)){
					JustinaHRI::say("por favor repite el comando");
					ros::Duration(1.0).sleep();
				}
				else{
					if(lastRecoSpeech.find("justina toma esta bolsa") != std::string::npos){
						JustinaManip::raGoTo("take", 10000);
            JustinaManip::startRaOpenGripper(0.6);
            JustinaManip::hdGoTo(0, -0.9, 5000);
            JustinaHRI::say("Por favor pon la bolsa en mi mano");
						ros::Duration(1.0).sleep();
            JustinaManip::getRightHandPosition(x, y, z);
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
            JustinaManip::startRaCloseGripper(0.4);
          	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            JustinaManip::raGoTo("navigation", 10000);
					}
				}
			nextState = SM_Goodbye;
			break;

			case SM_Goodbye:
				JustinaManip::hdGoTo(0, 0, 5000);
				ros::Duration(1.0).sleep();
				JustinaHRI::say("Ahora que quieres que haga");
				ros::Duration(1.0).sleep();
				if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000)){
					JustinaHRI::say("por favor repite el comando");
					ros::Duration(1.0).sleep();
				}
				else{
					if(lastRecoSpeech.find("justina despidete") != std::string::npos){
						JustinaHRI::say("Hasta luego televidentes de Hechos sabado, nos vemos en Japon");
						ros::Duration(1.0).sleep();
						JustinaHRI::say("Sayonara");
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
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
