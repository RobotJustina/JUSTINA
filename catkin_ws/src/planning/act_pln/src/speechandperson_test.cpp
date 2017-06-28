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
#include "std_msgs/Bool.h"
#include "string"

#define SM_InitialState 0
#define	SM_WaitingandTurn 10
#define SM_StatingtheCrowd 20
#define SM_RequestingOperator 30
#define	SM_RiddleGame 40
#define SM_WaitBlindGame 50
#define SM_BlindGame 60
#define SM_BlindGameRepeatQ 70
#define	SM_FinalState 80


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
std::stringstream contFake;

void confirmSizeCrowd()
{
	JustinaHRI::say(contFake.str());
	ros::Duration(2.5).sleep();
	JustinaNavigation::moveDistAngle(0.5, 0.0, 80000);
    ros::Duration(2.0).sleep();
	JustinaManip::startHdGoTo(-0.4, -0.15);
	ros::Duration(3.0).sleep();
	JustinaManip::startHdGoTo(0.0, -0.15);
	ros::Duration(3.0).sleep();
	JustinaManip::startHdGoTo(0.4, -0.15);
	ros::Duration(3.0).sleep();
	JustinaManip::startHdGoTo(0.0, 0.0);
	ros::Duration(3.0).sleep();
	JustinaHRI::say("I have verified the information ");
	ros::Duration(1.0).sleep();
	JustinaHRI::say("I am going to describe the crowd ");
	ros::Duration(1.0).sleep();
	JustinaHRI::say(contC.str());
	ros::Duration(1.0).sleep();
	JustinaHRI::say(contW.str());
	ros::Duration(1.0).sleep();
	JustinaHRI::say(contM.str());
	ros::Duration(1.0).sleep();
}

bool listenAndAnswer(const int& timeout)
{
	std::string answer;
	std::string lastRecoSpeech;
	bool PredQ;
	bool KDBQ;

	if(!JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeout))
	{
		std::cout << "no wait for"<<std::endl;
		return false;
	}

	//convert the lastRecoSpeech to lower case
	boost::to_lower(lastRecoSpeech);

	if(!JustinaKnowledge::comparePredQuestion(lastRecoSpeech,answer))
	{
		if(!JustinaRepresentation::answerQuestionFromKDB(lastRecoSpeech, answer, 500))
		{
			std::cout << "no match with any question" << std::endl;
			return false; 
		}
	}
	
	JustinaHRI::say(answer);
	ros::Duration(2.0).sleep();
	return true;
}

bool listenTurnAndAnswer(const int& timeout, ros::Rate& loop)
{
	float audioSourceAngle = 0;
	std::string answer;
	std::string lastRecoSpeech;
	bool PredQ;
	bool KDBQ;


	std::cout << "Starting audio source detection" << std::endl;
	JustinaAudio::startSimpleAudioSource();
	ros::spinOnce();
	ros::Duration(1.0).sleep();

	if(!JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeout))
	{
		std::cout << "no wait for"<<std::endl;
		return false;
	}

	
	//convert string to lower case
	boost::to_lower(lastRecoSpeech);
	ros::Duration(1.0).sleep();
	audioSourceAngle = JustinaAudio::getAudioSource();
	std::cout << "Audio source at" << (180 * audioSourceAngle / 3.141592) << "degrees" << std::endl;
	JustinaHRI::say("Wait while I turn and look at you");
	ros::Duration(1.0).sleep();
	JustinaNavigation::moveDistAngle(0, (double) audioSourceAngle, 5000);

	if(!JustinaKnowledge::comparePredQuestion(lastRecoSpeech,answer))
	{
		if(!JustinaRepresentation::answerQuestionFromKDB(lastRecoSpeech, answer, 500))
		{
			std::cout << "no match with any question" << std::endl;
			return false; 
		}
	}

	
	JustinaHRI::say(answer);
	ros::Duration(2.0).sleep();
	return true;
}


std::vector<vision_msgs::VisionFaceObject> recognizeAllFaces(float timeOut, bool &recognized)
{
	JustinaVision::startFaceRecognition();
	recognized = false;
	int previousSize = 20;
	int sameValue = 0;
	boost::posix_time::ptime curr;
	boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
	boost::posix_time::time_duration diff;
	std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;

	do
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		JustinaVision::facRecognize();
		JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
		ros::Duration(1.0).sleep();
		
		if(lastRecognizedFaces.size() == previousSize && lastRecognizedFaces.size() > 0)
			sameValue ++;
		if(sameValue == 3)
			recognized = true;
		else
		{
			previousSize = lastRecognizedFaces.size();
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
	JustinaKnowledge::setNodeHandle(&n);//knowledge

	JustinaRepresentation::initKDB("", true, 20000);

  	ros::Rate loop(10);

	bool fail = false;
	bool success = false;

  	int nextState = 0;
  	bool recog=false;
  	int numQuestion = 1;
  	std::string answer;
	std::stringstream ss;

	int mIndex=0;
	int women=0;
	int men=0;
	int unknown=0;
	int genero=10;
	int contCrowd=0;
	int standing=0;
	int sitting=0;
	int lying=0;
	int contChances=0;

	//vector para almacenar los rostros encontrados
	std::vector<vision_msgs::VisionFaceObject> dFaces;
	//load the predifined questions
  	JustinaKnowledge::getPredQuestions(questionList);

	int sleepAudioCaptureDelay = 4;


  	while(ros::ok() && !fail && !success)
  	{
		ros::Rate loop(sleepAudioCaptureDelay);
  		switch(nextState)
    	{

    		case SM_InitialState:
      			std::cout << "start the speech and person recognition test" << std::endl;
        		JustinaHardware::setHeadGoalPose(0.0, 0.0);
        		JustinaHRI::say("I am ready for the speech and person recognition test");
        		ros::Duration(2.0).sleep();
        		JustinaHRI::say("I want to play a riddle game");
        		ros::Duration(5.0).sleep();
        		nextState = SM_WaitingandTurn;
      		break;

      		case SM_WaitingandTurn:
        		std::cout << "finding the crowd" << std::endl;
        		JustinaHRI::say("I am turnning around to find the crowd");
				ros::Duration(1.0).sleep();
        		JustinaNavigation::moveDistAngle(0.0, 3.141592, 80000);
        		ros::Duration(1.0).sleep();
				JustinaManip::startHdGoTo(0.0, -0.15);
				ros::Duration(1.0).sleep();
        		nextState = SM_StatingtheCrowd;
      		break;

      		case SM_StatingtheCrowd:
        		std::cout << "requesting operator" << std::endl;
        		JustinaHRI::say("Please do not move, I am going to state the size of the crowd");
				ros::Duration(1.5).sleep();
        		while(!recog && contChances < 3)
				{
					dFaces = recognizeAllFaces(10000,recog);
					boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
					JustinaVision::stopFaceRecognition();
					contChances++;
				}

				std::cout <<"tamaño de arreglo " << dFaces.size() <<std::endl;

				for(int i=0; i<dFaces.size(); i++)
				{
					if(dFaces[i].gender==0)
						women++;
					if(dFaces[i].gender==1)
						men++;
					if(dFaces[i].gender==2)
						unknown++;
					if(dFaces[i].face_centroid.z < 0.8)
						lying++;
					if(dFaces[i].face_centroid.z >= 0.8 && dFaces[i].face_centroid.z <1.20)
						sitting++;
					if(dFaces[i].face_centroid.z >= 1.20)
						standing++;

					std::cout<<"hombres: "<< men << std::endl;
				}

				std::cout <<"Reporting results" << std::endl;

				contCrowd=women+men+unknown;
				contC << "the size of the crowd is " <<contCrowd << std::endl;
				contFake << "i think there are " << contCrowd << "people in the scene i will verify it" << std::endl;
				contW << "There are " << women << " women";
				contM << "There are " << men << " men";
				//contU << "There are " << unknown << " people with unknown genre";
				contStanding << "There are " << standing << " people standing";
				contSitting << "There are " << sitting << " people sitting";
				contLying << "There are " << lying << " people lying";

				if(dFaces.size()==0)
				{
					JustinaHRI::say("Sorry, I cannot state the size of the crowd, lets proceed with the test");
					ros::Duration(1.5).sleep();
					nextState = SM_RequestingOperator;
	      			break;
				}

				JustinaManip::startHdGoTo(0.0, 0.0);
				ros::Duration(1.0).sleep();
				//just for simulating movement
				//confirmSizeCrowd();
				JustinaHRI::say(contFake.str());
				ros::Duration(2.5).sleep();
				JustinaNavigation::moveDistAngle(0.5, 0.0, 80000);
        		ros::Duration(2.0).sleep();
				JustinaManip::startHdGoTo(-0.4, -0.15);
				ros::Duration(3.0).sleep();
				JustinaManip::startHdGoTo(0.0, -0.15);
				ros::Duration(3.0).sleep();
				JustinaManip::startHdGoTo(0.4, -0.15);
				ros::Duration(3.0).sleep();
				JustinaManip::startHdGoTo(0.0, 0.0);
				ros::Duration(3.0).sleep();
				JustinaHRI::say("I have verified the information ");
				ros::Duration(1.0).sleep();
				JustinaHRI::say("I am going to describe the crowd ");
				ros::Duration(1.0).sleep();
				JustinaHRI::say(contC.str());
				ros::Duration(1.0).sleep();
				JustinaHRI::say(contW.str());
				ros::Duration(1.0).sleep();
				JustinaHRI::say(contM.str());
				ros::Duration(1.0).sleep();
				std::cout<<"standing: "<< standing << std::endl;
				std::cout<<"sitting: "<< sitting << std::endl;
				std::cout<<"lying: "<< lying << std::endl;
				ros::Duration(1.0).sleep();
				nextState = SM_RequestingOperator;
      		break;

      		case SM_RequestingOperator:
				std::cout <<"Requesting Operator" << std::endl;
				JustinaHRI::say("Who want to play riddles with me");
				ros::Duration(1.5).sleep();
				JustinaHRI::say("Please, put in front of me");
				ros::Duration(1.5).sleep();
				JustinaHRI::say("Please, tell me the first question now");
				ros::Duration(1.5).sleep();
				//JustinaHRI::playSound();
				//ros::Duration(1.0).sleep();
        		nextState = SM_RiddleGame;
      		break;

      		case SM_RiddleGame:
				ros::Duration(1.0).sleep();
				ss.str(std::string()); // Clear the buffer
				if( !listenAndAnswer(10000))
					ss << "I did not understand the question ";
				if(++numQuestion < 6)
				{
					ss << "Please, tell me the question number " << numQuestion << " now";
					nextState = SM_RiddleGame;
				}
				else
				{
					ss << "Lets proceed with the blind mans bluff game";
					numQuestion = 1;
					nextState = SM_WaitBlindGame;
				}
				ss << ".";
				JustinaHRI::say(ss.str());
				//JustinaHRI::playSound();
				//ros::Duration(1.0).sleep();
				ros::Duration(1.5).sleep();
			break;

			case SM_WaitBlindGame:
				JustinaHRI::say("I will give you a few seconds to move around me, please, wait for the next instruction ");
				ros::Duration(9.0).sleep();
				JustinaHRI::playSound();
				ros::Duration(1.0).sleep();
				JustinaHRI::say("Ready, Please, tell me the first question now");
				//ros::Duration(1.5).sleep();
				//JustinaHRI::playSound();
				//ros::Duration(1.0).sleep();
				nextState = SM_BlindGame;
			break;

			case SM_BlindGame:
				ss.str(std::string()); // Clear the buffer
				if( listenTurnAndAnswer(8000, loop) )
				{
					if(++numQuestion < 6)
					{
						ss << "Please, tell me the question number " << numQuestion << " now";
						nextState = SM_BlindGame;
					}
					else
					{
						ss << "I will answer no more questions. Thank you";
						nextState = SM_FinalState;
					}
				}
				else
				{
					ss << "I did not hear you. Please repeat the question ";
					ss << numQuestion;
					nextState = SM_BlindGameRepeatQ;
				}
				ss << ".";
				JustinaHRI::say(ss.str());
				//ros::Duration(2.0).sleep();
				//JustinaHRI::playSound();
				//ros::Duration(1.0).sleep();
				sleepAudioCaptureDelay = 4;
			break;

			case SM_BlindGameRepeatQ:
				ss.str(std::string()); // Clear the buffer
				if( !listenAndAnswer(8000) )
					ss << "I did not understand the question. ";
				if(++numQuestion < 6)
				{
					ss << "Please, tell me the question number " << numQuestion << " now";
					nextState = SM_BlindGame;
				}
				else
				{
					ss << "I have finished the test";
					nextState = SM_FinalState;
				}
				ss << ".";
				JustinaHRI::say(ss.str());
				ros::Duration(2.0).sleep();
				//JustinaHRI::playSound();
				//ros::Duration(1.0).sleep();
			break;


			case SM_FinalState:
				//save results on PDF
				JustinaTools::pdfImageExport("SpeechAndPersonRecognitionTest","/home/$USER/faces/");
				std::cout <<"finalState reached" << std::endl;
				JustinaHRI::say("I have finished the speech and person recognition test...");
				ros::Duration(2.0).sleep();
				success=true;
			break;

    }
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
