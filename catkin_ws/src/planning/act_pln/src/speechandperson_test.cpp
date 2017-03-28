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
#include "std_msgs/Bool.h"
#include "string"

#define SM_InitialState 0
#define	SM_WaitingandTurn 10
#define SM_StatingtheCrowd 20
#define SM_RequestingOperator 30
#define	SM_RiddleGame 40
#define SM_BlindGame 50
#define SM_BlindGameRepeatQ 60
#define	SM_FinalState 70


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

bool listenAndAnswer(const int& timeout){
	std::string answer;
	std::string lastRecoSpeech;

	if(!JustinaHRI::waitForSpecificSentence(questionList, lastRecoSpeech, timeout)){
				return false;
				std::cout << "no wait for"<<std::endl;
	}
	if(!JustinaKnowledge::comparePredQuestion(lastRecoSpeech, answer))//using the knowledge node
		{return false;
			std::cout << "no compare predquestion"<<std::endl;
		}
	if(lastRecoSpeech=="what is the size of the crowd")
		answer=contC.str();
	if(lastRecoSpeech=="how many women are in the crowd")
		answer=contW.str();
	if(lastRecoSpeech=="how many men are in the crowd")
		answer=contM.str();
	if(lastRecoSpeech=="how many people in the crowd are standing")
		answer=contStanding.str();
	if(lastRecoSpeech=="how many people in the crowd are sitting")
		answer=contSitting.str();
	if(lastRecoSpeech=="how many people in the crowd are lying")
		answer=contLying.str();
	if(lastRecoSpeech=="how old do you think i am")
		answer="i think you are twenty seven years old";
	if(lastRecoSpeech=="the sitting person was a man or woman")
		answer="the sitting person was a man";
	if(lastRecoSpeech=="am i a man or a woman")
	 	answer="i couldn’t tell";
	JustinaHRI::say(answer);
	std::cout << "answer: "<< answer <<std::endl;
	return true;
}

bool listenTurnAndAnswer(const int& timeout, ros::Rate& loop){
	float audioSourceAngle = 0;
	std::string answer;
	std::string lastRecoSpeech;

	loop.sleep();

	std::cout << "Starting audio source detection" << std::endl;
	JustinaAudio::startSimpleAudioSource();
	ros::spinOnce();

	bool understood = JustinaHRI::waitForSpecificSentence(questionList, lastRecoSpeech, timeout);
	audioSourceAngle = JustinaAudio::getAudioSource();
	std::cout << "Audio source at" << (180 * audioSourceAngle / 3.141592) << "degrees" << std::endl;
	JustinaHRI::say("Wait while I turn and look at you.");
	JustinaNavigation::moveDistAngle(0, (double) audioSourceAngle, 5000);
	if(!understood || !JustinaKnowledge::comparePredQuestion(lastRecoSpeech, answer) )
		return false;
	if(lastRecoSpeech=="what is the size of the crowd")
		answer=contC.str();
	if(lastRecoSpeech=="how many women are")
		answer=contW.str();
	if(lastRecoSpeech=="how many men are")
		answer=contM.str();
	if(lastRecoSpeech=="how many people are standing")
		answer=contStanding.str();
	if(lastRecoSpeech=="how many people are sitting")
		answer=contSitting.str();
	if(lastRecoSpeech=="how many people are lying")
		answer=contLying.str();
	if(lastRecoSpeech=="how old do you think i am")
		answer="i think you are twenty seven years old";
	if(lastRecoSpeech=="the sitting person was a man or woman")
		answer="the sitting person was a man";
	if(lastRecoSpeech=="am i a man or a woman")
	 	answer="i couldn’t tell";

	JustinaHRI::say(answer);
	return true;
}


std::vector<vision_msgs::VisionFaceObject> recognizeAllFaces(float timeOut, bool &recognized){
		JustinaVision::startFaceRecognition();
		recognized = false;
		boost::posix_time::ptime curr;
		boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
		boost::posix_time::time_duration diff;
		std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;

		do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			JustinaVision::facRecognize();
			JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
			curr = boost::posix_time::second_clock::local_time();
			ros::spinOnce();
		}while(ros::ok() && (curr - prev).total_milliseconds()< timeOut);



		if(lastRecognizedFaces.size()>0)
			recognized = true;
		else
			recognized = false;

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
	JustinaKnowledge::setNodeHandle(&n);//knowledge
  ros::Rate loop(10);

	bool fail = false;
	bool success = false;

  int nextState = 30;
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
        JustinaHRI::say("I'm ready for the speech and person recognition test");
        ros::Duration(2.0).sleep();
        JustinaHRI::say("I want to play a riddle game");
        ros::Duration(12.0).sleep();
        nextState = SM_WaitingandTurn;
      break;

      case SM_WaitingandTurn:
        std::cout << "finding the crowd" << std::endl;
        JustinaHRI::say("I'm turnning around to find the crowd");
        JustinaNavigation::moveDistAngle(0.0, 3.141592, 80000);
        ros::Duration(1.0).sleep();
				//JustinaHardware::setHeadGoalPose(0.0, -0.2);
				JustinaManip::startHdGoTo(0.0, -0.2);
				ros::Duration(1.0).sleep();
        nextState = SM_StatingtheCrowd;
      break;

      case SM_StatingtheCrowd:
        std::cout << "requesting operator" << std::endl;
        JustinaHRI::say("Please do not move, I'm going to state the size of the crowd");
        while(!recog)
				{
					dFaces = recognizeAllFaces(10000,recog);
					JustinaVision::stopFaceRecognition();
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
					if(dFaces[i].face_centroid.z < 0.9)
						lying++;
					if(dFaces[i].face_centroid.z >= 0.9 & dFaces[i].face_centroid.z <1.5)
						sitting++;
					if(dFaces[i].face_centroid.z >= 1.5)
						standing++;

					std::cout<<"hombres: "<< men << std::endl;
				}
				std::cout <<"Reporting results" << std::endl;

				contCrowd=women+men+unknown;
				contC << "the size of the crowd is " <<contCrowd << std::endl;
				contW << "There are " << women << " women";
				contM << "There are " << men << " men";
				contU << "There are " << unknown << " people with unknown genre";
				contStanding << "There are" << standing << " people standing";
				contSitting << "There are" << sitting << " people sitting";
				contLying << "There are" << lying << " people lying";

				JustinaHRI::say("I am going to describe the crowd ");
				JustinaHRI::say(contC.str());
				JustinaHRI::say(contW.str());
				JustinaHRI::say(contM.str());
				JustinaHRI::say(contU.str());
				JustinaHRI::say(contStanding.str());
				JustinaHRI::say(contSitting.str());
				JustinaHRI::say(contLying.str());

				ros::Duration(2.0).sleep();
				nextState = SM_RequestingOperator;
      break;

      case SM_RequestingOperator:
				std::cout <<"Requesting Operator" << std::endl;
				JustinaHRI::say("Who want to play riddles with me?");
				ros::Duration(4.0).sleep();
				JustinaHRI::say("Please, put in front of me");
				ros::Duration(4.0).sleep();
				JustinaHRI::say("Please, tell me the first question now");
        nextState = SM_RiddleGame;
      break;

      case SM_RiddleGame:
				ros::Duration(1.0).sleep();
				ss.str(std::string()); // Clear the buffer
				if( !listenAndAnswer(10000) )
					ss << "I did not understand the question. ";
				if(++numQuestion < 6){
					ss << "Lets proceed with question " << numQuestion;
					nextState = SM_RiddleGame;
				}
				else{
					ss << "Lets proceed with the test";
					numQuestion = 1;
					nextState = SM_FinalState;
				}
				ss << ".";
				JustinaHRI::say(ss.str());
			break;

			case SM_BlindGame:
				ros::Duration(1.0).sleep();
				ss.str(std::string()); // Clear the buffer

				if( listenTurnAndAnswer(8000, loop) )
				{
					if(++numQuestion < 7){
						ss << "Lets proceed with question " << numQuestion;
						nextState = SM_BlindGame;
					}
					else{
						ss << "I will answer no more questions. Thank you!";
						nextState = SM_FinalState;
					}
				}
				else{
					ss << "I did not hear you. Please repeat question ";
					ss << numQuestion;
					nextState = SM_BlindGameRepeatQ;
				}
				ss << ".";
				JustinaHRI::say(ss.str());
				sleepAudioCaptureDelay = 4;
			break;

			case SM_BlindGameRepeatQ:
				ss.str(std::string()); // Clear the buffer
				if( !listenAndAnswer(8000) )
					ss << "I did not understood the question. ";
					if(++numQuestion < 6){
						ss << "Lets proceed with question " << numQuestion;
						nextState = SM_BlindGame;
					}
					else{
						ss << "I have finished the test";
						nextState = SM_FinalState;
					}
					ss << ".";
					JustinaHRI::say(ss.str());
			break;


			case SM_FinalState:
				std::cout <<"finalState reached" << std::endl;
				JustinaHRI::say("I have finished the speech and person recognition test...");
				success=true;
			break;

    }
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
