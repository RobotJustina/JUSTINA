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
#define	SM_WaitingandTurn 10
#define SM_StatingtheCrowd 20
#define SM_RequestingOperator 30
#define	SM_RiddleGame 40
#define SM_WaitBlindGame 50
#define SM_BlindGame 60
#define SM_BlindGameRepeatQ 70
#define	SM_FinalState 80

std::vector<std::string> questionList;

int nearestFace(vision_msgs::VisionFaceObjects faces){
	float distanceAux;
	float distance = 100.0;
	int auxIndex;
	int giro = 0;

	for(int i=0; i<faces.recog_faces.size(); i++){
		distanceAux=sqrt((faces.recog_faces[i].face_centroid.x * faces.recog_faces[i].face_centroid.x) +
						(faces.recog_faces[i].face_centroid.y * faces.recog_faces[i].face_centroid.y));
		if(distanceAux <= distance){
			distance=distanceAux;
			auxIndex=i;
		}
	}

	if(faces.recog_faces[auxIndex].face_centroid.y > 0.2)
		giro = 1; //cabeza a la izquierda
	else if(faces.recog_faces[auxIndex].face_centroid.y < -0.2)
		giro = 2; //cabeza a la derecha
	else
		giro= 0;

	return giro;
}

void facingOperator(int direction){
	if(direction == 0)
		JustinaManip::startHdGoTo(0.0, 0.0);
	else if(direction == 1)
		JustinaManip::startHdGoTo(0.2, 0.0);
	else if(direction == 2)
		JustinaManip::startHdGoTo(-0.2, 0.0);
}

vision_msgs::VisionFaceObjects recognizeFaces (float timeOut, bool &recognized){
	recognized = false;
	int previousSize = 20;
	int sameValue = 0;
	boost::posix_time::ptime curr;
	boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
	boost::posix_time::time_duration diff;
	vision_msgs::VisionFaceObjects lastRecognizedFaces;

	do
	{
		lastRecognizedFaces = JustinaVision::getFaces();
		
		if(lastRecognizedFaces.recog_faces.size() == previousSize && lastRecognizedFaces.recog_faces.size() > 0)
			sameValue ++;
		
		if (sameValue == 2)
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

bool listenAndAnswer(const int& timeout){
	std::string answer;
	std::string lastRecoSpeech;
	
	if(!JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeout))
	{
		std::cout << "no wait for"<<std::endl;
		JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
		return false;
	}
	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
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


bool listenTurnAndAnswer(const int& timeout){
	float audioSourceAngle = 0;
	std::string answer;
	std::string lastRecoSpeech;
	bool recogF = false;
	//almacena los rostros detectados por el servicio
  	vision_msgs::VisionFaceObjects dFaces;
	
	bool recogS = true;

	//JustinaManip::startHdGoTo(0.0, 0.0);
	//ros::Duration(1.0).sleep();

	//to set the input device KINECT
	JustinaHRI::setInputDevice(JustinaHRI::KINECT);
	JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
	
	if(!JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeout))
	{
		JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
		std::cout << "no wait for"<<std::endl;
		recogS = false;
	}
	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
	
	//convert string to lower case
	boost::to_lower(lastRecoSpeech);
	ros::Duration(1.0).sleep();
	audioSourceAngle = JustinaAudio::getAudioSource();
	std::cout << "Audio source at" << (180 * audioSourceAngle / 3.141592) << "degrees" << std::endl;
	JustinaHRI::say("Wait while I turn and look at you");
	ros::Duration(1.0).sleep();
	JustinaNavigation::moveDistAngle(0, (double) audioSourceAngle, 5000);
	ros::Duration(1.0).sleep();
	dFaces = recognizeFaces (2000,recogF);

	if(recogF)
	{
		int nF = nearestFace(dFaces);
		facingOperator(nF);
	}

	if(!recogS){
		//JustinaManip::startHdGoTo(0.0, 0.0);
		//ros::Duration(1.0).sleep();
		return false;
	}

	if(!JustinaKnowledge::comparePredQuestion(lastRecoSpeech,answer))
	{
		if(!JustinaRepresentation::answerQuestionFromKDB(lastRecoSpeech, answer, 500))
		{
			//JustinaManip::startHdGoTo(0.0, 0.0);
			//ros::Duration(1.0).sleep();
			std::cout << "no match with any question" << std::endl;
			return false; 
		}
	}

	JustinaHRI::say(answer);
	ros::Duration(2.0).sleep();
	
	return true; 
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

	JustinaHRI::loadGrammarSpeechRecognized("speechandperson.xml");//load the grammar
	JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
	JustinaRepresentation::initKDB("", true, 20000);

  	//ros::Rate loop(10);

	bool fail = false;
	bool success = false;

  	int nextState = 0;
  	bool recog=false;
  	int numQuestion = 1;
  	std::string answer;
	std::stringstream ss;
	
	int contChances=0;
	str1 = "/home/biorobotica/Script/stop_arecord.sh ";

	//load the predifined questions
  	JustinaKnowledge::getPredQuestions(questionList);

  	//set the KINECT as the input device 
  	JustinaHRI::setInputDevice(JustinaHRI::KINECT);

  	//alamcena los gestos detectados
  	std::vector<vision_msgs::GestureSkeleton> gestures;



  	while(ros::ok() && !fail && !success)
  	{
		ros::Rate loop(10);
  		switch(nextState)
    	{

    		case SM_InitialState:
    			std::cout << "start the audio source test" << std::endl;
        		JustinaHardware::setHeadGoalPose(0.0, 0.0);
        		JustinaHRI::say("I am ready for the audio source detection test");
        		ros::Duration(2.0).sleep();
        		nextState = SM_WaitBlindGame;
    		break;

    		case SM_WaitBlindGame:
				JustinaHRI::enableSpeechRecognized(false);
				JustinaHRI::say("I will give you a few seconds to move around me, please, wait for the next instruction");
				ros::Duration(3.0).sleep();
				JustinaHRI::playSound();
				ros::Duration(1.0).sleep();
				JustinaHRI::say("Ready, Please, tell me the first question now");
				//ros::Duration(1.5).sleep();

				//std::cout << system("/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/init_arecord.sh") << std::endl;
				JustinaHRI::initRecordAudio();
				
				JustinaAudio::startSimpleAudioSource();
				std::cout << "Starting audio source detection" << std::endl;
				ros::spinOnce();
				ros::Duration(1.0).sleep();
				nextState = SM_BlindGame;
			break;

			case SM_BlindGame:
				ss.str(std::string()); // Clear the buffer
				

				if(listenTurnAndAnswer(8000))
				{

					/*auxAudio.str("");
 					auxAudio.clear();
					auxAudio << "/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/stop_arecord.sh " << "Blind_"<<numQuestion;
					std::cout << system(auxAudio.str().c_str()) << std::endl;*/
					JustinaHRI::stopRecordAudio("Blind_", numQuestion);


					if(++numQuestion < 6)
					{
						ss << "Please, tell me the question number " << numQuestion << " now";
						nextState = SM_BlindGame;
						JustinaManip::startHdGoTo(0.0, 0.0);
						ros::Duration(1.0).sleep();
					}
					else
					{
						ss << "I will answer no more questions, Thank you";
						nextState = SM_FinalState;
					}
					ros::Duration(0.5).sleep();
					if(numQuestion < 6) 
						JustinaHRI::initRecordAudio();
						//std::cout << system("/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/init_arecord.sh") << std::endl;
					
				}
				else
				{
					ss << "I did not hear you, Please repeat the question ";
					ss << numQuestion;
					nextState = SM_BlindGameRepeatQ;
				}
		
				JustinaHRI::say(ss.str());				

				if (nextState == SM_BlindGameRepeatQ)
					JustinaHRI::enableSpeechRecognized(true);//enable recognized speech

				JustinaAudio::startSimpleAudioSource();
				ros::spinOnce();
				ros::Duration(1.0).sleep();
			break;

			case SM_BlindGameRepeatQ:
				ss.str(std::string()); // Clear the buffer
				if( !listenAndAnswer(8000) )
					ss << "I did not understand the question";
				
				/*auxAudio.str("");
				auxAudio.clear();
				auxAudio << "/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/stop_arecord.sh " << "Blind_"<<numQuestion;
				std::cout << system(auxAudio.str().c_str()) << std::endl;*/
				JustinaHRI::stopRecordAudio("Blind_", numQuestion);

				if(++numQuestion < 6)
				{
					ss << "Please, tell me the question number " << numQuestion << " now";
					nextState = SM_BlindGame;
					JustinaManip::startHdGoTo(0.0, 0.0);
					ros::Duration(1.0).sleep();
				}
				else
				{
					//ss << "I have finished the test";
					nextState = SM_FinalState;
				}

				JustinaHRI::say(ss.str());
				if(numQuestion < 6) 
					JustinaHRI::initRecordAudio();
					//std::cout << system("/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/init_arecord.sh") << std::endl;

				JustinaAudio::startSimpleAudioSource();
				ros::spinOnce();
				ros::Duration(1.0).sleep();
			break;


			case SM_FinalState:
				std::cout <<"finalState reached" << std::endl;
				JustinaHRI::say("I have finished the test");
				ros::Duration(2.0).sleep();
				success=true;
			break;
    	}
    	ros::spinOnce();
    	loop.sleep();
  	}
  	return 0;
}
