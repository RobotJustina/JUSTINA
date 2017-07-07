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
std::vector<std::string> malesVec;
std::vector<std::string> menVec;
std::vector<std::string> boysVec;
std::vector<std::string> femalesVec;
std::vector<std::string> womenVec;
std::vector<std::string> girlsVec;
std::vector<std::string> standingVec;
std::vector<std::string> sittingVec;
std::vector<std::string> lyingVec;
std::vector<std::string> personVec1;
std::vector<std::string> personVec2;
std::vector<std::string> personVec3;
std::vector<std::string> crowdVec;
std::vector<std::string> eldersVec;
std::vector<std::string> adultsVec;
std::vector<std::string> childrenVec;

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
	/*JustinaNavigation::moveDistAngle(0.5, 0.0, 80000);
    ros::Duration(2.0).sleep();
	JustinaManip::startHdGoTo(-0.4, -0.15);
	ros::Duration(3.0).sleep();
	JustinaManip::startHdGoTo(0.0, -0.15);
	ros::Duration(3.0).sleep();
	JustinaManip::startHdGoTo(0.4, -0.15);
	ros::Duration(3.0).sleep();
	JustinaManip::startHdGoTo(0.0, 0.0);
	ros::Duration(3.0).sleep();*/
	sensor_msgs::Image image;
    JustinaTasks::getPanoramic(-0.2, -0.2, -0.6, -0.3, 0.3, 0.3, image, 30000);
    panoramicFaces = JustinaVision::getRecogFromPano(image);	
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
	//bool PredQ;
	//bool KDBQ;
	//JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
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

bool listenTurnAndAnswer(const int& timeout)
{
	float audioSourceAngle = 0;
	std::string answer;
	std::string lastRecoSpeech;
	//bool PredQ;
	//bool KDBQ;
	bool recogS = true;

	JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
	std::cout << "Starting audio source detection" << std::endl;
	/*JustinaAudio::startSimpleAudioSource();
	ros::spinOnce();
	ros::Duration(1.0).sleep();*/

	if(!JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeout))
	{
		JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
		std::cout << "no wait for"<<std::endl;
		recogS = false;
		//return false;
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

	if(!recogS)
		return false;

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
	std::stringstream auxFill;

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
	vision_msgs::VisionFaceObjects panoramicFaces;
	//load the predifined questions
  	JustinaKnowledge::getPredQuestions(questionList);

	//int sleepAudioCaptureDelay = 4;


  	while(ros::ok() && !fail && !success)
  	{
		//ros::Rate loop(sleepAudioCaptureDelay);
		ros::Rate loop(10);
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
        		JustinaHRI::say("I am turnning around to find you");
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

				//new stuff
				/*sensor_msgs::Image image;
        		JustinaTasks::getPanoramic(-0.2, -0.2, -0.6, -0.3, 0.3, 0.3, image, 30000);
        		panoramicFaces = JustinaVision::getRecogFromPano(image);*/

				for(int i=0; i<dFaces.size(); i++)
				{
					auxFill << "usuario_" << i;
					personVec1.push_back(auxFill.str());
					personVec2.push_back(auxFill.str());
					personVec3.push_back(auxFill.str());


					if(dFaces[i].face_centroid.z < 0.8){
						lying++;
						personVec1.push_back("lying");
						personVec2.push_back("lying");
						personVec3.push_back("lying");	
					}
					if(dFaces[i].face_centroid.z >= 0.8 && dFaces[i].face_centroid.z <1.20){
						sitting++;
						personVec1.push_back("sitting");
						personVec2.push_back("sitting");
						personVec3.push_back("sitting");	
					}
					if(dFaces[i].face_centroid.z >= 1.20){
						standing++;
						personVec1.push_back("standing");
						personVec2.push_back("standing");
						personVec3.push_back("standing");
					}
					if(dFaces[i].gender==0){
						women++;
						personVec1.push_back("female");
						personVec2.push_back("woman");
						personVec3.push_back("girl");
					}
					if(dFaces[i].gender==1){
						men++;
						personVec1.push_back("male");
						personVec2.push_back("man");
						personVec3.push_back("boy");
					}
					if(dFaces[i].gender==2)
						unknown++;	

					JustinaRepresentation::insertKDB("cmd_set_prsn", personVec1, 500);
					JustinaRepresentation::insertKDB("cmd_set_prsn", personVec2, 500);
					JustinaRepresentation::insertKDB("cmd_set_prsn", personVec3, 500);

					auxFill.str(std::string()); // Clear the buffer
					personVec1.clear();
					personVec2.clear();
					personVec3.clear();

				}

				std::cout <<"Reporting results" << std::endl;

				contCrowd=women+men+unknown;
				contC << "the size of the crowd is " <<contCrowd << std::endl;
				contFake << "i think there are " << contCrowd << " people in the scene, i will verify it";
				contW << "There are " << women << " women";
				contM << "There are " << men << " men";
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
				confirmSizeCrowd();
				/*JustinaHRI::say(contFake.str());
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
				JustinaHRI::say("I have verified the information");
				ros::Duration(1.0).sleep();
				JustinaHRI::say("I am going to describe the crowd");
				ros::Duration(1.0).sleep();
				JustinaHRI::say(contC.str());
				ros::Duration(1.0).sleep();
				JustinaHRI::say(contW.str());
				ros::Duration(1.0).sleep();
				JustinaHRI::say(contM.str());
				ros::Duration(1.0).sleep();*/
				std::cout<<"standing: "<< standing << std::endl;
				std::cout<<"sitting: "<< sitting << std::endl;
				std::cout<<"lying: "<< lying << std::endl;
				ros::Duration(1.0).sleep();

				//fill the information en KDB
				//information gender males
				auxFill << men;
				malesVec.push_back("males");
				malesVec.push_back(auxFill.str()); 
				JustinaRepresentation::insertKDB("cmd_set_gender_q", malesVec, 500);

				
				menVec.push_back("men");
				menVec.push_back(auxFill.str()); 
				JustinaRepresentation::insertKDB("cmd_set_gender_q", menVec, 500);

				
				boysVec.push_back("boys");
				boysVec.push_back(auxFill.str()); 
				JustinaRepresentation::insertKDB("cmd_set_gender_q", boysVec, 500);

				auxFill.str(std::string()); // Clear the buffer

				//information gender females
				auxFill << women;
				
				femalesVec.push_back("females");
				femalesVec.push_back(auxFill.str()); 
				JustinaRepresentation::insertKDB("cmd_set_gender_q", femalesVec, 500);

				
				womenVec.push_back("women");
				womenVec.push_back(auxFill.str()); 
				JustinaRepresentation::insertKDB("cmd_set_gender_q", womenVec, 500);

				
				girlsVec.push_back("girls");
				girlsVec.push_back(auxFill.str()); 
				JustinaRepresentation::insertKDB("cmd_set_gender_q", girlsVec, 500);

				auxFill.str(std::string()); // Clear the buffer

				//information elders'  and adults number
				auxFill << contCrowd;

				eldersVec.push_back("elders");
				eldersVec.push_back(auxFill.str());
				JustinaRepresentation::insertKDB("cmd_set_gender_q", eldersVec, 500);

				adultsVec.push_back("adults");
				adultsVec.push_back(auxFill.str());
				JustinaRepresentation::insertKDB("cmd_set_gender_q", adultsVec, 500);
				auxFill.str(std::string()); //clear the buffer

				//information children number
				childrenVec.push_back("children");
				childrenVec.push_back("0");
				JustinaRepresentation::insertKDB("cmd_set_gender_q", childrenVec, 500);
				auxFill.str(std::string()); //clear the buffer

				//information poses standing
				auxFill << standing;
				
				standingVec.push_back("standing");
				standingVec.push_back(auxFill.str()); 
				JustinaRepresentation::insertKDB("cmd_set_pose_q", standingVec, 500);

				auxFill.str(std::string()); // Clear the buffer

				//information poses sitting
				auxFill << sitting;
				
				sittingVec.push_back("sitting");
				sittingVec.push_back(auxFill.str()); 
				JustinaRepresentation::insertKDB("cmd_set_pose_q", sittingVec, 500);

				auxFill.str(std::string()); // Clear the buffer

				//information poses lying
				auxFill << lying;

				lyingVec.push_back("lying");
				lyingVec.push_back(auxFill.str());
				JustinaRepresentation::insertKDB("cmd_set_pose_q", lyingVec, 500);
				auxFill.str(std::string()); // Clear the buffer

				//information total people
				auxFill << contCrowd;
				crowdVec.push_back(auxFill.str());
				JustinaRepresentation::insertKDB("cmd_set_total_q", crowdVec, 500);
				auxFill.str(std::string()); // Clear the buffer

				nextState = SM_RequestingOperator;
      		break;

      		case SM_RequestingOperator:
				std::cout <<"Requesting Operator" << std::endl;
				JustinaHRI::say("Who want to play riddles with me");
				ros::Duration(1.5).sleep();
				JustinaHRI::say("Please, put in front of me");
				ros::Duration(1.5).sleep();
				JustinaHRI::say("Please, tell me the first question now");
				JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
				ros::Duration(1.0).sleep();
        		nextState = SM_RiddleGame;
      		break;

      		case SM_RiddleGame:
				//ros::Duration(1.0).sleep();
				ss.str(std::string()); // Clear the buffer
				if( !listenAndAnswer(10000))
					ss << "I did not understand the question";
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
				//ss << ".";
				JustinaHRI::say(ss.str());
				JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
				ros::Duration(1.0).sleep();
			break;

			case SM_WaitBlindGame:
				JustinaHRI::enableSpeechRecognized(false);
				JustinaHRI::say("I will give you a few seconds to move around me, please, wait for the next instruction");
				ros::Duration(9.0).sleep();
				JustinaHRI::playSound();
				ros::Duration(1.0).sleep();
				JustinaHRI::say("Ready, Please, tell me the first question now");
				//ros::Duration(1.5).sleep();
				JustinaAudio::startSimpleAudioSource();
				ros::spinOnce();
				ros::Duration(1.0).sleep();
				nextState = SM_BlindGame;
			break;

			case SM_BlindGame:
				ss.str(std::string()); // Clear the buffer
				if(listenTurnAndAnswer(8000))
				{
					if(++numQuestion < 6)
					{
						ss << "Please, tell me the question number " << numQuestion << " now";
						nextState = SM_BlindGame;
					}
					else
					{
						ss << "I will answer no more questions, Thank you";
						nextState = SM_FinalState;
					}
				}
				else
				{
					ss << "I did not hear you, Please repeat the question ";
					ss << numQuestion;
					nextState = SM_BlindGameRepeatQ;
				}
				JustinaHRI::say(ss.str());
				//ros::Duration(2.0).sleep();
				//sleepAudioCaptureDelay = 4;
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
				//ss << ".";
				JustinaHRI::say(ss.str());
				JustinaAudio::startSimpleAudioSource();
				ros::spinOnce();
				ros::Duration(1.0).sleep();
			break;


			case SM_FinalState:
				//save results on PDF
				JustinaTools::pdfImageExport("SpeechAndPersonRecognitionTest","/home/$USER/faces/");
				std::cout <<"finalState reached" << std::endl;
				JustinaHRI::say("I have finished the speech and person recognition test");
				ros::Duration(2.0).sleep();
				success=true;
			break;

    }
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
