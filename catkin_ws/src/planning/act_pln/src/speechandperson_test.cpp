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
std::vector<std::string> pointingLVec;
std::vector<std::string> pointingRVec;
std::vector<std::string> risingLVec;
std::vector<std::string> risingRVec;
std::vector<std::string> wavingVec;

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

std::stringstream auxFill;

int women=0;
int men=0;
int unknown=0;
int standing=0;
int sitting=0;
int lying=0;
int pointing_left=0;
int pointing_right=0;
int waving=0;
int rising_left_arm=0;
int rising_right_arm=0;
int contCrowd=0;

bool moveHead=true;


//función para reconocer los rostros que aparecen en una escena

vision_msgs::VisionFaceObjects recognizeFaces (float timeOut, int attempts, bool &recognized)
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
		lastRecognizedFaces = JustinaVision::getFaces();
		
		if(lastRecognizedFaces.recog_faces.size() == previousSize && lastRecognizedFaces.recog_faces.size() > 0)
			sameValue ++;
		
		if (sameValue == attempts)
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

//funcion para responder preguntas frente al robot

bool listenAndAnswer(const int& timeout)
{
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


//función para responder una pregunta, previamente reconociendo la fuente de sonido

bool listenTurnAndAnswer(const int& timeout)
{
	float audioSourceAngle = 0;
	std::string answer;
	std::string lastRecoSpeech;
	bool recogF = false;
	//almacena los rostros detectados por el servicio
  	vision_msgs::VisionFaceObjects faces;
	
	bool recogS = true;

	//to set the input device KINECT
	//JustinaHRI::setInputDevice(JustinaHRI::KINECT);
	JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
	
	//std::cout << "Starting audio source detection" << std::endl;
	/*JustinaAudio::startSimpleAudioSource();
	ros::spinOnce();
	ros::Duration(1.0).sleep();*/

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

	if(moveHead){
		faces = recognizeFaces (2000, 2, recogF);

		if(recogF)
		{
			int nF = nearestFace(faces);
			facingOperator(nF);
		}
	}

	

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



//función para llenar la KDB con la información de la pose de las personas de la multitud

void setPoseCrowdInKDB(vision_msgs::VisionFaceObjects faces)
{
	for(int i=0; i<faces.recog_faces.size(); i++)
	{
		auxFill << "usuario_" << i;
		personVec1.push_back(auxFill.str());
		personVec2.push_back(auxFill.str());
		personVec3.push_back(auxFill.str());


		if(faces.recog_faces[i].face_centroid.z < 0.8){
			lying++;
			personVec1.push_back("lying");
			personVec2.push_back("lying");
			personVec3.push_back("lying");	
		}
		if(faces.recog_faces[i].face_centroid.z >= 0.8 && faces.recog_faces[i].face_centroid.z <1.20){
			sitting++;
			personVec1.push_back("sitting");
			personVec2.push_back("sitting");
			personVec3.push_back("sitting");	
		}
		if(faces.recog_faces[i].face_centroid.z >= 1.20){
			standing++;
			personVec1.push_back("standing");
			personVec2.push_back("standing");
			personVec3.push_back("standing");
		}
		if(faces.recog_faces[i].gender==0){
			//women++;
			personVec1.push_back("female");
			personVec2.push_back("woman");
			personVec3.push_back("girl");
		}
		if(faces.recog_faces[i].gender==1){
			//men++;
			personVec1.push_back("male");
			personVec2.push_back("man");
			personVec3.push_back("boy");
		}
					
		JustinaRepresentation::insertKDB("cmd_set_prsn", personVec1, 500);
		JustinaRepresentation::insertKDB("cmd_set_prsn", personVec2, 500);
		JustinaRepresentation::insertKDB("cmd_set_prsn", personVec3, 500);

		auxFill.str(std::string()); // Clear the buffer
		personVec1.clear();
		personVec2.clear();
		personVec3.clear();
	}
}

void setGestureCrowdInKDB(std::vector<vision_msgs::GestureSkeleton> gestures)
{
	//JustinaVision::lastGestureRecog.clear();
	auxFill.str(std::string()); // Clear the buffer

	for(int i=0; i<gestures.size(); i++)
	{

		if(gestures[i].gesture == "pointing_right" && gestures[i].gesture_centroid.y < 2.0)
			pointing_right++;
		else if(gestures[i].gesture == "pointing_left" && gestures[i].gesture_centroid.y < 2.0)
			pointing_left++;
		else if(gestures[i].gesture == "right_hand_rised" && gestures[i].gesture_centroid.y < 2.0)
			rising_right_arm++;
		else if(gestures[i].gesture == "left_hand_rised" && gestures[i].gesture_centroid.y < 2.0) 
			rising_left_arm++;
		else
			waving++;
	}

	//information pointing right gesture
	auxFill << pointing_right;
	pointingRVec.push_back("pointing_to_the_right");
	pointingRVec.push_back(auxFill.str()); 
	JustinaRepresentation::insertKDB("cmd_set_gesture_q", pointingRVec, 500);

	auxFill.str(std::string()); // Clear the buffer

	//information pointing left gesture
	auxFill << pointing_left;
	pointingLVec.push_back("pointing_to_the_left");
	pointingLVec.push_back(auxFill.str());
	JustinaRepresentation::insertKDB("cmd_set_gesture_q", pointingLVec, 500);

	auxFill.str(std::string()); // Clear the buffer

	//information rising left arm
	auxFill << rising_left_arm;
	risingLVec.push_back("raising_their_left_arm");
	risingLVec.push_back(auxFill.str()); 
	JustinaRepresentation::insertKDB("cmd_set_gesture_q", risingLVec, 500);

	auxFill.str(std::string()); // Clear the buffer

	//information rising right arm
	auxFill << rising_right_arm;
	risingRVec.push_back("raising_their_right_arm");
	risingRVec.push_back(auxFill.str()); 
	JustinaRepresentation::insertKDB("cmd_set_gesture_q", risingRVec, 500);

	auxFill.str(std::string()); // Clear the buffer

	//information rising right arm
	auxFill << waving;
	wavingVec.push_back("waving");
	wavingVec.push_back(auxFill.str()); 
	JustinaRepresentation::insertKDB("cmd_set_gesture_q", wavingVec, 500);

	auxFill.str(std::string()); // Clear the buffer

}


//función para llenar la KDB con la informacion sobre el genero de las personas de la multitud

void setGenderCrowdInKDB()
{
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

	//information elders number
	eldersVec.push_back("elders");
	eldersVec.push_back("0");
	JustinaRepresentation::insertKDB("cmd_set_gender_q", eldersVec, 500);
	//information adults number
	auxFill << contCrowd;

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
}

//función para confirmar el tamaño de la multitud mediante una foto panoramica

void confirmSizeCrowd(vision_msgs::VisionFaceObjects faces)
{
	vision_msgs::VisionFaceObjects panoramicFaces;
	sensor_msgs::Image image;
	
	JustinaHRI::say(contFake.str());
	ros::Duration(2.5).sleep();
	JustinaNavigation::moveDistAngle(0.5, 0.0, 5000);
    ros::Duration(2.0).sleep();
	
	
    JustinaTasks::getPanoramic(-0.2, -0.3, -0.5, -0.3, 0.3, 0.3, image, 30000);
    panoramicFaces = JustinaVision::getRecogFromPano(image);
    ros::Duration(3.0).sleep();
	JustinaManip::startHdGoTo(0.0, 0.0);
	ros::Duration(3.0).sleep();	

	if(panoramicFaces.recog_faces.size() >= faces.recog_faces.size()){

		contCrowd = panoramicFaces.recog_faces.size();
		contC << "the size of the crowd is " << contCrowd << std::endl;
		for(int i=0; i<panoramicFaces.recog_faces.size(); i++)
		{
			if(panoramicFaces.recog_faces[i].gender==0)
				women++;
			
			if(panoramicFaces.recog_faces[i].gender==1)
				men++;	
		}
	}
	else
	{
		contCrowd = faces.recog_faces.size();
		contC << "the size of the crowd is " << contCrowd << std::endl;
		for(int i=0; i<faces.recog_faces.size(); i++)
		{
			if(faces.recog_faces[i].gender==0)
				women++;
			
			if(faces.recog_faces[i].gender==1)
				men++;	
		}
	}

	contW << "There are " << women << " women";
	contM << "There are " << men << " men";
	//contC << "the size of the crowd is " << panoramicFaces.recog_faces.size() << std::endl;
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
	
	int contChances=0;
	str1 = "/home/biorobotica/Script/stop_arecord.sh ";

	//vector para almacenar los rostros encontrados
	//std::vector<vision_msgs::VisionFaceObject> dFaces;

	//load the predifined questions
  	JustinaKnowledge::getPredQuestions(questionList);

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
      			std::cout << "start the speech and person recognition test" << std::endl;
        		JustinaManip::startHdGoTo(0.0, 0.0);
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
        		JustinaNavigation::moveDistAngle(0.0, 3.141592, 5000);
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
					dFaces = recognizeFaces (10000, 3, recog);
					JustinaVision::startFaceRecognition(false);
					contChances++;
				}

				//std::cout <<"tamaño de arreglo " << dFaces.size() <<std::endl;
				std::cout <<"tamaño de arreglo " << dFaces.recog_faces.size() <<std::endl;
				//fill the KDB with the pose crowd
				setPoseCrowdInKDB(dFaces);				

				
				std::cout <<"Reporting results" << std::endl;

				//contFake << "i think there are " << dFaces.size() << " people in the scene, please do not move, i will verify it";
				contFake << "i think there are " << dFaces.recog_faces.size() << " people in the scene, please do not move, i will check up";

				
				if(dFaces.recog_faces.size()==0)
				{
					JustinaHRI::say("Sorry, I cannot state the size of the crowd, lets proceed with the test");
					ros::Duration(1.5).sleep();
					nextState = SM_RequestingOperator;
	      			break;
				}

                JustinaVision::startSkeletonFinding();
                ros::Duration(2.0).sleep();
				if(JustinaTasks::waitRecognizedGesture(gestures, 2000.0)){
					setGestureCrowdInKDB(gestures);
					ros::Duration(1.0).sleep();
					std::cout << "Gestures detected: " << gestures.size() << std::endl;
				}
				else {
					std::cout << "Cannot get gestures..." << std::endl;
					ros::Duration(1.0).sleep();
				}

                JustinaVision::stopSkeletonFinding();

				JustinaManip::startHdGoTo(0.0, 0.0);
				ros::Duration(1.0).sleep();

				//confirm with the photo panoramic 
				confirmSizeCrowd(dFaces);
				
				std::cout<<"standing: "<< standing << std::endl;
				std::cout<<"sitting: "<< sitting << std::endl;
				std::cout<<"lying: "<< lying << std::endl;
				ros::Duration(1.0).sleep();

				//fill the information in KDB
				setGenderCrowdInKDB();

				nextState = SM_RequestingOperator;
      		break;

      		case SM_RequestingOperator:
				std::cout <<"Requesting Operator" << std::endl;
				JustinaHRI::say("Who want to play riddles with me");
				ros::Duration(1.5).sleep();
				JustinaHRI::say("Please, put in front of me");
				ros::Duration(1.5).sleep();
				JustinaHRI::say("Please, tell me the first question now");
				std::cout << system("/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/init_arecord.sh") << std::endl;
				JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
				ros::Duration(1.0).sleep();
        		nextState = SM_RiddleGame;
      		break;

      		case SM_RiddleGame:
				//ros::Duration(1.0).sleep();
				ss.str(std::string()); // Clear the buffer
				//std::cout << system("/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/init_arecord.sh") << std::endl;

				if( !listenAndAnswer(10000))
					ss << "I did not understand the question";
				auxAudio.str("");
 				auxAudio.clear();
				auxAudio << "/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/stop_arecord.sh " << "Riddle_"<<numQuestion;
				std::cout << system(auxAudio.str().c_str()) << std::endl;

				if(++numQuestion < 6)
				{
					ss << "Please, tell me the question number " << numQuestion << " now";
					nextState = SM_RiddleGame;
				}
				else
				{
					ss << "Lets proceed with the blind mans bluff game";
					//numQuestion = 1;
					nextState = SM_WaitBlindGame;
				}

				JustinaHRI::say(ss.str());
				if(numQuestion < 6){
					std::cout << system("/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/init_arecord.sh") << std::endl;
				}
				else{
					numQuestion = 1;
				}

				JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
				ros::Duration(1.0).sleep();
			break;

			case SM_WaitBlindGame:
				JustinaHRI::enableSpeechRecognized(false);
				//set the KINECT as the input device 
  				JustinaHRI::setInputDevice(JustinaHRI::KINECT);
				JustinaHRI::say("I will give you a few seconds to move around me, please, wait for the next instruction");
				ros::Duration(9.0).sleep();
				JustinaHRI::playSound();
				ros::Duration(1.0).sleep();
				JustinaHRI::say("Ready, Please, tell me the first question now");
				//ros::Duration(1.5).sleep();

				std::cout << system("/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/init_arecord.sh") << std::endl;
				
				JustinaAudio::startSimpleAudioSource();
				std::cout << "Starting audio source detection" << std::endl;
				ros::spinOnce();
				ros::Duration(1.0).sleep();
				nextState = SM_BlindGame;
			break;

			case SM_BlindGame:
				ss.str(std::string()); // Clear the buffer
				//JustinaManip::startHdGoTo(0.0, 0.0);
				//ros::Duration(1.0).sleep();

				if(listenTurnAndAnswer(8000))
				{

					auxAudio.str("");
 					auxAudio.clear();
					auxAudio << "/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/stop_arecord.sh " << "Blind_"<<numQuestion;
					std::cout << system(auxAudio.str().c_str()) << std::endl;


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
						std::cout << system("/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/init_arecord.sh") << std::endl;


										
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
				
				auxAudio.str("");
				auxAudio.clear();
				auxAudio << "/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/stop_arecord.sh " << "Blind_"<<numQuestion;
				std::cout << system(auxAudio.str().c_str()) << std::endl;

				if(++numQuestion < 6)
				{
					ss << "Please, tell me the question number " << numQuestion << " now";
					nextState = SM_BlindGame;
					JustinaManip::startHdGoTo(0.0, 0.0);
					ros::Duration(1.0).sleep();
				}
				else
				{
					ss << "I have finished the test";
					nextState = SM_FinalState;
				}

				JustinaHRI::say(ss.str());
				if(numQuestion < 6) 
					std::cout << system("/home/biorobotica/JUSTINA/catkin_ws/src/tools/justina_tools/src/init_arecord.sh") << std::endl;

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
