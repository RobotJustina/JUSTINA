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

JustinaHRI::Queue *tas;
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

std::stringstream auxFill;

int women=0;
int men=0;
int unknown=0;
int standing=0;
int sitting=0;
int lying=0;
int contCrowd=0;


//funcion para responder preguntas frente al robot

bool listenAndAnswer(const int& timeout)
{
	std::string answer;
	std::string lastRecoSpeech;
//	std::stringstream auxAudio;

	//to set the input device DEFUALT
	//JustinaHRI::setInputDevice(JustinaHRI::DEFUALT);
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

	//strcat(str1,lastRecoSpeech);
	//auxAudio << str1 << lastRecoSpeech<<".wav";
	

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

//función para reconocer los rostros que aparecen en una escena

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
		lastRecognizedFaces = JustinaVision::getFaces();
		
		if(lastRecognizedFaces.recog_faces.size() == previousSize && lastRecognizedFaces.recog_faces.size() > 0)
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
	
	
    JustinaTasks::getPanoramic(-0.2, -0.2, -0.6, -0.3, 0.3, 0.3, image, 30000);
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

	//JustinaRepresentation::initKDB("", true, 20000);

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
  	//JustinaKnowledge::getPredQuestions(questionList);

  	//set the KINECT as the input device 
  	//JustinaHRI::setInputDevice(JustinaHRI::KINECT);

  	//almacena los rstros detectados por el servicio
  	//vision_msgs::VisionFaceObjects dFaces;

//	JustinaHRI::Queue *tas;
	//std::string nombre;
	//char numero;
    //	if((tas = (JustinaHRI::Queue*)malloc(sizeof(JustinaHRI::Queue)))==NULL)
      //  	return -1;
    
	//if((nombre = (std::string*)malloc(sizeof(std::string)*50)) == NULL)
        //	return -1;

	/*JustinaHRI::inicializa();
	JustinaHRI::push( "hello my name is Justina, I meet another robot, its name is robbie");
	JustinaHRI::push( "my team is pumas, since 2011, i am very happy");
	JustinaHRI::push( "of the university of mexico");
	JustinaHRI::push( "I have two arms, how many arms do you have");*/
	//JustinaHRI::pop();
	
    ros::Time time;


  	while(ros::ok() && !fail && !success)
  	{
		ros::Rate rate(10);
  		switch(nextState)
    	{
		case SM_InitialState:
			//JustinaHRI::inicializa();
            std::cout << "INICIA TEST" << std::endl;
            time = ros::Time::now();
			JustinaHRI::insertAsyncSpeech( "i am ready for the speech and person recognition test", 2000, time.sec, 10);
			JustinaHRI::insertAsyncSpeech( "i want to play a ridle game", 2000, time.sec, 10);
			JustinaHRI::insertAsyncSpeech("i am turning arround to find you", 3000, time.sec, 20);
			JustinaHRI::insertAsyncSpeech("i am moving my head to find you", 2000, time.sec, 20);
	        JustinaHRI::insertAsyncSpeech( "hello my name is Justina, I meet another robot, its name is robbie", 3000, time.sec, 20);
            JustinaHRI::insertAsyncSpeech( "my team is pumas, since 2011, i am very happy", 2000, time.sec, 20);
	        JustinaHRI::insertAsyncSpeech( "of the university of mexico", 2000, time.sec, 20);
	        JustinaHRI::insertAsyncSpeech( "I have two arms, how many arms do you have", 2000, time.sec, 20);
			//JustinaHRI::asyncSpeech();
			JustinaHardware::setHeadGoalPose(0.0, 0.0);
			nextState = SM_WaitingandTurn;
		break;

    		case SM_WaitingandTurn:
		         JustinaTasks::sayAndSyncNavigateToLoc("kitchen", 120000);
			/*JustinaHRI::insertAsyncSpeech("i am turning arround to find you", 3000);
			JustinaHRI::insertAsyncSpeech("i am moving my head to find you", 2000);
			JustinaHRI::asyncSpeech();
        		JustinaNavigation::moveDistAngle(0.0, 3.141592, 5000);
        		ros::Duration(1.0).sleep();
			JustinaManip::startHdGoTo(0.0, -0.15);
        		ros::Duration(1.0).sleep();
			JustinaManip::startHdGoTo(0.0, -.9);
        		ros::Duration(1.0).sleep();
			JustinaManip::startHdGoTo(0.0, -0.15);
        		ros::Duration(1.0).sleep();*/
        		nextState = SM_InitialState;
      		break;
            case SM_StatingtheCrowd:
                std::cout << "Finish the Asyncrhonus speech test" << std::endl;
            break;
		
	}
		rate.sleep();
		ros::spinOnce();
	}
}
