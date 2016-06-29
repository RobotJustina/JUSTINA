#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "std_msgs/Bool.h"
#include "string"



#define SM_InitialState 0
#define	SM_WaitProfessional 10
#define SM_ASK_REPEAT_COMMAND 20
#define SM_PARSE_SPOKEN_COMMAND 30
#define	SM_TrainningPerson 40
#define	SM_MoveRobot 50
#define	SM_PersonRecognition 60
#define	SM_ReportResult 70
#define	SM_FinalState 80

bool personFound=false;
std::string personName = "operator";
float conf_val;

bool trainFace(std::string t_faceID, int t_timeout, int t_frames)
{  
    using namespace boost;
    boost::posix_time::ptime curr;
	boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
	boost::posix_time::time_duration diff;

	do{
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		JustinaVision::facTrain(t_faceID, t_frames);
		
		curr = boost::posix_time::second_clock::local_time();
		ros::spinOnce();
	}while(ros::ok() && (curr - prev).total_milliseconds() < t_timeout);
    
    return true;
}

float getAngle(int timeOut)
{
	float angle=100;
	boost::posix_time::ptime curr;
	boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
	boost::posix_time::time_duration diff;
	std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;
	do{
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		angle = JustinaVision::getAngleTC();
		//std::cout << "angle: " << angle << std::endl;
		curr = boost::posix_time::second_clock::local_time();
		ros::spinOnce();
	}while(ros::ok() && (curr - prev).total_milliseconds() < timeOut);
	return angle;
	
}

bool recognizePerTrain(float timeOut, std::string id)
{
		bool recognized = false;
		boost::posix_time::ptime curr;
		boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
		boost::posix_time::time_duration diff;
		std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;
		do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			JustinaVision::facRecognize(id);
			curr = boost::posix_time::second_clock::local_time();
			ros::spinOnce();
		}while(ros::ok() && (curr - prev).total_milliseconds() < timeOut);

		return true;
}

std::vector<vision_msgs::VisionFaceObject> recognizeAllFaces(float timeOut, bool &recognized)
{
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
    std::cout << "INITIALIZING ACT_PLN-FOLLOW ME BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    ros::Rate loop(10);

    int c_point=0;
    int nextState = 0;
    bool fail = false;
    bool success = false;
    bool stop=false;
    bool recog=false;
    bool aux_findP=false;
    int giro=0;

    float gPan=0.0;
	float gTilt=0.0;
	
	std::stringstream contW;
	std::stringstream contM;
	std::stringstream contU;
	std::stringstream profPlace;
	std::stringstream genderOperator;
	std::stringstream contC;
	int mIndex=0;
	int women=0;
	int men=0;
	int unknown=0;
	int genero=10;
	int contCrowd=0;
	
	int cont=0;
	int cont_sP=0;
	
	int c_right=0;
	int c_left=0;
	float angle_robot=10.0;
	std::vector<vision_msgs::VisionFaceObject> dFaces;

    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    validCommands.push_back("start");
    validCommands.push_back("robot start");
    float timeOutSpeech = 2000;

    

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {

        case SM_InitialState:
        	std::cout << "executing initial state" << std::endl;
        	
			//JustinaHRI::say("I am going to start the person recognition test...");
			JustinaVision::facClearByID(personName);
			JustinaHardware::setHeadGoalPose(0.0, 0.0);
			JustinaHRI::say("Hello, my name is Justina. I am going to start the person recognition test...");
            nextState = SM_WaitProfessional;

        break;

        case SM_WaitProfessional:
        	//std::cout << "angle robot "<< angle_robot<<std::endl;
        	//JustinaVision::stopThermalCamera();
        	std::cout << "waiting for the professional.." << std::endl;
			//detectar cuando aparece el profesional frente al robot
			//esto se realizaría con el sistema de Carlos utilizando la cámara térmica
			std::cout << "meeting human..." << std::endl;
			nextState = SM_TrainningPerson;

        	/*std::cout << "Waiting Speech" << std::endl;
            if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, timeOutSpeech))
               	nextState = SM_ASK_REPEAT_COMMAND;
            else
               	nextState = SM_PARSE_SPOKEN_COMMAND;*/

        break;

        /*case SM_ASK_REPEAT_COMMAND:
            JustinaHRI::say("Please repeat the command...");
   			nextState = SM_WaitProfessional;

		break;

		case SM_PARSE_SPOKEN_COMMAND:
			std::cout << "Starting test" << std::endl;
          	if(lastRecoSpeech.find("start") != std::string::npos || lastRecoSpeech.find("robot start") != std::string::npos)
				nextState = SM_TrainningPerson;
	    break;*/

        case SM_TrainningPerson:

        	
			JustinaVision::startFaceRecognition();

			if(cont>2)
			{
				JustinaHRI::say("I could not memorize your face, I am aborting the test");
				ros::Duration(2.0).sleep();
				JustinaHardware::setHeadGoalPose(0.0, 0.0);
				JustinaVision::stopFaceRecognition();
				
				nextState = SM_FinalState;
				break;
			}

			JustinaHRI::say("I will memorize your face, Please look straight to my kinect camera");
			std::cout << "I will remember your face, Please look straight to my kinect camera" <<std::endl;
			ros::Duration(1.0).sleep();
	
			
			if(trainFace(personName, 40000, 20))//train person
			{
				JustinaHRI::say("I have memorized your face, now you can place into the crowd");
				std::cout << "I have remembered your face" <<std::endl;
				JustinaVision::stopFaceRecognition();
				ros::Duration(10.0).sleep();
				nextState = SM_MoveRobot;

			}
			
			else
			{
				JustinaHRI::say("I could not memorized your face, I will try it again");
				std::cout << "I could not remember your face, I will try it again" <<std::endl;
				cont ++;
				nextState = SM_TrainningPerson;
			}	
		
			
			
        break;

        case SM_MoveRobot:
        	ros::Duration(1.0).sleep();
        	std::cout << "finding the crowd" << std::endl;
        	JustinaHardware::setHeadGoalPose(0.0, 0.0);
        	JustinaNavigation::moveDistAngle(0.0, 3.141592, 80000);
        	ros::Duration(1.0).sleep();

        	/*while((angle_robot == -10.0 || angle_robot==10)){
        		angle_robot = getAngle(3000);
        		if ((angle_robot == -10.0 || angle_robot==10)){
        			ros::Duration(1.0).sleep();
        			JustinaNavigation::moveDistAngle(0.0, 0.4, 80000);
        			angle_robot = getAngle(3000);
        			giro=1;
        		}
        		if ((angle_robot == -10.0 || angle_robot==10) && giro==1){
        			ros::Duration(1.0).sleep();
        			JustinaNavigation::moveDistAngle(0.0, -0.4, 80000);
        			angle_robot = getAngle(3000);
        			giro=2;
        		}
        		if((angle_robot == -10.0 || angle_robot==10) && giro==2){
        			ros::Duration(1.0).sleep();
        			JustinaHRI::say("I could not find the crow, I am aborting the test..");
        			nextState = SM_FinalState;
        			break;
        		}
			}

        	ros::Duration(1.0).sleep();
        	JustinaNavigation::moveDistAngle(0.0, angle_robot, 80000);*/
        	JustinaNavigation::moveDistAngle(0.8, 0.0, 80000);
        	ros::Duration(1.0).sleep();
        	

			std::cout << "I am looking for the professional into the crowd " <<std::endl;
	
			nextState = SM_PersonRecognition;
            
        break;

        case SM_PersonRecognition:
									
       	 	conf_val=0.7;
       	 	
	
			if(cont_sP==1){
				gPan=-0.4;
				mIndex=0;
				women=0;
				men=0;
				unknown=0;
				recog=false;
				aux_findP=false;
			}
			else if(cont_sP==2){
				gPan=0.4;
				mIndex=0;
				women=0;
				men=0;
				unknown=0;
				recog=false;
				aux_findP=false;
			}
			else if (cont_sP==3){
				mIndex=0;
				women=0;
				men=0;
				unknown=0;
				gPan=0.0;
				recog=false;
				aux_findP=false;
				JustinaNavigation::moveDistAngle(-0.8, 0.0, 80000);
			}
			else if(cont_sP>3)
			{
				JustinaVision::stopFaceRecognition();
				nextState = SM_ReportResult;
				break;
			}

			std::cout << "con_sp: "<<cont_sP<<std::endl;
			
			
			JustinaHardware::setHeadGoalPose(gPan, gTilt);
			

			JustinaHRI::say(" I am looking for the operator into the crowd ...");
			JustinaHRI::say(" Please do not move until I have had finished the test ...");
			std::cout <<"I am looking for the professional into the crowd" << std::endl;
			
			while(!recog)
			{
				dFaces = recognizeAllFaces(10000,recog);
				JustinaVision::stopFaceRecognition();
			}
	
			
			std::cout <<"tamaño de arreglo " << dFaces.size() <<std::endl;

			for(int i=0; i<dFaces.size(); i++)
			{
				if(dFaces[i].id==personName && dFaces[i].confidence>=conf_val)	
				{
					conf_val=dFaces[i].confidence;
					mIndex=i;
					personFound=true;
					
					std::cout << "indice de la persona " << mIndex << std::endl; 
					std::cout << "valor de confianza " << conf_val << std::endl; 
				}

				genero=dFaces[mIndex].gender;

				
				if(dFaces[i].gender==0)
					women++;
				if(dFaces[i].gender==1)
					men++;
				if(dFaces[i].gender==2)
					unknown++;
	
				if(genero==0)
					genderOperator << "and I think that you are a women";
				if(genero==1)
					genderOperator << "and I think that you are a men";
				if(genero==2)
					genderOperator << "Sorry, but I cannot define your genre";

				std::cout<<"hombres: "<< men << std::endl;

			}

			if(personFound){
				while(!aux_findP)
				{
					JustinaVision::startFaceRecognition();
					aux_findP=recognizePerTrain(10000, personName);
					JustinaVision::stopFaceRecognition();
				}
				
				nextState = SM_ReportResult;
			}
			else
			{	
				JustinaVision::startFaceRecognition();
				cont_sP++;
				nextState = SM_PersonRecognition;
			}
        break;


        case SM_ReportResult:

        	std::cout <<"Reporting results" << std::endl;
		
		
			mIndex = mIndex + 1;
			c_left = dFaces.size() - mIndex;
			c_right = mIndex - 1;
			contCrowd=women+men+unknown;
			contC << "the size of the crowd is " <<contCrowd << std::endl;

	
			contW << "There are " << women << " women";
			contM << "There are " << men << " men";
			contU << "There are " << unknown << " people with unknown genre";

			profPlace << "I have found you " << "There are " << c_left << " people to your left and " << c_right << " people to your right ";
	

			if(personFound && dFaces.size()==mIndex){
				JustinaHRI::say("I have found you. You are the right most person into the crowd...");
				JustinaHRI::say(genderOperator.str());
			}
			if(personFound && mIndex==1){
				JustinaHRI::say("I have found you. You are the left most person into the crowd...");
				JustinaHRI::say(genderOperator.str());
			}

			if(personFound && mIndex!=1 && dFaces.size()!=mIndex){
				JustinaHRI::say(profPlace.str());
				JustinaHRI::say(genderOperator.str());
			}
			if(!personFound)
				JustinaHRI::say("I could not find you");

			JustinaHRI::say("I am going to describe the crowd ");
			JustinaHRI::say(contC.str());
			JustinaHRI::say(contW.str());
			JustinaHRI::say(contM.str());
			JustinaHRI::say(contU.str());
	
			ros::Duration(4.0).sleep();
			//save results on PDF
			JustinaTools::pdfImageExport("PersonRecognitionTest","/home/$USER/faces/");
            nextState = SM_FinalState;
        break;

        case SM_FinalState:
            std::cout <<"finalState reached" << std::endl;
			JustinaHRI::say("I have finished the person recognition test...");
			success=true;
        break;

        }

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
