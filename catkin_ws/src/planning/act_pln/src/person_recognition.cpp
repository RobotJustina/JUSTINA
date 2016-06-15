#include <iostream>
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
#define	SM_MeetHuman 20
#define	SM_MoveRobot 30
#define	SM_PersonRecognition 40
#define	SM_ReportResult 50
#define	SM_FinalState 60

bool trainFace(std::string t_faceID, int t_timeout, int t_frames)
{  
	bool faceTrained = false;   
    int m_framesTrained=0;
    using namespace boost;
    boost::posix_time::ptime curr;
	boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
	boost::posix_time::time_duration diff;

	do{
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		
		if(m_framesTrained < t_frames){
			
			JustinaVision::facTrain(t_faceID, t_frames);
			
		}
		
		m_framesTrained++;

		if(m_framesTrained == (int)(t_frames/2))
		{
			faceTrained = true;
			std::cout << "face trained"  << std::endl;
		}

		curr = boost::posix_time::second_clock::local_time();
		ros::spinOnce();
	}while(ros::ok() && (curr - prev).total_milliseconds() < t_timeout && faceTrained==false);

	
    /*JustinaVision::facTrain(t_faceID, t_frames);

    //startFaceTraining(t_faceID, t_frames);

    //loop until reach timeout or until found and train a face
    bool faceTrained = false;
    chrono::milliseconds elapsedTime;
    chrono::steady_clock::time_point startTime = chrono::steady_clock::now();
    while(ros::ok() && elapsedTime.count()<t_timeout && !faceTrained)
    {
        if(m_framesTrained <= (int)(t_frames/2))
        {
            JustinaVision::facTrain(t_faceID, t_frames);
        }
        if(m_framesTrained > (int)(t_frames/2))
        {
            faceTrained = true;
            std::cout << "face trained"  << std::endl;
        }
        elapsedTime = chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now() - startTime
                );
        m_framesTrained++;
        ros::spinOnce();
    }

    if(elapsedTime.count() >= t_timeout || !ros::ok())
    {
        return false;
    }*/
    
    return faceTrained;
}

std::vector<vision_msgs::VisionFaceObject> waitRecognizeFace(float timeOut, std::string id, bool &recognized)
{
		boost::posix_time::ptime curr;
		boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
		boost::posix_time::time_duration diff;
		std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;
		do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			if(id.compare("") == 0)
				JustinaVision::facRecognize();
			else
				JustinaVision::facRecognize(id);
			JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
			curr = boost::posix_time::second_clock::local_time();
			ros::spinOnce();
		}while(ros::ok() && (curr - prev).total_milliseconds() < timeOut && lastRecognizedFaces.size() == 0);

		if(lastRecognizedFaces.size() > 0)
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

    int c_point=0,i=1;
    int nextState = 0;
    bool fail = false;
    bool success = false;
    bool stop=false;

    float gPan=0.0;
	float gTilt=0.0;
	std::string personName = "professional";
	std::stringstream contW;
	std::stringstream contM;
	std::stringstream contU;
	std::stringstream profPlace;
	int mIndex=0;
	int women=0;
	int men=0;
	int unknown=0;
	bool personFound=false;
	int cont=0;
	int cont_sP=0;
	float conf_val;
	int c_right=0;
	int c_left=0;
	std::vector<vision_msgs::VisionFaceObject> dFaces;

    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    validCommands.push_back("start follow me");
    validCommands.push_back("pause");
    validCommands.push_back("continue");
    validCommands.push_back("stop");
    validCommands.push_back("checkpoint");
    validCommands.push_back("goalpoint");
    validCommands.push_back("return home");

    

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {
        case SM_InitialState:
        	std::cout << "executing initial state" << std::endl;
			JustinaHRI::say("I am going to start the person recognition test...");
			JustinaVision::facClearByID(personName);
            nextState = SM_WaitProfessional;

            break;

        case SM_WaitProfessional:

        	std::cout << "waiting for the professional.." << std::endl;
			//detectar cuando aparece el profesional frente al robot
			//esto se realizaría con el sistema de Carlos utilizando la cámara térmica
			std::cout << "meeting human..." << std::endl;
            
            nextState = SM_MeetHuman;

            break;

        case SM_MeetHuman:

        	/*if(cont>2)
			{
				JustinaHRI::say("I could not memorize your face, I am aborting the test");
				ros::Duration(2.0).sleep();///esperar 10 segundos
				JustinaHardware::setHeadGoalPose(0.0, 0.0);
				//nav_tasks.syncMove(0.0,3.141592,80000);
				//nav_tasks.syncMove(1.0,0.0,80000);
				
				nextState = SM_ReportResult;
			}*/


			JustinaHRI::say("I will memorize your face, Please look straight to my kinect camera");
			std::cout << "I will remember your face, Please look straight to my kinect camera" <<std::endl;
			ros::Duration(1.0).sleep();
	
			//planning_msgs::search_remember_face srv;
			//srv.request.robot_instructions = "default";
			//srv.request.face_id = "professional";
			//srv.request.head_movs = movH;
			//clientRememberFace.call(srv);
			//srv.response.training_success;

			//if(srv.response.training_success)
			JustinaVision::startFaceRecognition();
			//JustinaVision::facTrain(personName, 25);
			if(trainFace(personName, 30000, 20))//recuerda al profesional
			{
				JustinaVision::stopFaceRecognition();
				JustinaHRI::say("I have memorized your face, now you can place into the crowd");
				std::cout << "I have remembered your face" <<std::endl;
				//JustinaVision::stopFaceRecognition();
				ros::Duration(10.0).sleep();///esperar 10 segundos
				
				nextState = SM_MoveRobot;

			}
			else{
				JustinaHRI::say("I could not memorized your face, I will try it again");
				std::cout << "I could not remember your face, I will try it again" <<std::endl;
			}	
		
			//cont ++;
			//nextState = SM_MeetHuman;
			
            break;

        case SM_MoveRobot:

        	std::cout << "finding the crowd" << std::endl;
        	JustinaHardware::setHeadGoalPose(0.0, 0.0);
        	JustinaNavigation::moveDistAngle(0.0, 3.141592, 80000);
        	JustinaNavigation::moveDistAngle(1.5, 0.0, 80000);

			//sg_tasks.syncSpeech("I am looking for the professional into the crowd ", 7000);//describir al grupo, genero
			std::cout << "I am looking for the professional into the crowd " <<std::endl;
	
			nextState = SM_PersonRecognition;
            
            break;

        case SM_PersonRecognition:

       	 	conf_val=0.75;
	
			if(cont_sP==1)
				gPan=-0.4;
			if(cont_sP==2)
				gPan=0.4;
			if(cont_sP==3)
				nextState = SM_ReportResult;

			std::cout << "con_sp: "<<cont_sP<<std::endl;
			bool recog;
			
			JustinaHardware::setHeadGoalPose(gPan, gTilt);
			JustinaHRI::say(" I am looking for the professional into the crowd ...");
			std::cout <<"I am looking for the professional into the crowd" << std::endl;

			JustinaVision::startFaceRecognition();
			dFaces = waitRecognizeFace(5000, personName, recog);
			JustinaVision::stopFaceRecognition();
	
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
				
				if(dFaces[i].gender==0)
					women++;
				if(dFaces[i].gender==1)
					men++;
				if(dFaces[i].gender==2)
					unknown++;
	
				std::cout<<"hombres: "<< men << std::endl;

			}

			if(personFound)
				nextState = SM_ReportResult;
			else
			{	
				cont_sP++;
				nextState = SM_PersonRecognition;
			}
            break;


        case SM_ReportResult:

           std::cout <<"Reporting results" << std::endl;
		
		
			mIndex = mIndex + 1;
			c_left = dFaces.size() - mIndex;
			c_right = mIndex - 1;
	
			contW << "There are " << women << " women";
			contM << "There are " << men << " men";
			contU << "There are " << unknown << " people with unknown genre";

			profPlace << "I have found you " << "There are " << c_left << " people to your left and " << c_right << " people to your right ";
	
	
	
			if(personFound && dFaces.size()==mIndex)
				JustinaHRI::say("I have found you. You are the left most person ...");
			if(personFound && mIndex==1)
				JustinaHRI::say("I have found you. You are the right most person ...");

			if(personFound && mIndex!=1 && dFaces.size()!=mIndex)
				JustinaHRI::say(profPlace.str());
			if(!personFound)
				JustinaHRI::say("I could not find you");//describir al grupo, genero


			JustinaHRI::say("I am going to describe the crowd ");
			JustinaHRI::say(contW.str());//describir al grupo, genero
			JustinaHRI::say(contM.str());
			JustinaHRI::say(contU.str());
	
			ros::Duration(4.0).sleep();
				//falta guardar resultados en pdf
            nextState = SM_FinalState;
            break;

        case SM_FinalState:
            std::cout <<"finalState reached" << std::endl;
			JustinaHRI::say("I have finished the person recognition test...");
			//JustinaVision::stopFaceRecognition();
			success=true;
            break;

        }

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
