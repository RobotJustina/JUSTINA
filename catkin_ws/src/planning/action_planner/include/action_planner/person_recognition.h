#include "action_planner/primitives_tasks.h"
#include "action_planner/states_machines.h"
#include "service_manager.h"
#include "roah_rsbb_comm_ros/BenchmarkState.h"
#include "robot_service_manager/speechgeneratortasks.h"
#include "robot_service_manager/headstatus.h"
#include "robot_service_manager/navigationstatus.h"
#include "robot_service_manager/navigationtasks.h"
#include "robot_service_manager/facerecognitiontasks.h"
#include "simple_task_planner/simpletasks.h"
#include "planning_msgs/ask_store_name.h"
#include "planning_msgs/search_remember_face.h"
#include "std_msgs/UInt8.h"
#include "std_srvs/Empty.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"
#include <iostream>
#include <sstream>

/*************PERSON RECOGNITION*********************
	An Operator is introduced to the robot, which needs to learn what the Operator looks like. 
	Once the robot has gathered enough information about the Operator, the Operator mixes within a
	crowd and the robot needs to find the Operator. Once the robot has found its Operator, it must
	state some information about the Operator and the crowd, such as genders.

	Goal
	The robot has to identify the Operator within a crowd and state information about the Operator
	and the crowd.

	Focus
	This test focuses on people detection and recognition; as well as pose recognition and human-
	robot interaction with unknown people.

	Setup
	1. Operator: A “professional” operator is selected by the TC to test the robot. This person
	may a different be drafted from the crowd in each run.
	2. Other people There are no restrictions on other people walking by or standing around
	throughout the complete task.


	Task
	This test may also be held outside the arena This is in order to have the possibility
	to run multiple robots in parallel and reduce the total time needed to test all robots.

	1. Start: The robot starts at a designated starting position, and waits for the “professional”
	operator.

	2. Memorizing the operator: The robot has to memorize the operator. During this phase,
	the robot may instruct the operator to follow a certain setup procedure.

	3. Learning operator name: Optionally, the robot may ask the operator for his/her name
	and make the interaction after finding the operator again more natural.

	4. Waiting 10 seconds: Once the robot states it has finished memorizing the operator, it
	must wait 10 seconds while the operator walks around and mixes with the crowd.

	5. Find the crowd: After the time elapses, the robot must turn around about 180°, find
	the crowd, and start looking for the operator.

		• Crowd size: The crowd may contain between 5 and 10 people, standing or sitting
		or lying within an area of 5 meters (diameter).
		• Crowd position: The crowd will be located behind the robot at a distance between
		2 and 3 meters apart.

	6. Finding the operator: Once the crowd has been located, the robot must greet the
	operator (optionally by name) and state their gender, pose (sitting, standing, raised arm),
	and relative position within the crowd. Alternatively to state the relative position of the
	operator, the robot may point or approach to the operator. In any case, it must be clear
	to the referees that the robot has found the operator, unambiguously. Examples include:
		• My operator is the girl sitting left most of the crowd.
		• Mary, you are standing behind the bearded man with black shirt wearing glasses.
		• Adam is the blond guy standing in the center between two female people.

	Remark: In the case of the slightest ambiguity, no points will be granted.This includes
	the referees not being able to understand or hear the robot.

	7. Describing the crowd: Finally, robot must tell the size of the crowd and how many
	men, women and unidentified people are (even including children).

	8. Delivering report file: Immediately after the test, an USB pen-drive will be collected by
	the referees from the robot. 
	*********************************************************************************************/


class PersonRecognitionSM
{
public:
	/*
	*	ADD THE STATES ID YOU NEED FOR EACH STATE FUNCTION (IF YOU WANT)
	*/
	//struct FaceObject;
	enum States
	{
		InitialState,
		WaitProfessional,
		MeetHuman,
		MoveRobot,
		PersonRecognition,
		ReportResult,
		FinalState
	};

	//for the SM api
	StatesMachines SM;
	static PrimitivesTasks m_tasks;
	static ServiceManager srv_man;
	static SpeechGeneratorTasks sg_tasks;
	static HeadStatus h_status;
	static NavigationStatus nav_status;
	static NavigationTasks nav_tasks;
	static FaceRecognitionTasks face_tasks;
 	static std::vector<FaceRecognitionTasks::FaceObject> dFaces; 
 	//static std::vector< std::pair< float, float > > movementsHead;
 	//static std::vector< std::pair< std_msgs::Float32MultiArray, std_msgs::Float32MultiArray > > movementsHead;
 	//static std_msgs::Float32MultiArray movH;
 	//static SimpleTasks st;
 	//static planning_msgs::ask_store_name store_n;
 	//static planning_msgs::search_remember_face
 	static ros::ServiceClient clientAskName;
 	static ros::ServiceClient clientRememberFace;

 




	static int initialState();
	static int waitProfessional();
	static int meetHuman();
	static int moveRobot();
	static int personRecognition();
	static int reportResult();
	static int finalState();


	/**********************************************************************/
	
	/*
	* A particular constructor for your state machine
	* Initialize your state machine here (add states, define the final state, define the execution method, etc)
	*/
	PersonRecognitionSM(ros::NodeHandle *n);
	bool execute();
	
};
//
//END OF CLASS DEFINITION
//

PrimitivesTasks PersonRecognitionSM::m_tasks; 
ServiceManager PersonRecognitionSM::srv_man;
SpeechGeneratorTasks PersonRecognitionSM::sg_tasks;
HeadStatus PersonRecognitionSM::h_status;
NavigationStatus PersonRecognitionSM::nav_status;
NavigationTasks PersonRecognitionSM::nav_tasks;
FaceRecognitionTasks PersonRecognitionSM::face_tasks;
std::vector<FaceRecognitionTasks::FaceObject> PersonRecognitionSM::dFaces;
ros::ServiceClient  PersonRecognitionSM::clientAskName;
ros::ServiceClient  PersonRecognitionSM::clientRememberFace;





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


	int PersonRecognitionSM::initialState()
	{

		std::cout << "executing initial state" << std::endl;
		sg_tasks.syncSpeech("I am going to start the person recognition test...", 7000);
		face_tasks.clearSpecificFace(personName);
		return (int)WaitProfessional;
	}

	int PersonRecognitionSM::waitProfessional()
	{
		std::cout << "waiting for the professional.." << std::endl;
		//detectar cuando aparece el profesional frente al robot
		//esto se realizaría con el sistema de Carlos utilizando la cámara térmica
		std::cout << "meeting human..." << std::endl;
		//st.askForName(personName, 30000,10000, 3);//pregunta el nombre del profesional
		/*planning_msgs::ask_store_name srv;
		srv.request.attempt_timeout=30000;
		srv.request.repeat_timeout=10000;
		srv.request.max_attempts=3;
		clientAskName.call(srv);
		srv.response.stored_name;
		srv.response.success;
		personName=srv.response.stored_name;
		if(srv.response.success)
		{
			h_status.setHeadPose(0,0);
			ros::Duration(2.0).sleep();
			sg_tasks.syncSpeech("",8000);
			return (int) MeetHuman;
		}
		else */
			return (int) MeetHuman;
			//return (int) WaitProfessional;
	}
	
	int PersonRecognitionSM::meetHuman()
	{		
		if(cont>2)
		{
			sg_tasks.syncSpeech("I could not memorize your face, I am aborting the test", 8000);
			ros::Duration(2.0).sleep();///esperar 10 segundos
			h_status.setHeadPose(0.0,0.0);
			//nav_tasks.syncMove(0.0,3.141592,80000);
			//nav_tasks.syncMove(1.0,0.0,80000);
			
			return (int)ReportResult;
		}


		sg_tasks.syncSpeech("I will memorize your face, Please look straight to my kinect camera", 8000);
		std::cout << "I will remember your face, Please look straight to my kinect camera" <<std::endl;
		ros::Duration(1.0).sleep();

		//planning_msgs::search_remember_face srv;
		//srv.request.robot_instructions = "default";
		//srv.request.face_id = "professional";
		//srv.request.head_movs = movH;
		//clientRememberFace.call(srv);
		//srv.response.training_success;

		//if(srv.response.training_success)

		if(face_tasks.trainFace(personName, 30000, 25))//recuerda al profesional
		{
			sg_tasks.syncSpeech("I have memorized your face, now you can place into the crowd",7000);
			std::cout << "I have remembered your face" <<std::endl;
			ros::Duration(10.0).sleep();///esperar 10 segundos
			return (int)MoveRobot;
		}
		else
			sg_tasks.syncSpeech("I could not memorized your face, I will try it again",7000);
			std::cout << "I could not remember your face, I will try it again" <<std::endl;

		
		cont ++;
		return (int)MeetHuman;
	}
	
	int PersonRecognitionSM::moveRobot()
	{
		

		std::cout << "finding the crowd" << std::endl;
		h_status.setHeadPose(0.0,0.0);
		nav_tasks.syncMove(0.0,3.141592,80000);
		nav_tasks.syncMove(1.5,0.0,80000);

		//sg_tasks.syncSpeech("I am looking for the professional into the crowd ", 7000);//describir al grupo, genero
		std::cout << "I am looking for the professional into the crowd " <<std::endl;
	
		return (int)PersonRecognition;
	}

	int PersonRecognitionSM::personRecognition()
	{
		conf_val=0.75;

		if(cont_sP==1)
			gPan=-0.4;
		if(cont_sP==2)
			gPan=0.4;
		if(cont_sP==3)
			return (int) ReportResult;

		
		
		h_status.setHeadPose(gPan,gTilt);
		sg_tasks.syncSpeech(" I am looking for the professional into the crowd ...", 7000);
		std::cout <<"I am looking for the professional into the crowd" << std::endl;	
		face_tasks.recognizeFaces(dFaces, 7000);//encontrar al profesional

		std::cout <<"tamaño de arreglo " << dFaces.size() <<std::endl;

		for(int i=0; i<dFaces.size(); i++)
		{
			if(dFaces[i].faceID==personName && dFaces[i].confidence>=conf_val)
			{
				conf_val=dFaces[i].confidence;
				mIndex=i;
				personFound=true;
				std::cout << "indice de la persona " << mIndex << std::endl; 
				std::cout << "valor de confianza " << conf_val << std::endl; 
			}
			
			if(dFaces[i].faceGender==0)
				women++;
			if(dFaces[i].faceGender==1)
				men++;
			if(dFaces[i].faceGender==2)
				unknown++;

			std::cout<<"hombres: "<< men << std::endl;

		}

		if(personFound)
			return (int) ReportResult;	
		else
		{
			cont_sP++;
			return (int) PersonRecognition;
		}

	}

	int PersonRecognitionSM::reportResult()
	{
		
		std::cout <<"Reporting results" << std::endl;
		
		
		mIndex = mIndex + 1;
		c_left = dFaces.size() - mIndex;
		c_right = mIndex - 1;

		contW << "There are " << women << " women";
		contM << "There are " << men << " men";
		contU << "There are " << unknown << " people with unknown genre";

		profPlace << "I have found you " << "There are " << c_left << " people to your left and " << c_right << " people to your right ";



		if(personFound && dFaces.size()==mIndex)
			sg_tasks.syncSpeech("I have found you. You are the left most person ...", 7000);
		if(personFound && mIndex==1)
			sg_tasks.syncSpeech("I have found you. You are the right most person ...", 7000);

		if(personFound && mIndex!=1 && dFaces.size()!=mIndex)
			sg_tasks.syncSpeech(profPlace.str(), 9000);
		else
			sg_tasks.syncSpeech("I could not find you", 7000);//describir al grupo, genero


		sg_tasks.syncSpeech("I am going to describe the crowd ", 7000);
		sg_tasks.syncSpeech(contW.str(), 7000);//describir al grupo, genero
		sg_tasks.syncSpeech(contM.str(), 7000);
		sg_tasks.syncSpeech(contU.str(), 7000);

		ros::Duration(4.0).sleep();
		//falta guardar resultados en pdf
		return (int)FinalState;
	}

	int PersonRecognitionSM::finalState()
	{
		std::cout <<"finalState reached" << std::endl;
		sg_tasks.syncSpeech("I have finished the person recognition test...", 7000);
		return (int)FinalState;
	}

PersonRecognitionSM::PersonRecognitionSM(ros::NodeHandle *n)
{

	h_status.initRosConnection(n);
	face_tasks.initRosConnection(n);
	nav_status.initRosConnection(n);
	nav_tasks.initRosConnection(n);

	clientAskName = n->serviceClient<planning_msgs::ask_store_name>("/simple_task_planner/ask_store_name");
	clientRememberFace = n->serviceClient<planning_msgs::search_remember_face>("/simple_task_planner/search_and_remember_face");
	
	
	//add states to the state machine
	SM.addState((int)InitialState, &initialState);
	SM.addState((int)WaitProfessional, &waitProfessional);
	SM.addState((int)MeetHuman, &meetHuman);
	SM.addState((int)MoveRobot, &moveRobot);
	SM.addState((int)PersonRecognition, &personRecognition);
	SM.addState((int)ReportResult, &reportResult);
	SM.addState((int)FinalState, &finalState, true);
	
	
	//execute the state machine from the initial state until the final state
	//while(SM.runNextStep());

	//return true;
}
bool PersonRecognitionSM::execute()
{
	while(SM.runNextStep())
	{
		ros::spinOnce();
	}
	return true;
}
