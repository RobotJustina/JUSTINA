#include "action_planner/primitives_tasks.h"
#include "action_planner/service_manager.h"
#include "action_planner/states_machines.h"
#include "roah_rsbb_comm_ros/ResultHOPF.h"
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include <iostream>
#include <vector>

class WelcomingSM
{
private:
	/*
	*	ADD THE STATES ID YOU NEED FOR EACH STATE FUNCTION (IF YOU WANT)
	*/
	enum States
	{
		InitialState,
		WaitForStartCommand,
		WaitForBellRing,
		RecognizeVisitor,
		GreetVisitor,
		PerformDrKimbleRoutine,
		PerformDeliManRoutine,
		PerformPostManRoutine,
		PerformUnknownPersonRoutine,
		MoveToInitialPosition,
		FinalState
	};

	static ros::NodeHandle* node;
	//for the SM api
	StatesMachines SM;
	//service manager
	static ServiceManager srv_man;
	//stpln
	static PrimitivesTasks m_tasks;
	//SM variables
	static std::string visitorName;
	static ros::ServiceClient srvCltEndPrepare;
	static ros::ServiceClient srvCltEndExecute;
	static ros::Subscriber subBenchmarkState;
	static ros::Subscriber subDoorBell;
	static ros::Publisher pubMessagesSaved;
	static ros::Publisher pubRecordData;

	//states variables
	static std_msgs::String entranceDoorPosition;
	static std_msgs::String kitchenPosition;
	static std_msgs::String GAPosition;
	static std_msgs::String letterPosition;
	static std_msgs::String robotInitialPosition;
	static std_msgs::String exitPosition;
	static bool doorBellRing;
	/********************************************************************/
	/*
	*	ADD THE STATE FUNCTIONS YOU NEED
	*/
	static int fInitialState();
	static int fWaitForStartCommand();
	static int fWaitForBellRing();
	static int fRecognizeVisitor();
	static int fGreetVisitor();
	static int fPerformDrKimbleRoutine();
	static int fPerformDeliManRoutine();
	static int fPerformPostManRoutine();
	static int fPerformUnknownPersonRoutine();
	static int fMoveToInitialPosition();
	static int fFinalState();
	/**********************************************************************/

	static void callback_benchmark_state(const roah_rsbb_comm_ros::BenchmarkState::ConstPtr& msg);
	static void callback_door_bell(const std_msgs::Empty&);
	
public:
	WelcomingSM(PrimitivesTasks tasks,  ros::NodeHandle*);
	bool execute();
};

PrimitivesTasks WelcomingSM::m_tasks;
ServiceManager WelcomingSM::srv_man;
ros::NodeHandle* WelcomingSM::node;
ros::ServiceClient WelcomingSM::srvCltEndPrepare;
ros::ServiceClient WelcomingSM::srvCltEndExecute;
ros::Subscriber WelcomingSM::subBenchmarkState;
ros::Publisher WelcomingSM::pubMessagesSaved;
ros::Publisher WelcomingSM::pubRecordData;
ros::Subscriber WelcomingSM::subDoorBell;

std_msgs::String WelcomingSM::entranceDoorPosition;
std_msgs::String WelcomingSM::kitchenPosition;
std_msgs::String WelcomingSM::GAPosition;
std_msgs::String WelcomingSM::letterPosition;
std_msgs::String WelcomingSM::robotInitialPosition;
std_msgs::String WelcomingSM::exitPosition;
bool WelcomingSM::doorBellRing;

/*
* A particular constructor for your state machine
* Initialize your state machine here (add states, define the final state, define the execution method, etc)
*/
WelcomingSM::WelcomingSM(PrimitivesTasks tasks, ros::NodeHandle* n)
{
	m_tasks = tasks;
	node = n;

	//int (WelcomingSM::*
	//add states to the state machine
	SM.addState((int)InitialState, &fInitialState);
	SM.addState((int)WaitForStartCommand, &fWaitForStartCommand);
	SM.addState((int)WaitForBellRing, &fWaitForBellRing);
	SM.addState((int)RecognizeVisitor, &fRecognizeVisitor);
	SM.addState((int)GreetVisitor, &fGreetVisitor);
	SM.addState((int)PerformDrKimbleRoutine, &fPerformDrKimbleRoutine);
	SM.addState((int)PerformDeliManRoutine, &fPerformDeliManRoutine);
	SM.addState((int)PerformPostManRoutine, &fPerformPostManRoutine);
	SM.addState((int)PerformUnknownPersonRoutine, &fPerformUnknownPersonRoutine);
	SM.addState((int)MoveToInitialPosition, &fMoveToInitialPosition);
	SM.addState((int)FinalState, &fFinalState, true);

	srvCltEndPrepare = n->serviceClient<std_srvs::Empty>("roah_rsbb/end_prepare");
	srvCltEndExecute = n->serviceClient<std_srvs::Empty>("roah_rsbb/end_execute");
	subBenchmarkState = n->subscribe("/roah_rsbb/benchmark/state", 100, WelcomingSM::callback_benchmark_state);
	pubMessagesSaved = n->advertise<std_msgs::UInt32>("roah_rsbb/messages_saved", 100);
	pubRecordData = n->advertise<std_msgs::Bool>("br_record_data", 100);

	subDoorBell = n->subscribe("/roah_rsbb/devices/bell", 100, WelcomingSM::callback_door_bell);
}
bool WelcomingSM::execute()
{
	while(SM.runNextStep())
		ros::spinOnce();

	return true;
}

/*
*	ADD THE STATE FUNCTIONS YOU NEED
*/
 int WelcomingSM::fInitialState()
{
	std::cout << "Initializing Welcoming Visitors SM" << std::endl;

	entranceDoorPosition.data = "entrance";
	kitchenPosition.data = "kitchen";
	GAPosition.data = "bedroom";
	letterPosition.data = "livingroom";
	robotInitialPosition.data = "startlocation";
	exitPosition.data = "exitA";

	doorBellRing = false;

	return (int)WaitForStartCommand;
}

int WelcomingSM::fWaitForStartCommand()
{
	std::cout << "waiting for prepare signal....." << std::endl;
	if(!ros::service::waitForService("/roah_rsbb/end_prepare", 300000))
	{
		std::cout << "RSBB NOT AVAILABLE :'(" << std::endl;
		return (int)FinalState;
	}
	std::cout << "Prepare signal received..." << std::endl;

	//--------------------------------------------------------//

	std_srvs::Empty es;
	std::cout << "Calling end_prepare service" << std::endl;
	srvCltEndPrepare.call(es);

	//----------------------------------------------------------//

	std::cout << "Waiting for execute signal..." << std::endl;
	if(!ros::service::waitForService("roah_rsbb/end_execute", 300000))		 
	{
		std::cout << "RSBB NOT AVAILABLE :'(" << std::endl;
		return (int)FinalState;	
	}
	std::cout << "Execute signal received :D" << std::endl;

	//----------------------------------------------------------//
	
	std_msgs::UInt32 msgMessages;
	pubMessagesSaved.publish(msgMessages);

	return (int)WaitForBellRing;
}

int WelcomingSM::fWaitForBellRing()
{
	std::cout << "Waiting for the door bell ring....." << std::endl;

	std_msgs::Float32 tilt, pan, cTilt, cPan;
	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);

	//wait for the door bell ring
	doorBellRing = false;
	while(!doorBellRing)
		ros::spinOnce();
	doorBellRing = false;

	//the bell rings, move to entrance door
	if(!srv_man.mpGetClose(entranceDoorPosition))
		if(!srv_man.mpGetClose(entranceDoorPosition))
			srv_man.mpGetClose(entranceDoorPosition);

	return (int)RecognizeVisitor;
}

int WelcomingSM::fRecognizeVisitor()
{
	std::cout << "Recognizing visitor using vision..." << std::endl;

	std_msgs::Float32 tilt, pan, cTilt, cPan;
	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);

	//request for the door to be opened 
	srv_man.spgenAsay("Please, open the door and I will identify the visitor.");
	ros::Duration(6).sleep();

	//wait for the door to be opened
	ros::Duration(20).sleep();
	//asume the door was opened

	//try to recognize Dr. Kim using face
	if(srv_man.prsfndFind("doctor", 10000))		//the doctor was ringing the door
		return (int)PerformDrKimbleRoutine;
	
	//if the person is not the Dr. try using clothes
	std::string recognizedPerson;
	if(srv_man.vsnPersonReco(recognizedPerson))
	{
		//find some person
		if(recognizedPerson.compare("postman")==0)
 			return (int)PerformPostManRoutine;

		if(recognizedPerson.compare("bakery")==0)
 			return (int)PerformDeliManRoutine;
	}

	//unknown person
	return (int)PerformUnknownPersonRoutine;

	//TODO: Call the vision routine
	//std::cout << "visitor recognized: " << std::endl;
	//return (int)GreetVisitor;
}
int WelcomingSM::fGreetVisitor()
{
	return (int)FinalState;
}
 int WelcomingSM::fPerformDrKimbleRoutine()
{
	srv_man.spgenAsay("Hi Dr. Kimble, I am coming to open the door.");
	ros::Duration(6).sleep();
	srv_man.spgenAsay("Please come in, I will lead you to the Annies bedroom.");

	std_msgs::Float32 tilt, pan, cTilt, cPan;
	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);
	//guide the Dr to the GA position
	if(!srv_man.mpGetClose(GAPosition))
		if(!srv_man.mpGetClose(GAPosition))
			srv_man.mpGetClose(GAPosition);

	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);
	//wait until the dr leaves the arena
	srv_man.spgenAsay("Dr. Kimble, I am not able to follow you. Please leave the house when you finish.");
	ros::Duration(10).sleep();
	srv_man.spgenAsay("I will show you the exit.");

	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);
	//guide the Dr to the exit
	if(!srv_man.mpGetClose(exitPosition))
		if(!srv_man.mpGetClose(exitPosition))
			srv_man.mpGetClose(exitPosition);

	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);
	srv_man.spgenAsay("I will return to my initial position.");

	return (int)MoveToInitialPosition;
}
 int WelcomingSM::fPerformDeliManRoutine()
{
	srv_man.spgenAsay("Hello, I am coming to get the breakfast.");
	ros::Duration(6).sleep();

	srv_man.spgenAsay("Please come in, I will lead you to the kitchen.");

	std_msgs::Float32 tilt, pan, cTilt, cPan;
	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);
	//guide the deliman to the GA position
	if(!srv_man.mpGetClose(kitchenPosition))
		if(!srv_man.mpGetClose(kitchenPosition))
			srv_man.mpGetClose(kitchenPosition);

	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);
	//wait until the deli man leaves the arena
	srv_man.spgenAsay("Please, put the breakfast food on the table.");
	ros::Duration(10).sleep();
	srv_man.spgenAsay("I will show you the exit. Please follow me.");

	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);
	//guide the deli man to the exit
	if(!srv_man.mpGetClose(exitPosition))
		if(!srv_man.mpGetClose(exitPosition))
			srv_man.mpGetClose(exitPosition);

	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);
	srv_man.spgenAsay("Please leave the house.");
	ros::Duration(3).sleep();

	srv_man.spgenAsay("I will return to my initial position.");
	return (int)MoveToInitialPosition;
}
 int WelcomingSM::fPerformPostManRoutine()
{
	srv_man.spgenAsay("Hello, I am coming to get the post mail.");
	ros::Duration(6).sleep();

	srv_man.spgenAsay("Please come in and put the postal mail on the hall table.");

	//wait for the postman to perform its operation
	ros::Duration(10).sleep();

	srv_man.spgenAsay("Please leave the house.");
	ros::Duration(5).sleep();

	srv_man.spgenAsay("I will return to my initial position.");
	return (int)MoveToInitialPosition;
}
 int WelcomingSM::fPerformUnknownPersonRoutine()
{
	std_msgs::Float32 tilt, pan, cTilt, cPan;
	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);

	srv_man.spgenAsay("Sorry, I do not know you. I cannot open the door.");
	ros::Duration(6).sleep();

	srv_man.spgenAsay("I will return to my initial position.");
	return (int)MoveToInitialPosition;
}
 int WelcomingSM::fMoveToInitialPosition()
{
	std_msgs::Float32 tilt, pan, cTilt, cPan;
	tilt.data = 0.0;
	pan.data = 0.0;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);

	//the bell rings, move to entrance door
	if(!srv_man.mpGetClose(robotInitialPosition))
		if(!srv_man.mpGetClose(robotInitialPosition))
			srv_man.mpGetClose(robotInitialPosition);

	return (int)WaitForBellRing;
}
 int WelcomingSM::fFinalState()
{
	return (int)FinalState;
}
/**********************************************************************/

/***********ROS TOPICS CALLBACKS*****************/
void WelcomingSM::callback_benchmark_state(const roah_rsbb_comm_ros::BenchmarkState::ConstPtr &msg)
{
	//std::cout << "BechmarkState changed: " << msg->benchmark_state << std::endl;
}

void WelcomingSM::callback_door_bell(const std_msgs::Empty &d)
{
	doorBellRing = true;
}
///////////////************************///////////////
