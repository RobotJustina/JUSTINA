#include "action_planner/primitives_tasks.h"
#include "action_planner/states_machines.h"
#include "service_manager.h"
#include "roah_rsbb_comm_ros/BenchmarkState.h"
#include "std_msgs/UInt8.h"
#include "std_srvs/Empty.h"
#include "std_msgs/UInt32.h"
#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"
#include <iostream>

class NavigationSM
{
public:
	/*
	*	ADD THE STATES ID YOU NEED FOR EACH STATE FUNCTION (IF YOU WANT)
	*/
	enum States
	{
		InitialState,
		WaitForPrepareSignal,
		MovingRobotToStartPosition,
		SendingPrepareReadySignal,
		WaitForExecuteSignal,
		WaitingForGoalPoses,
		MovingRobotToGoalPoint,
		FinalState
	};

	//for the SM api
	StatesMachines SM;
	static PrimitivesTasks m_tasks;
	static ServiceManager srv_man;
	static ros::NodeHandle* node;
	static std::string classPrompt;
	static ros::ServiceClient srvCltEndPrepare;
	static ros::ServiceClient srvCltEndExecute;
	static ros::Subscriber subGoalPose;
	static ros::Subscriber subBenchmarkState;
	static ros::Publisher pubMessagesSaved;
	static ros::Publisher pubReachedWaypoint;
	static ros::Publisher pubRecordData;

	static int initialState();
	static int waitForPrepareSignal();
	static int movingRobotToStartPosition();
	static int waitForExecuteSignal();
	static int waitingForGoalPoses();
	static int movingRobotToGoalPoint();
	static int finalState();

	static bool prepareSignalReceived;
	static bool executeSignalReceived;
	static float currentGoalX;
	static float currentGoalY;
	static float currentGoalTheta;
	static bool newGoalPointReceived;
	static uint8_t goalPointCounter;
	
	static void callback_goal_pose(const geometry_msgs::Pose2D::ConstPtr& msg);
	static void callback_benchmark_state(const roah_rsbb_comm_ros::BenchmarkState::ConstPtr& msg);

	/**********************************************************************/
	
	/*
	* A particular constructor for your state machine
	* Initialize your state machine here (add states, define the final state, define the execution method, etc)
	*/
	NavigationSM(PrimitivesTasks tasks, ros::NodeHandle* n);
	bool execute();
	
};
//
//END OF CLASS DEFINITION
//



PrimitivesTasks NavigationSM::m_tasks; 
ServiceManager NavigationSM::srv_man;
ros::NodeHandle* NavigationSM::node;
std::string NavigationSM::classPrompt;
ros::ServiceClient NavigationSM::srvCltEndPrepare;
ros::ServiceClient NavigationSM::srvCltEndExecute;
ros::Subscriber NavigationSM::subGoalPose;
ros::Subscriber NavigationSM::subBenchmarkState;
ros::Publisher NavigationSM::pubMessagesSaved;
ros::Publisher NavigationSM::pubReachedWaypoint;
ros::Publisher NavigationSM::pubRecordData;

float NavigationSM::currentGoalX;
float NavigationSM::currentGoalY;
float NavigationSM::currentGoalTheta;
bool NavigationSM::newGoalPointReceived;
uint8_t NavigationSM::goalPointCounter;

int NavigationSM::initialState()
{
	std::cout << classPrompt << "Executing initial state: waiting for needed services..." << std::endl;
	bool success = true;
	//success &= ros::service::waitForService("mp_setspeeds", 5000);
	//success &= ros::service::waitForService("mp_getclose", 5000);
	//success &= ros::service::waitForService("vsn_findonplanes", 5000);
	ros::service::waitForService("spg_say", 5000); //This service is optional for this test
	if(success)
	{
		std::cout << classPrompt << "Ready for the navigation test :D" << std::endl;
		srv_man.spgenSay("I am ready for the navigation test...", 7000);
		return (int)WaitForPrepareSignal;
	}
	else
	{
		std::cout << classPrompt << "Needed services are not available :'(" << std::endl;
		return (int)FinalState;
	}
}

int NavigationSM::waitForPrepareSignal()
{
	std::cout << classPrompt << "Waiting for prepare signal... " << std::endl;
	if(!ros::service::waitForService("/roah_rsbb/end_prepare", 60000))
	{
		std::cout << classPrompt << "RSBB NOT AVAILABLE :'(" << std::endl;
		return (int)FinalState;
	}
	std::cout << classPrompt << "Prepare signal received..." << std::endl;
	return (int)MovingRobotToStartPosition;
}

int NavigationSM::movingRobotToStartPosition()
{
	std::cout << classPrompt << "Dear user. Please move Justina to the start position and press any key when ready..." << std::endl;
	std::string mistring;
	std::cin >> mistring;
	std_srvs::Empty es;
	std::cout << classPrompt << "Calling end_prepare service" << std::endl;
	srvCltEndPrepare.call(es);
	return (int)WaitForExecuteSignal;
}

int NavigationSM::waitForExecuteSignal()
{
	std::cout << classPrompt << "Waiting for execute signal..." << std::endl;
	if(!ros::service::waitForService("roah_rsbb/end_execute", 60000))		 
	{
		std::cout << classPrompt << "RSBB NOT AVAILABLE :'(" << std::endl;
		return (int)FinalState;	
	}
	std::cout << classPrompt << "Execute signal received :D" << std::endl;

	std::cout << classPrompt << "Sending messages log command..." << std::endl;
	std_msgs::UInt32 msgMessages;
	pubMessagesSaved.publish(msgMessages);

	return (int)WaitingForGoalPoses;
}

int NavigationSM::waitingForGoalPoses()
{
	std::cout << classPrompt << "Waiting for new goal point ... " << std::endl;
	ros::Rate loop(10);
	while(ros::ok() && !newGoalPointReceived)
	{
		loop.sleep();
		ros::spinOnce();
	}
	std::cout << classPrompt << "New goal point received... " << std::endl;
	newGoalPointReceived = false;
	return (int)MovingRobotToGoalPoint;
}

int NavigationSM::movingRobotToGoalPoint()
{
	std::cout << classPrompt << "Moving robot to goal point:" << currentGoalX << "  " << currentGoalY << "  "  << currentGoalTheta << std::endl;
	std_msgs::Float32 msgGoalX;
	std_msgs::Float32 msgGoalY;
	std_msgs::Float32 msgGoalTheta;
	msgGoalX.data = currentGoalX;
	msgGoalY.data = currentGoalY;
	msgGoalTheta.data = currentGoalTheta;

	std::cout << classPrompt << "Sending topic br_record_data = true" << std::endl;
	std_msgs::Bool msgRecordData;
	msgRecordData.data = true;
	pubRecordData.publish(msgRecordData);

	if(!srv_man.mpGetClose(msgGoalX, msgGoalY, msgGoalTheta))
		if(!srv_man.mpGetClose(msgGoalX, msgGoalY, msgGoalTheta))
			if(!srv_man.mpGetClose(msgGoalX, msgGoalY, msgGoalTheta))
				std::cout << classPrompt << "Cannot reach goal point " << std::endl;

	std::cout << classPrompt << "Sending goal-reached notification..." << std::endl;
	std_msgs::UInt8 msgCounter;
	msgCounter.data = goalPointCounter;
	pubReachedWaypoint.publish(msgCounter);
	std::cout << classPrompt << "Sending topic br_record_data = false" << std::endl;
	msgRecordData.data = false;
	pubRecordData.publish(msgRecordData);
	std::cout << classPrompt << "Calling end_execute service..." << std::endl;
	std_srvs::Empty es;
	srvCltEndExecute.call(es);

	return (int)WaitForPrepareSignal;
}

int NavigationSM::finalState()
{
	std::cout << "finalState reached" << std::endl;
	srv_man.spgenSay("I have finished the navigation test...", 7000);
	return (int)FinalState;
}

void NavigationSM::callback_benchmark_state(const roah_rsbb_comm_ros::BenchmarkState::ConstPtr &msg)
{
	std::cout << classPrompt << "BechmarkState changed: " << msg->benchmark_state << std::endl;
}

void NavigationSM::callback_goal_pose(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	std::cout << classPrompt << "Goal point received: " << msg->x << "  " << msg->y << "  " << msg->theta << std::endl;
	currentGoalX = msg->x;
	currentGoalY = msg->y;
	currentGoalTheta = msg->theta;
	newGoalPointReceived = true;
}

NavigationSM::NavigationSM(PrimitivesTasks tasks, ros::NodeHandle* n)
{
	classPrompt = "ACT-PLN:NAVIGATION.->";
	m_tasks = tasks;
	node = n;
	//add states to the state machine
	SM.addState((int)InitialState, &initialState);
	SM.addState((int)WaitForPrepareSignal, &waitForPrepareSignal);
	SM.addState((int)MovingRobotToStartPosition, &movingRobotToStartPosition);
	SM.addState((int)WaitForExecuteSignal, &waitForExecuteSignal);
	SM.addState((int)WaitingForGoalPoses, &waitingForGoalPoses);
	SM.addState((int)MovingRobotToGoalPoint, &movingRobotToGoalPoint);
	SM.addState((int)FinalState, &finalState, true);

	srvCltEndPrepare = n->serviceClient<std_srvs::Empty>("roah_rsbb/end_prepare");
	srvCltEndExecute = n->serviceClient<std_srvs::Empty>("roah_rsbb/end_execute");
	subBenchmarkState = n->subscribe("/roah_rsbb/benchmark/state", 100, NavigationSM::callback_benchmark_state);
	subGoalPose = n->subscribe("/roah_rsbb/goal", 100, NavigationSM::callback_goal_pose);
	pubMessagesSaved = n->advertise<std_msgs::UInt32>("roah_rsbb/messages_saved", 100);
	pubReachedWaypoint = n->advertise<std_msgs::UInt8>("roah_rsbb/reached_waypoint", 100);
	pubRecordData = n->advertise<std_msgs::Bool>("br_record_data", 100);

	goalPointCounter = 0;

	//execute the state machine from the initial state until the final state
	//while(SM.runNextStep());

	//return true;
}
bool NavigationSM::execute()
{
	while(SM.runNextStep())
	{
		ros::spinOnce();
	}
	return true;
}
