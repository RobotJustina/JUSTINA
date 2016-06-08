#include "action_planner/primitives_tasks.h"
#include "action_planner/service_manager.h"
#include "action_planner/robot_knowledge.h"
#include "visualization_msgs/MarkerArray.h"
#include "action_planner/states_machines.h"
#include "ros/ros.h"
#include "roah_rsbb_comm_ros/BenchmarkState.h"
#include "roah_rsbb_comm_ros/ResultHOPF.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "geometry_msgs/Pose2D.h"
#include "std_srvs/Empty.h"
#include <iostream>
#include <vector>
#include <map>

class ObjectPerceptionSM
{
private:
	/*
	*	ADD THE STATES ID YOU NEED FOR EACH STATE FUNCTION (IF YOU WANT)
	*/
	enum States
	{
		InitialState,
		WaitForInitCommand,
		MoveHead,
		LookForObjects,
		ReportResult,
		FinalState
	};
	struct headParams
	{
		headParams(double hpan, double htilt) : tilt(htilt), pan(hpan) {}
		double tilt;
		double pan;
	};

	static ros::NodeHandle* node;

	//for the SM api
	StatesMachines SM;
	//service manager
	static ServiceManager srv_man;
	//stpln
	static PrimitivesTasks m_tasks;
	static RobotKnowledge know;
	//state func members
	static visualization_msgs::Marker currentObjectFound;
	static visualization_msgs::Marker bestObjectFound;
	static int searchAttempt;
	static int maxAttempts;
	static int currentHeadPosition;
	static std::vector<headParams> headPositions;
	static std::map<std::string, std::string> realObjectNames;

	static ros::ServiceClient srvCltEndPrepare;
	static ros::ServiceClient srvCltEndExecute;
	static ros::Subscriber subBenchmarkState;
	static ros::Publisher pubMessagesSaved;
	static ros::Publisher pubRecordData;

	/********************************************************************/
	/*
	*	ADD THE STATE FUNCTIONS YOU NEED
	*/
	static int initialState();
	static int waitForInitCommand();
	static int moveHead();
	static int lookForObjects();
	static int reportResult();
	static int finalState();
	/**********************************************************************/
	static void callback_benchmark_state(const roah_rsbb_comm_ros::BenchmarkState::ConstPtr& msg);
	
public:
	ObjectPerceptionSM(PrimitivesTasks tasks, ros::NodeHandle *);
	bool execute();
};

int ObjectPerceptionSM::maxAttempts;
visualization_msgs::Marker ObjectPerceptionSM::currentObjectFound;
visualization_msgs::Marker ObjectPerceptionSM::bestObjectFound;
int ObjectPerceptionSM::searchAttempt;
PrimitivesTasks ObjectPerceptionSM::m_tasks;
std::vector<ObjectPerceptionSM::headParams> ObjectPerceptionSM::headPositions;
int ObjectPerceptionSM::currentHeadPosition;
ServiceManager ObjectPerceptionSM::srv_man;
ros::NodeHandle* ObjectPerceptionSM::node;
ros::ServiceClient ObjectPerceptionSM::srvCltEndPrepare;
ros::ServiceClient ObjectPerceptionSM::srvCltEndExecute;
ros::Subscriber ObjectPerceptionSM::subBenchmarkState;
ros::Publisher ObjectPerceptionSM::pubMessagesSaved;
ros::Publisher ObjectPerceptionSM::pubRecordData;
RobotKnowledge ObjectPerceptionSM::know;
std::map<std::string, std::string> ObjectPerceptionSM::realObjectNames;

/*
* A particular constructor for your state machine
* Initialize your state machine here (add states, define the final state, define the execution method, etc)
*/
ObjectPerceptionSM::ObjectPerceptionSM(PrimitivesTasks tasks, ros::NodeHandle *n)
{
	m_tasks = tasks;
	node = n;
	//int (ObjectPerceptionSM::*
	//add states to the state machine
	SM.addState((int)InitialState, &initialState);
	SM.addState((int)WaitForInitCommand, &waitForInitCommand);
	SM.addState((int)MoveHead, &moveHead);
	SM.addState((int)LookForObjects, &lookForObjects);
	SM.addState((int)ReportResult, &reportResult);;
	SM.addState((int)FinalState, &finalState, true);

	srvCltEndPrepare = n->serviceClient<std_srvs::Empty>("roah_rsbb/end_prepare");
	srvCltEndExecute = n->serviceClient<roah_rsbb_comm_ros::ResultHOPF>("roah_rsbb/end_execute");
	subBenchmarkState = n->subscribe("/roah_rsbb/benchmark/state", 100, ObjectPerceptionSM::callback_benchmark_state);
	pubMessagesSaved = n->advertise<std_msgs::UInt32>("roah_rsbb/messages_saved", 100);
	pubRecordData = n->advertise<std_msgs::Bool>("br_record_data", 100);
}
bool ObjectPerceptionSM::execute()
{
	while(SM.runNextStep())
	{
		ros::spinOnce();
	}
	return true;
}

/*
*	ADD THE STATE FUNCTIONS YOU NEED
*/
int ObjectPerceptionSM::initialState()
{
	std::cout << "executing initial state " << maxAttempts << std::endl;

	/*Initialize member variables*/
	searchAttempt = 0;
	currentHeadPosition = 0;
	bestObjectFound.ns = "small_white_mug";

	realObjectNames["pepsi"] = "a4";
	realObjectNames["tazaBlanca"] = "a2";
	realObjectNames["tazaGris"] = "a3";
	realObjectNames["tazaNegra"] = "a1";
	realObjectNames["fork"] = "b1";
	realObjectNames["knife"] = "b2";
	realObjectNames["cajaAmarilla"] = "c1";
	realObjectNames["cajaRosa"] = "c2";
	realObjectNames["marcoAmarillo"] = "d1";
	realObjectNames["marcoNegro"] = "d2";
	
	////head positions to find the object
	//headPositions.push_back(headParams(0.0,-0.8));
	//headPositions.push_back(headParams(0.3,-0.8));
	//headPositions.push_back(headParams(-0.3,-0.8));
	//headPositions.push_back(headParams(0.0,-0.6));
	//headPositions.push_back(headParams(0.3,-0.6));
	//headPositions.push_back(headParams(-0.3,-0.6));

	headPositions.push_back(headParams(0.0,-0.8));
	headPositions.push_back(headParams(0.0,-0.8));
	headPositions.push_back(headParams(0.0,-0.8));
	maxAttempts =  headPositions.size();

	return (int)WaitForInitCommand;
}

int ObjectPerceptionSM::waitForInitCommand()
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
	
	std::cout << "Sending messages log command..." << std::endl;
	std_msgs::UInt32 msgMessages;
	pubMessagesSaved.publish(msgMessages);


	//std::getchar();
	searchAttempt=0;
	currentHeadPosition = 0;
	return (int)MoveHead;
}

int ObjectPerceptionSM::moveHead()
{
	std::cout << "moving head " << std::endl;
	
	//move the head to current pos
	std_msgs::Float32 tilt, pan, cTilt, cPan;
	tilt.data = headPositions[currentHeadPosition % headPositions.size()].tilt;
	pan.data = headPositions[currentHeadPosition % headPositions.size()].pan;
	srv_man.hdLookAt(pan, tilt, cPan, cTilt);
	currentHeadPosition++;
	//currentHeadPosition = (currentHeadPosition < headPositions.size()) ? currentHeadPosition+1: 0;
	
	//std::cout << "moving head to : (" << pan.data << ", " << tilt.data << ")" << std::endl;

	return (int)LookForObjects;
}

int ObjectPerceptionSM::lookForObjects()
{
	std::cout << "looking for objects" << std::endl;
	
	currentObjectFound.ns = "clean";
	
	ros::Duration(0.5).sleep();
	if(m_tasks.searchSingleObject(currentObjectFound))
	{
		bestObjectFound = currentObjectFound;
		//one object found, report its position
		return (int)ReportResult;
	}
	if(searchAttempt<maxAttempts)
	{
		searchAttempt++;
		if(currentObjectFound.ns.compare("clean")==0)
		{
			//no object found
			std::cout << "ACT-PLN: no objects found, try again moving head" << std::endl;
			return (int)MoveHead;
		}
		std::cout << "more than one object found, try again on same place" << std::endl;
		bestObjectFound = currentObjectFound;
		return (int)MoveHead;
	}
	return (int)ReportResult;
}

int ObjectPerceptionSM::reportResult()
{
	std::cout << "Repoorting results (ie call end_execute)... " << std::endl;

	std::cout << "Calling end_execute service..." << std::endl;

	roah_rsbb_comm_ros::ResultHOPF objResult;
	objResult.request.object_name = realObjectNames[bestObjectFound.ns];
	objResult.request.object_class = know.objectDictionary[realObjectNames[bestObjectFound.ns]];
	geometry_msgs::Pose2D pose;
	pose.x = bestObjectFound.pose.position.x;
	pose.y = bestObjectFound.pose.position.y;
	pose.theta = bestObjectFound.pose.position.z;
	objResult.request.object_pose = pose;

	srvCltEndExecute.call(objResult);
	std::cout << "FOUND OBJECT REPORT: " << objResult.request <<  std::endl;
	//return (int)FinalState;
	return (int)WaitForInitCommand;
}

int ObjectPerceptionSM::finalState()
{
	std::cout << "finalState reached" << std::endl;
	return (int)FinalState;
}

/**********************************************************************/

/***********ROS TOPICS CALLBACKS*****************/
void ObjectPerceptionSM::callback_benchmark_state(const roah_rsbb_comm_ros::BenchmarkState::ConstPtr &msg)
{
	std::cout << "BechmarkState changed: " << msg->benchmark_state << std::endl;
}
///////////////************************///////////////
