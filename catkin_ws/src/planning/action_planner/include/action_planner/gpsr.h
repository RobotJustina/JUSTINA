#include "robot_service_manager/navigationtasks.h"
#include "robot_service_manager/speechgeneratortasks.h"
#include "robot_service_manager/robotarmtasks.h"
#include "action_planner/states_machines.h"
#include "ros/ros.h"
#include "planning_msgs/PlanningCmdClips.h"
#include "planning_msgs/planning_cmd.h"

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <boost/algorithm/string.hpp>

#include <iostream>

#include <tf/transform_listener.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

using namespace boost::algorithm;

class GPSRSM
{
public:
	/*
	*	ADD THE STATES ID YOU NEED FOR EACH STATE FUNCTION (IF YOU WANT)
	*/
	enum States
	{
		InitialState,
		FinalState
	};

	//for the SM api
	StatesMachines SM;
	//stpln
	static ServiceManager srv_man;
	static ros::NodeHandle * nh;
	static std::string classPrompt;
	static ros::Publisher command_response_pub;

	static ros::Subscriber subCmdSpeech;
	static ros::Subscriber subCmdInterpret;
	static ros::Subscriber subCmdConfirmation;
	static ros::Subscriber subCmdGetTasks;
	static ros::Subscriber subCmdNavigation;
	static ros::Subscriber subCmdAnswer;
	static ros::Subscriber subCmdFindObject;
	static ros::Subscriber subCmdAskFor;
	static ros::Subscriber subCmdStatusObject;
	static ros::Subscriber subCmdUnknown;

	static ros::ServiceClient srvCltInterpreter;
	static ros::ServiceClient srvCltGetTasks;
	static ros::ServiceClient srvCltWaitConfirmation;
	static ros::ServiceClient srvCltWaitForCommand;
	
	static ros::ServiceClient srvCltGetObject;
	static ros::ServiceClient srvCltHandOverObject;
	static ros::ServiceClient srvCltPlaceObject;
	static ros::ServiceClient srvCltFindPerson;
	static ros::ServiceClient srvCltNavigation;
	static ros::ServiceClient srvCltAnswer;

	static NavigationTasks navTasks;
	static RobotArmTasks armTasks;
	
	static int initialState();
	static int finalState();

	static void callbackCmdSpeech(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdInterpret(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdConfirmation(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdGetTasks(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdNavigation(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdAnswer(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdFindObject(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackAskFor(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackStatusObject(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackUnknown(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	
	/**********************************************************************/
	
	/*
	* A particular constructor for your state machine
	* Initialize your state machine here (add states, define the final state, define the execution method, etc)
	*/
	GPSRSM(ros::NodeHandle* n);
	bool execute();


private:
	static bool findObject(planning_msgs::PlanningCmdClips cmd);
	static planning_msgs::planning_cmd findPerson();
	static ros::Subscriber subsToRobotPositionSim;


};

ros::NodeHandle * GPSRSM::nh;
std::string GPSRSM::classPrompt;
ros::Publisher GPSRSM::command_response_pub;
ServiceManager GPSRSM::srv_man;
NavigationTasks GPSRSM::navTasks;
RobotArmTasks GPSRSM::armTasks;

ros::Subscriber GPSRSM::subCmdSpeech;
ros::Subscriber GPSRSM::subCmdInterpret;
ros::Subscriber GPSRSM::subCmdConfirmation;
ros::Subscriber GPSRSM::subCmdGetTasks;
ros::Subscriber GPSRSM::subCmdNavigation;
ros::Subscriber GPSRSM::subCmdAnswer;
ros::Subscriber GPSRSM::subCmdFindObject;
ros::Subscriber GPSRSM::subCmdAskFor;
ros::Subscriber GPSRSM::subCmdStatusObject;
ros::Subscriber GPSRSM::subCmdUnknown;

ros::ServiceClient GPSRSM::srvCltGetTasks;
ros::ServiceClient GPSRSM::srvCltInterpreter;
ros::ServiceClient GPSRSM::srvCltWaitConfirmation;
ros::ServiceClient GPSRSM::srvCltWaitForCommand;

ros::ServiceClient GPSRSM::srvCltGetObject;
ros::ServiceClient GPSRSM::srvCltHandOverObject;
ros::ServiceClient GPSRSM::srvCltFindPerson;
ros::ServiceClient GPSRSM::srvCltNavigation;
ros::ServiceClient GPSRSM::srvCltAnswer;

ros::ServiceClient GPSRSM::srvCltPlaceObject;

int GPSRSM::initialState()
{
	return (int)InitialState;
}

int GPSRSM::finalState()
{
	std::cout << classPrompt << "FinalState reached" << std::endl;
	ros::service::waitForService("spg_say", 5000); //This service is optional for this test
	srv_man.spgenSay("I have finished the gpsr test", 7000);
	return (int)FinalState;
}

void GPSRSM::callbackCmdSpeech(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << classPrompt << "--------- Command Speech ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	bool success = srv_man.spgenSay("I am ready for a spoken command", 7000);
	success = success & ros::service::waitForService("/planning_clips/wait_command", 50000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	if(success){
		planning_msgs::planning_cmd srv;
		srv.request.name = "test_wait";
		srv.request.params = "Ready";
		if(srvCltWaitForCommand.call(srv)){
			std::cout << "Response of wait for command:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
		}
		else{
			std::cout << classPrompt << "Failed to call service of wait_command" << std::endl;
			responseMsg.successful = 0;
		}
		responseMsg.params = srv.response.args;
		responseMsg.successful = srv.response.success;
	}
	else{
		std::cout << classPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);
}

void GPSRSM::callbackCmdInterpret(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << classPrompt << "--------- Command interpreter ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("/planning_clips/interpreter", 5000);
	if(success){
		planning_msgs::planning_cmd srv;
		srv.request.name = "test_interprete";
		srv.request.params = "Ready to interpretation";
		if(srvCltInterpreter.call(srv)){
			std::cout << "Response of interpreter:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			responseMsg.params = srv.response.args;
		responseMsg.successful = srv.response.success;
		}
		else{
			std::cout << classPrompt << "Failed to call service of interpreter" << std::endl;
			responseMsg.successful = 0;
		}
	}
	else{
		std::cout << classPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);

}

void GPSRSM::callbackCmdConfirmation(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command confirmation ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
	bool success = ros::service::waitForService("spg_say", 5000);
	success = success & ros::service::waitForService("/planning_clips/confirmation", 5000);
	if(success){
		std::string to_spech = responseMsg.params;
		boost::replace_all(to_spech, "_", " ");
		std::stringstream ss;
		ss << "Do you want me " << to_spech;
		std::cout << "------------- to_spech: ------------------ " << ss.str() << std::endl;
		success = srv_man.spgenSay(ss.str(), 7000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));

		planning_msgs::planning_cmd srv;
		srv.request.name = "test_confirmation";
		srv.request.params = responseMsg.params;
		if(srvCltWaitConfirmation.call(srv)){
			std::cout << "Response of confirmation:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			if(srv.response.success)
				srv_man.spgenSay("Ok i start to make the command", 7000);
			else
				srv_man.spgenSay("Repeate the command please", 7000);

			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;
		}
		else{
			std::cout << classPrompt << "Failed to call service of confirmation" << std::endl;
			responseMsg.successful = 0;
			srv_man.spgenSay("Repeate the command please", 7000);
		}
	}
	else{
		std::cout << classPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);
}

void GPSRSM::callbackCmdGetTasks(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command get tasks ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("/planning_clips/get_task" ,5000);
	if(success){
		planning_msgs::planning_cmd srv;
		srv.request.name = "cmd_task";
		srv.request.params = "Test of get_task module";
		if(srvCltGetTasks.call(srv)){
			std::cout << "Response of get tasks:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;
		}
		else{
			std::cout << classPrompt << "Failed to call get tasks" << std::endl;
			responseMsg.successful = 0;
		}
	}
	else{
		std::cout << classPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);
}

void GPSRSM::callbackCmdNavigation(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << classPrompt << "--------- Command Navigation ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	ros::Rate rate(1);
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;

	bool success = true;

	if(tokens[1] == "person"){
		success = true;
	}
	else{
		std::cout << classPrompt << "Navigation to " << tokens[1] << std::endl;
		ss.str("");
		ss << "I will navigate to the " << tokens[1];
		success = srv_man.spgenSay(ss.str(), 7000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		success = navTasks.syncGetClose(tokens[1], 120000);
		ss.str("");
		if(success){
			ss << "I have reached the " << tokens[1];
			success = srv_man.spgenSay(ss.str(), 7000);
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}
		else{
			ss.str("");
			ss << "I cannot reached the " << tokens[1];
			success = srv_man.spgenSay(ss.str(), 7000);
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}
	}
	if(success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	command_response_pub.publish(responseMsg);
}

void GPSRSM::callbackCmdAnswer(const planning_msgs::PlanningCmdClips::ConstPtr& msg){

	std::cout << classPrompt << "--------- Command answer a question ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("spg_say", 5000);
	success = success & ros::service::waitForService("/planning_clips/answer", 5000);
	if(success){
		success = srv_man.spgenSay("I am waiting for the user question.", 7000);
		planning_msgs::planning_cmd srv;
		srvCltAnswer.call(srv);
		if(srv.response.success)
			success = srv_man.spgenSay(srv.response.args, 7000);
		else
			success = false;
	}
	else
		success = false;

	if(success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	command_response_pub.publish(responseMsg);
}

void GPSRSM::callbackCmdFindObject(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command find a object ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	ros::Rate rate(1);
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;

	bool success = ros::service::waitForService("spg_say" ,5000);
	ss << "I am going to find an object " << tokens[0];
	if(success){
		std::cout << classPrompt << "find: " << tokens[0] << std::endl;
		ss.str("");
		if(tokens[0] == "person")
			ss << "I am going to find a person ";
		else
			ss << "I am going to find an object " <<  tokens[0];
		
		success = srv_man.spgenSay(ss.str(), 7000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		ss.str("");
		if(tokens[0] == "person")
			ss << "I have found a person";
		else
			ss << "I have found an object" << tokens[0];
		success = srv_man.spgenSay(ss.str(), 7000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		ss.str("");
		ss << responseMsg.params << " " << 1 << " " << 2 << " " << 3;
		responseMsg.params = ss.str();
	}
	if(success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	command_response_pub.publish(responseMsg);
}


void GPSRSM::callbackAskFor(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command Ask for ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::stringstream ss;
	ss << responseMsg.params << " " << "table";
	responseMsg.params = ss.str();
	responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

void GPSRSM::callbackStatusObject(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command Status object ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::stringstream ss;
	ss << responseMsg.params << " " << "open";
	responseMsg.params = ss.str();
	responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

void GPSRSM::callbackUnknown(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command unknown ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

GPSRSM::GPSRSM(ros::NodeHandle* n)
{
	nh = n;
	//add states to the state machine
	SM.addState((int)InitialState, &initialState);
	SM.addState((int)FinalState, &finalState, true);

	navTasks.initRosConnection(n);
	armTasks.initRosConnection(n);

	srvCltGetTasks = n->serviceClient<planning_msgs::planning_cmd>("/planning_clips/get_task");
	srvCltInterpreter = n->serviceClient<planning_msgs::planning_cmd>("/planning_clips/interpreter");
	srvCltWaitConfirmation = n->serviceClient<planning_msgs::planning_cmd>("/planning_clips/confirmation");
	srvCltWaitForCommand = n->serviceClient<planning_msgs::planning_cmd>("/planning_clips/wait_command");
	srvCltAnswer = n->serviceClient<planning_msgs::planning_cmd>("/planning_clips/answer");

	//srvCltPlaceObject = n->serviceClient<planning_msgs::wait_for_place_object>("/simple_task_planner/wait_for_place_object");

	subCmdSpeech = n->subscribe("/planning_clips/cmd_speech", 1 , GPSRSM::callbackCmdSpeech);
	subCmdInterpret = n->subscribe("/planning_clips/cmd_int", 1 , GPSRSM::callbackCmdInterpret);
	subCmdConfirmation = n->subscribe("/planning_clips/cmd_conf", 1, GPSRSM::callbackCmdConfirmation);
	subCmdGetTasks = n->subscribe("/planning_clips/cmd_task", 1, GPSRSM::callbackCmdGetTasks);

	subCmdNavigation = n->subscribe("/planning_clips/cmd_goto", 1, GPSRSM::callbackCmdNavigation);
	subCmdAnswer = n->subscribe("/planning_clips/cmd_answer", 1, GPSRSM::callbackCmdAnswer);
	subCmdFindObject = n->subscribe("/planning_clips/cmd_find_object", 1, GPSRSM::callbackCmdFindObject);
	subCmdAskFor = n->subscribe("/planning_clips/cmd_ask_for", 1, GPSRSM::callbackAskFor);
	subCmdStatusObject = n->subscribe("/planning_clips/cmd_status_object", 1, GPSRSM::callbackStatusObject);
	subCmdUnknown = n->subscribe("/planning_clips/cmd_unknown", 1, GPSRSM::callbackUnknown);

	command_response_pub = n->advertise<planning_msgs::PlanningCmdClips>("/planning_clips/command_response", 1);

}

bool GPSRSM::execute()
{
	ros::Rate loop(2);
	while(SM.runNextStep() && ros::ok())
	{
		loop.sleep();
		ros::spinOnce();
	}
	return true;
}