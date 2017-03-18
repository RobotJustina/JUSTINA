#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "planning_msgs/PlanningCmdClips.h"
#include "planning_msgs/planning_cmd.h"

#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaManip.h"

#include <vector>
#include <ctime>

using namespace boost::algorithm;

enum SMState {
	SM_INIT,
	SM_SAY_WAIT_FOR_DOOR,
	SM_WAIT_FOR_DOOR,
	SM_NAVIGATE_TO_THE_LOCATION,
	SM_SEND_INIT_CLIPS,
	SM_RUN_SM_CLIPS
};

ros::Publisher command_response_pub;
std::string testPrompt;
SMState state = SM_INIT;
bool runSMCLIPS = false;
bool startSignalSM = false;
planning_msgs::PlanningCmdClips initMsg;

// This is for the attemps for a actions
std::string lastCmdName = "";
int numberAttemps = 0;

ros::ServiceClient srvCltGetTasks;
ros::ServiceClient srvCltInterpreter;
ros::ServiceClient srvCltWaitConfirmation;
ros::ServiceClient srvCltWaitForCommand;
ros::ServiceClient srvCltAnswer;

void validateAttempsResponse(planning_msgs::PlanningCmdClips msg) {
	lastCmdName = msg.name;
	if (msg.successful == 0
			&& (msg.name.compare("move_actuator") == 0
					|| msg.name.compare("find_object") == 0
					|| msg.name.compare("status_object") == 0)) {
		if (msg.name.compare(lastCmdName) != 0)
			numberAttemps = 0;
		else if (numberAttemps == 3) {
			msg.successful = 1;
			numberAttemps = 0;
		} else
			numberAttemps++;
	}
	command_response_pub.publish(msg);
}

void callbackCmdSpeech(const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Speech ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	bool success = true;
	startSignalSM = true;

	if (!runSMCLIPS)
		success = false;

	success = success
			& ros::service::waitForService("/planning_clips/wait_command",
					50000);
	if (success) {
		planning_msgs::planning_cmd srv;
		srv.request.name = "test_wait";
		srv.request.params = "Ready";
		if (srvCltWaitForCommand.call(srv)) {
			std::cout << "Response of wait for command:" << std::endl;
			std::cout << "Success:" << (long int) srv.response.success
					<< std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
		} else {
			std::cout << testPrompt << "Failed to call service of wait_command"
					<< std::endl;
			responseMsg.successful = 0;
		}
		responseMsg.params = srv.response.args;
		responseMsg.successful = srv.response.success;
	} else {
		if (!runSMCLIPS) {
			initMsg = responseMsg;
			return;
		}
		std::cout << testPrompt << "Needed services are not available :'("
				<< std::endl;
		responseMsg.successful = 0;
	}
	if (runSMCLIPS) {
		validateAttempsResponse(responseMsg);
		//command_response_pub.publish(responseMsg);
	}
}

void callbackCmdInterpret(
		const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command interpreter ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("/planning_clips/interpreter",
			5000);
	if (success) {
		planning_msgs::planning_cmd srv;
		srv.request.name = "test_interprete";
		srv.request.params = "Ready to interpretation";
		if (srvCltInterpreter.call(srv)) {
			std::cout << "Response of interpreter:" << std::endl;
			std::cout << "Success:" << (long int) srv.response.success
					<< std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;
		} else {
			std::cout << testPrompt << "Failed to call service of interpreter"
					<< std::endl;
			responseMsg.successful = 0;
		}
	} else {
		std::cout << testPrompt << "Needed services are not available :'("
				<< std::endl;
		responseMsg.successful = 0;
	}
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);

}

void callbackCmdConfirmation(
		const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command confirmation ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("spg_say", 5000);
	success = success
			& ros::service::waitForService("/planning_clips/confirmation",
					5000);
	if (success) {
		std::string to_spech = responseMsg.params;
		boost::replace_all(to_spech, "_", " ");
		std::stringstream ss;
		ss << "Do you want me " << to_spech;
		std::cout << "------------- to_spech: ------------------ " << ss.str()
				<< std::endl;

		JustinaHRI::waitAfterSay(ss.str(), 2500);

		planning_msgs::planning_cmd srv;
		srv.request.name = "test_confirmation";
		srv.request.params = responseMsg.params;
		if (srvCltWaitConfirmation.call(srv)) {
			std::cout << "Response of confirmation:" << std::endl;
			std::cout << "Success:" << (long int) srv.response.success
					<< std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			if (srv.response.success)
				JustinaHRI::waitAfterSay("Ok i start to make the command",
						2000);
			else
				JustinaHRI::waitAfterSay("Repeate the command please", 2000);

			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;
		} else {
			std::cout << testPrompt << "Failed to call service of confirmation"
					<< std::endl;
			responseMsg.successful = 0;
			JustinaHRI::waitAfterSay("Repeate the command please", 2000);
		}
	} else {
		std::cout << testPrompt << "Needed services are not available :'("
				<< std::endl;
		responseMsg.successful = 0;
	}
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackCmdGetTasks(const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command get tasks ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("/planning_clips/get_task",
			5000);
	if (success) {
		planning_msgs::planning_cmd srv;
		srv.request.name = "cmd_task";
		srv.request.params = "Test of get_task module";
		if (srvCltGetTasks.call(srv)) {
			std::cout << "Response of get tasks:" << std::endl;
			std::cout << "Success:" << (long int) srv.response.success
					<< std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;
		} else {
			std::cout << testPrompt << "Failed to call get tasks" << std::endl;
			responseMsg.successful = 0;
		}
	} else {
		std::cout << testPrompt << "Needed services are not available :'("
				<< std::endl;
		responseMsg.successful = 0;
	}
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackCmdNavigation(
		const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Navigation ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));

	bool success = true;

	if (tokens[1] == "person") {
		success = true;
	} else {
		success = JustinaTasks::sayAndSyncNavigateToLoc(tokens[1], 120000);
	}
	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackCmdAnswer(const planning_msgs::PlanningCmdClips::ConstPtr& msg) {

	std::cout << testPrompt << "--------- Command answer a question ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::stringstream ss;
	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));

	bool success = ros::service::waitForService("spg_say", 5000);
	if (success) {
		std::string param1 = tokens[1];
		if (param1.compare("a_question") == 0) {
			success = ros::service::waitForService("/planning_clips/answer",
					5000);
			if (success) {
				success = JustinaHRI::waitAfterSay(
						"I am waiting for the user question", 2000);
				planning_msgs::planning_cmd srv;
				srvCltAnswer.call(srv);
				if (srv.response.success)
					success = JustinaHRI::waitAfterSay(srv.response.args, 2000);
				else
					success = false;
			}
		} else if (param1.compare("your_name") == 0) {
			JustinaHRI::waitAfterSay("Hellow my name is justina", 2000);
		} else if (param1.compare("your_team_name") == 0
				|| param1.compare("the_name_of_your_team") == 0) {
			JustinaHRI::waitAfterSay("Hello my team is pumas", 2000);
		} else if (param1.compare("introduce_yourself") == 0 || param1.compare("something_about_yourself") == 0) {
			JustinaHRI::waitAfterSay("Hello my name is justina", 2000);
			JustinaHRI::waitAfterSay("i am from Mexico city", 2000);
			JustinaHRI::waitAfterSay("my team is pumas", 2000);
			JustinaHRI::waitAfterSay(
					"of the national autonomous university of mexico", 2000);
		} else if (param1.compare("the_day") == 0
				|| param1.compare("the_time") == 0) {
			ss.str("");
			//std::locale::global(std::locale("de_DE.utf8"));
			//std::locale::global(std::locale("en_us.utf8"));
			time_t now = time(0);
			char* dt = ctime(&now);
			std::cout << "Day:" << dt << std::endl;
			JustinaHRI::waitAfterSay(dt, 2000);
		} else if (param1.compare("what_time_is_it") == 0) {
			ss.str("");
			std::time_t now = time(0);
			std::tm *ltm = localtime(&now);
			ss << "The time is " << ltm->tm_hour << " " << ltm->tm_min;
			JustinaHRI::waitAfterSay(ss.str(), 2000);
		} else if (param1.compare("what_day_is_tomorrow") == 0) {
			std::time_t now = time(0);
			std::tm *ltmnow = localtime(&now);
			std::cout << "Curr day :" << ltmnow->tm_mday << std::endl;
			ltmnow->tm_mday = ltmnow->tm_mday + 1;
			std::cout << "Tomorrow day :" << ltmnow->tm_mday << std::endl;
			std::time_t tomorrow = std::mktime(ltmnow);
			char* dt = ctime(&tomorrow);
			std::cout << "Tomorrow format :" << dt << std::endl;
			JustinaHRI::waitAfterSay(dt, 2000);
		}else if (param1.compare("the_day_of_the_week") == 0){
			std::time_t now = time(0);
			std::tm *ltmnow = localtime(&now);
			std::cout << "Curr day :" << ltmnow->tm_wday << std::endl;
			std::cout << "The day of week:" << ltmnow->tm_wday << std::endl;
			std::time_t day_week = std::mktime(ltmnow);
			std::cout << "Week day format :" << ltmnow->tm_wday << std::endl;
			if(ltmnow->tm_wday == 0)
				JustinaHRI::waitAfterSay("The day of the week is sunday", 2000);
			else if(ltmnow->tm_wday == 1)
				JustinaHRI::waitAfterSay("The day of the week is monday", 2000);
			else if(ltmnow->tm_wday == 2)
				JustinaHRI::waitAfterSay("The day of the week is tuesday", 2000);
			else if(ltmnow->tm_wday == 3)
				JustinaHRI::waitAfterSay("The day of the week is wednesday", 2000);
			else if(ltmnow->tm_wday == 4)
				JustinaHRI::waitAfterSay("The day of the week is thursday", 2000);
			else if(ltmnow->tm_wday == 5)
				JustinaHRI::waitAfterSay("The day of the week is friday", 2000);
			else if(ltmnow->tm_wday == 6)
				JustinaHRI::waitAfterSay("The day of the week is saturday", 2000);

		}else if (param1.compare("the_day_of_the_month") == 0) {
			ss.str("");
			//std::locale::global(std::locale("de_DE.utf8"));
			time_t now = time(0);
			char* dt = ctime(&now);
			std::cout << "Day:" << dt << std::endl;
			JustinaHRI::waitAfterSay(dt, 2000);
		}else if (param1.compare("a_joke") == 0) {
			ss.str("");
			JustinaHRI::waitAfterSay("I am going to say a joke", 2000);
			JustinaHRI::waitAfterSay("What is the longest word in the English language", 2000);
			JustinaHRI::waitAfterSay("SMILES, there is a mile between the first and last letters", 2000);
		}
	} else
		success = false;

	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackCmdFindObject(
		const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command find a object ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;

	bool success = ros::service::waitForService("spg_say", 5000);
	if (success) {
		std::cout << testPrompt << "find: " << tokens[0] << std::endl;

		ss.str("");
		if (tokens[0] == "person") {
			success = JustinaTasks::findPerson();
			ss << responseMsg.params << " " << 1 << " " << 1 << " " << 1;
		} else if (tokens[0] == "man") {
			success = JustinaTasks::findAndFollowPersonToLoc(tokens[1]);
			ss << responseMsg.params;
		} else if (tokens[0] == "specific") {
			success = JustinaTasks::findPerson();//success = JustinaTasks::findPerson(tokens[1])
			ss << "find_spc_person " << tokens[0] << " " << tokens[1];//ss << responseMsg.params;
		} else {
			geometry_msgs::Pose pose;
			bool withLeftOrRightArm;
			success = JustinaTasks::findObject(tokens[0], pose, withLeftOrRightArm);
			ss << responseMsg.params << " " << pose.position.x << " "
					<< pose.position.y << " " << pose.position.z;
		}
		responseMsg.params = ss.str();
	}
	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackAskFor(const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Ask for ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	/*std::stringstream ss;
	 ss << responseMsg.params << " " << "table";
	 responseMsg.params = ss.str();*/
	responseMsg.successful = 1;
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackStatusObject(
		const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Status object ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::stringstream ss;
	ss << responseMsg.params << " " << "open";

	bool success = JustinaTasks::alignWithTable(0.35);

	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;

	responseMsg.params = ss.str();
	responseMsg.successful = 1;
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackMoveActuator(
		const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Move actuator ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));

	bool success = ros::service::waitForService("spg_say", 5000);
	//success = success & tasks.moveActuator(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()), tokens[0]);
	success = success
			& JustinaTasks::moveActuatorToGrasp(atof(tokens[1].c_str()),
					atof(tokens[2].c_str()), atof(tokens[3].c_str()), false,
					tokens[0]);
	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;

	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackDrop(const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Drop ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool succes = JustinaTasks::dropObject();

	if (succes)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;

	validateAttempsResponse(responseMsg);
}

void callbackUnknown(const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command unknown ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	responseMsg.successful = 1;
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}


void callbackAskPerson(
		const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command ask for person ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("spg_say", 5000);
	success = success
			& ros::service::waitForService("/planning_clips/confirmation",
					5000);
	JustinaManip::startHdGoTo(0, 0.0);
	if (success) {
		std::string to_spech = responseMsg.params;
		boost::replace_all(to_spech, "_", " ");
		std::stringstream ss;
		ss << "Hello, your name is " << to_spech;
		std::cout << "------------- to_spech: ------------------ " << ss.str()
				<< std::endl;

		JustinaHRI::waitAfterSay(ss.str(), 2000);

		planning_msgs::planning_cmd srv;
		srv.request.name = "test_confirmation";
		srv.request.params = responseMsg.params;
		if (srvCltWaitConfirmation.call(srv)) {
			std::cout << "Response of confirmation:" << std::endl;
			std::cout << "Success:" << (long int) srv.response.success
					<< std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			if (srv.response.success){
				ss.str("");
				ss << "Hello " << to_spech;
				JustinaHRI::waitAfterSay(ss.str(),1500);
				
			}
			else{
				ss.str("");
				ss << to_spech << ", I try to find you again ";
				JustinaHRI::waitAfterSay(ss.str(), 1500);
			}

			responseMsg.params = responseMsg.params;//srv.response.args;
			responseMsg.successful = srv.response.success;
		} else {
			std::cout << testPrompt << "Failed to call service of confirmation"
					<< std::endl;
			responseMsg.successful = 0;
			JustinaHRI::waitAfterSay("Repeate the command please", 2000);
		}
	} else {
		std::cout << testPrompt << "Needed services are not available :'("
				<< std::endl;
		responseMsg.successful = 0;
	}
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "gpsr_test");
	ros::NodeHandle n;
	ros::Rate rate(10);

	srvCltGetTasks = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_clips/get_task");
	srvCltInterpreter = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_clips/interpreter");
	srvCltWaitConfirmation = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_clips/confirmation");
	srvCltWaitForCommand = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_clips/wait_command");
	srvCltAnswer = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_clips/answer");

	ros::Subscriber subCmdSpeech = n.subscribe("/planning_clips/cmd_speech", 1,
			callbackCmdSpeech);
	ros::Subscriber subCmdInterpret = n.subscribe("/planning_clips/cmd_int", 1,
			callbackCmdInterpret);
	ros::Subscriber subCmdConfirmation = n.subscribe("/planning_clips/cmd_conf",
			1, callbackCmdConfirmation);
	ros::Subscriber subCmdGetTasks = n.subscribe("/planning_clips/cmd_task", 1,
			callbackCmdGetTasks);

	ros::Subscriber subCmdNavigation = n.subscribe("/planning_clips/cmd_goto",
			1, callbackCmdNavigation);
	ros::Subscriber subCmdAnswer = n.subscribe("/planning_clips/cmd_answer", 1,
			callbackCmdAnswer);
	ros::Subscriber subCmdFindObject = n.subscribe(
			"/planning_clips/cmd_find_object", 1, callbackCmdFindObject);
	ros::Subscriber subCmdAskFor = n.subscribe("/planning_clips/cmd_ask_for", 1,
			callbackAskFor);
	ros::Subscriber subCmdStatusObject = n.subscribe(
			"/planning_clips/cmd_status_object", 1, callbackStatusObject);
	ros::Subscriber subCmdMoveActuator = n.subscribe(
			"/planning_clips/cmd_move_actuator", 1, callbackMoveActuator);
	ros::Subscriber subCmdDrop = n.subscribe("/planning_clips/cmd_drop", 1,
			callbackDrop);
	ros::Subscriber subCmdUnknown = n.subscribe("/planning_clips/cmd_unknown",
			1, callbackUnknown);
	ros::Subscriber subAskPerson = n.subscribe("/planning_clips/cmd_ask_person",
			1, callbackAskPerson);

	command_response_pub = n.advertise<planning_msgs::PlanningCmdClips>(
			"/planning_clips/command_response", 1);

	JustinaHRI::setNodeHandle(&n);
	JustinaHardware::setNodeHandle(&n);
	JustinaKnowledge::setNodeHandle(&n);
	JustinaManip::setNodeHandle(&n);
	JustinaNavigation::setNodeHandle(&n);
	JustinaTasks::setNodeHandle(&n);
	JustinaTools::setNodeHandle(&n);
	JustinaVision::setNodeHandle(&n);

	while (ros::ok()) {

		switch (state) {
		case SM_INIT:
			if (startSignalSM) {
				JustinaHRI::waitAfterSay("I am ready for the gpsr test", 4000);
				state = SM_SAY_WAIT_FOR_DOOR;
			}
			break;
		case SM_SAY_WAIT_FOR_DOOR:
			JustinaHRI::waitAfterSay("I am waiting for the door to be open",
					4000);
			state = SM_WAIT_FOR_DOOR;
			break;
		case SM_WAIT_FOR_DOOR:
			if (!JustinaNavigation::obstacleInFront())
				state = SM_NAVIGATE_TO_THE_LOCATION;
			break;
		case SM_NAVIGATE_TO_THE_LOCATION:
			JustinaHRI::waitAfterSay("I can see now that the door is open",
					4000);
			std::cout << "GPSRTest.->First try to move" << std::endl;
			if (!JustinaTasks::sayAndSyncNavigateToLoc("arena", 120000)) {
				std::cout << "GPSRTest.->Second try to move" << std::endl;
				if (!JustinaTasks::sayAndSyncNavigateToLoc("arena", 120000)) {
					std::cout << "GPSRTest.->Third try to move" << std::endl;
					if (JustinaTasks::sayAndSyncNavigateToLoc("arena",
							120000)) {
						JustinaHRI::waitAfterSay(
								"I am ready for a spoken command", 4000);
						state = SM_SEND_INIT_CLIPS;
					}
				} else {
					JustinaHRI::waitAfterSay("I am ready for a spoken command",
							4000);
					state = SM_SEND_INIT_CLIPS;
				}
			} else {
				JustinaHRI::waitAfterSay("I am ready for a spoken command",
						4000);
				state = SM_SEND_INIT_CLIPS;
			}
			break;
		case SM_SEND_INIT_CLIPS:
			JustinaVision::startQRReader();
			initMsg.successful = false;
			runSMCLIPS = true;
			command_response_pub.publish(initMsg);
			state = SM_RUN_SM_CLIPS;
			break;
		case SM_RUN_SM_CLIPS:
			break;
		}

		rate.sleep();
		ros::spinOnce();
	}

	JustinaVision::stopQRReader();

	return 0;

}
