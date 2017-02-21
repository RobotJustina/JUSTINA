#include "ros/ros.h"

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

using namespace boost::algorithm;

enum SMState {
	SM_INIT,
	SM_SAY_WAIT_FOR_DOOR,
	SM_WAIT_FOR_DOOR,
	SM_NAVIGATE_TO_THE_LOCATION,
	SM_SEND_INIT_CLIPS,
	SM_RUN_SM_CLIPS
};

std::vector<std::string> objectsids;
ros::Publisher command_response_pub;
SMState state;
std::string testPrompt;
bool hasBeenInit;

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
ros::ServiceClient srvCltWhatSee;
ros::ServiceClient srvCltExplain;
ros::ServiceClient srvCltDisponible;

void validateAttempsResponse(planning_msgs::PlanningCmdClips msg) {
	lastCmdName = msg.name;
	if (msg.successful == 0
			&& (msg.name.compare("move_actuator") == 0
					|| msg.name.compare("find_object") == 0)) {
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

	if (success) {
		responseMsg.successful = 1;
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

	bool success = ros::service::waitForService(
			"/planning_open_challenge/interpreter", 5000);
	if (success) {
		planning_msgs::planning_cmd srv;
		srv.request.name = "test_interprete";
		srv.request.params = "Ready to interpretation";
		if (srvCltInterpreter.call(srv)) {
			std::cout << "Response of interpreter:" << std::endl;
			std::cout << "Success:" << (long int) srv.response.success
					<< std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			//responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;

			if (srv.response.success == 1) {
				std::string to_spech = srv.response.args;
				boost::replace_all(to_spech, "_", " ");
				std::stringstream ss;

				std::vector<std::string> tokens;
				std::string str = to_spech;
				split(tokens, str, is_any_of(" "));
				if (tokens.size() >= 7)
					ss << srv.response.args << " " << tokens[2] << " "
							<< tokens[7];
				else
					responseMsg.successful = 0;
				responseMsg.params = ss.str();
			}

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

void callbackCmdDisponible(
		const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Allowed ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = msg->params;
	split(tokens, str, is_any_of(" "));

	responseMsg.successful = 1;

	if (tokens[0] == "nil" || tokens[0] == "droped") {

		bool success;
		success = ros::service::waitForService(
				"/planning_open_challenge/disponible", 5000);
		if (success) {
			std::cout << "------------- No Disponible: ------------------ "
					<< std::endl;

			if (tokens[2] == "found")
				JustinaTasks::sayAndSyncNavigateToLoc("inspection", 120000);

			planning_msgs::planning_cmd srv;
			srv.request.name = "test_disponible";
			srv.request.params = responseMsg.params;
			if (srvCltDisponible.call(srv)) {
				std::cout << "Response of confirmation:" << std::endl;
				std::cout << "Success:" << (long int) srv.response.success
						<< std::endl;
				std::cout << "Args:" << srv.response.args << std::endl;
				if (tokens[0] == "nil")
					JustinaHRI::waitAfterSay("the object is not on the table",1000);
				else{
					JustinaHRI::waitAfterSay(tokens[3], 1000);
					JustinaHRI::waitAfterSay("have the object", 1000);}
				JustinaHRI::waitAfterSay("Would you like something else", 1000);
				responseMsg.successful = 0;
			} else {
				std::cout << testPrompt
						<< "Failed to call service of confirmation"
						<< std::endl;
				responseMsg.successful = 0;
			}

		} else {
			std::cout << testPrompt << "Needed services are not available :'("
					<< std::endl;
			responseMsg.successful = 0;
		}
	}
	validateAttempsResponse(responseMsg);
}

void callbackCmdHappen(const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Happen ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = msg->params;
	split(tokens, str, is_any_of(" "));

	responseMsg.successful = 1;

	if (tokens[1] == "nil") {
		JustinaHRI::waitAfterSay("Some one else take the object", 1000);
		JustinaHRI::waitAfterSay("Would you like something else", 1000);
		responseMsg.successful = 0;
		responseMsg.params = "obj prs fuente";
	}

	else if (tokens[1] == "open_table") {
		JustinaHRI::waitAfterSay("The object remaince on the table", 1000);
		JustinaHRI::waitAfterSay("Would you like something else", 1000);
		responseMsg.successful = 0;
		responseMsg.params = "obj prs fuente";
	}
	
	 else {
		std::cout << testPrompt << "Needed services are not available :'("
				<< std::endl;
		responseMsg.successful = 0;
		responseMsg.params = "obj prs fuente";
	}
	validateAttempsResponse(responseMsg);
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
			& ros::service::waitForService(
					"/planning_open_challenge/confirmation", 5000);

	if (success) {

		std::string to_spech = responseMsg.params;
		boost::replace_all(to_spech, "_", " ");
		std::stringstream ss;

		ss << "Do you want me " << to_spech;
		std::cout << "------------- to_spech: ------------------ " << ss.str()
				<< std::endl;
		JustinaHRI::waitAfterSay(ss.str(), 1000);

		planning_msgs::planning_cmd srv;
		srv.request.name = "test_confirmation";
		srv.request.params = responseMsg.params;
		if (srvCltWaitConfirmation.call(srv)) {
			std::cout << "Response of confirmation:" << std::endl;
			std::cout << "Success:" << (long int) srv.response.success
					<< std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			if (srv.response.success)
				JustinaHRI::waitAfterSay("would you like something else", 1000);
			else
				JustinaHRI::waitAfterSay("Repeate the command please", 1000);

			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;
		} else {
			std::cout << testPrompt << "Failed to call service of confirmation"
					<< std::endl;
			responseMsg.successful = 0;
			JustinaHRI::waitAfterSay("Repeate the command please", 1000);
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

	bool success = ros::service::waitForService(
			"/planning_open_challenge/get_task", 5000);
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

void callbackCmdExplainThePlan(
		const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Explain the plan ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	bool explain;

	bool success2 = ros::service::waitForService(
			"/planning_open_challenge/what_see", 5000);
	if (success2) {

		planning_msgs::planning_cmd srv2;
		srv2.request.name = "test_what_see";
		srv2.request.params = responseMsg.params;

		if (srvCltWhatSee.call(srv2)) {

			if (srv2.response.args == "explain") {
				JustinaHRI::waitAfterSay("I am going to explain the plan",
						1000);
				explain = true;
			} else {
				JustinaHRI::waitAfterSay("I start to execute the plan", 1000);
				explain = false;
			}
		}

		else {
			std::cout << testPrompt << "Failed to call service explain plan"
					<< std::endl;
			responseMsg.successful = 0;
		}
	} else {
		std::cout << testPrompt << "Needed services are not available :'("
				<< std::endl;
		responseMsg.successful = 0;
	}

	bool success = ros::service::waitForService(
			"/planning_open_challenge/plan_explain", 5000);
	if (success) {
		bool finish = false;
		do {
			planning_msgs::planning_cmd srv;
			srv.request.name = "cmd_explain";
			srv.request.params = "Test of cmd_explain module";
			if (srvCltExplain.call(srv)) {
				std::cout << "Response of explain plan:" << std::endl;
				std::cout << "Success:" << (long int) srv.response.success
						<< std::endl;
				std::cout << "Args:" << srv.response.args << std::endl;
				if (!srv.response.success)
					finish = true;
				else {

					std::stringstream ss;
					std::vector<std::string> tokens;
					std::string str = srv.response.args;
					split(tokens, str, is_any_of(" "));

					/*if(tokens[0].compare("find_person_in_room") == 0){
					 ss << "I have to find a person "  <<
					 tasks.syncSpeech();
					 }*/
					/*else if(tokens[0].compare("wait_for_user_instruction") == 0){
					 tasks.syncSpeech();
					 }*/
					std::string param1 = tokens[2];
					std::string param2 = tokens[3];

					if (param1.compare("update_object_location") == 0
							&& explain) {
						ss.str("");
						ss << "I have to locate the " << param2
								<< " on the table";
						//tasks.syncSpeech(ss.str(), 30000, 2000);
					} else if (param1.compare("get_object") == 0 && explain) {
						ss.str("");
						ss << "First I have to align with table";
						JustinaHRI::waitAfterSay(ss.str(), 1000);
						ss.str("");
						ss << "After I have to find the " << param2;
						JustinaHRI::waitAfterSay(ss.str(), 1000);
						ss.str("");
						ss << "So I have to grasp the " << param2;
						JustinaHRI::waitAfterSay(ss.str(), 1000);
					} else if (param1.compare("find_person_in_room") == 0
							&& explain) {
						ss.str("");
						ss << "After I have to look for " << param2;
						JustinaHRI::waitAfterSay(ss.str(), 1000);
						ss.str("");
						ss << "And approach to him";
						JustinaHRI::waitAfterSay(ss.str(), 1000);
					} else if (param1.compare("handover_object") == 0
							&& explain) {
						ss.str("");
						ss << "Then I have to verify the person is before me";
						JustinaHRI::waitAfterSay(ss.str(), 1000);
						ss.str("");
						ss << "Finally I have to deliver the " << param2;
						JustinaHRI::waitAfterSay(ss.str(), 1000);
					}
				}
			} else {
				std::cout << testPrompt << "Failed to call get tasks"
						<< std::endl;
				responseMsg.successful = 0;
			}
			responseMsg.successful = 1;
		} while (ros::ok() && !finish);
	} else {
		std::cout << testPrompt << "Needed services are not available :'("
				<< std::endl;
		responseMsg.successful = 0;
	}
	responseMsg.name = "cmd_explain";
	responseMsg.params = "open 3";
	responseMsg.id = msg->id;
	responseMsg.successful = 1;
	validateAttempsResponse(responseMsg);
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
						"I am waiting for the user question", 1000);
				planning_msgs::planning_cmd srv;
				srvCltAnswer.call(srv);
				if (srv.response.success)
					success = JustinaHRI::waitAfterSay(srv.response.args, 1000);
				else
					success = false;
			}
		} else if (param1.compare("your_name") == 0) {
			JustinaHRI::waitAfterSay("Hellow my name is justina", 1000);
		} else if (param1.compare("your_team_name") == 0
				|| param1.compare("the_name_of_your_team") == 0) {
			JustinaHRI::waitAfterSay("Hello my team is pumas", 1000);
		} else if (param1.compare("introduce_yourself") == 0) {
			JustinaHRI::waitAfterSay("Hello my name is justina", 1000);
			JustinaHRI::waitAfterSay("i am from Mexico city", 1000);
			JustinaHRI::waitAfterSay("my team is pumas", 1000);
			JustinaHRI::waitAfterSay(
					"of the national autonomous university of mexico", 1000);
		} else if (param1.compare("the_day") == 0
				|| param1.compare("the_time") == 0) {
			ss.str("");
			//std::locale::global(std::locale("de_DE.utf8"));
			//std::locale::global(std::locale("en_us.utf8"));
			time_t now = time(0);
			char* dt = ctime(&now);
			std::cout << "Day:" << dt << std::endl;
			JustinaHRI::waitAfterSay(dt, 1000);
		} else if (param1.compare("what_time_is_it") == 0) {
			ss.str("");
			std::time_t now = time(0);
			std::tm *ltm = localtime(&now);
			ss << "The time is " << ltm->tm_hour << " " << ltm->tm_min;
			JustinaHRI::waitAfterSay(ss.str(), 1000);
		} else if (param1.compare("what_day_is_tomorrow") == 0) {
			std::time_t now = time(0);
			std::tm *ltmnow = localtime(&now);
			std::cout << "Curr day :" << ltmnow->tm_mday << std::endl;
			ltmnow->tm_mday = ltmnow->tm_mday + 1;
			std::cout << "Tomorrow day :" << ltmnow->tm_mday << std::endl;
			std::time_t tomorrow = std::mktime(ltmnow);
			char* dt = ctime(&tomorrow);
			std::cout << "Tomorrow format :" << dt << std::endl;
			JustinaHRI::waitAfterSay(dt, 1000);
		} else if (param1.compare("the_day_of_the_month") == 0) {
			ss.str("");
			//std::locale::global(std::locale("de_DE.utf8"));
			time_t now = time(0);
			char* dt = ctime(&now);
			std::cout << "Day:" << dt << std::endl;
			JustinaHRI::waitAfterSay(dt, 1000);
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

void callbackCmdWorld(const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command World ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	int robert = 0;
	int other = 0;
	int robertCI = 0;
	int robertCD = 0;
	int otherCI = 0;
	int otherCD = 0;
	int arthur = 0;
	int arthurCI = 0;
	int arthurCD = 0;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	planning_msgs::PlanningCmdClips responseModify;
	responseModify.name = "cmd_modify";
	responseModify.params = "modify";
	responseModify.id = msg->id;

	planning_msgs::PlanningCmdClips responseObject;
	responseObject.name = "cmd_wobj";
	responseObject.params = "wobj";
	responseObject.id = msg->id;

	std::stringstream ss;

	//responseMsg.successful = 1;
	//command_response_pub.publish(responseMsg);

	//bool success = ros::service::waitForService("spg_say", 5000);
	bool success = ros::service::waitForService(
			"/planning_open_challenge/what_see", 5000);
	if (success) {

		planning_msgs::planning_cmd srv;
		srv.request.name = "test_what_see";
		srv.request.params = responseMsg.params;

		if (srvCltWhatSee.call(srv)) {

			if (srv.response.args == "what_see_yes") {
				JustinaHRI::waitAfterSay(
						"I am going to search persons in the scene", 1000);

				/*tasks.waitHeadGoalPose(0.0, -0.7, 3000);
				 boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
				 tasks.waitHeadGoalPose(-0.6, 0.0, 3000);
				 boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
				 tasks.waitHeadGoalPose(0.6, 0.0, 3000);
				 boost::this_thread::sleep(boost::posix_time::milliseconds(4000));*/
				JustinaManip::hdGoTo(0, 0.0, 5000);
				/*JustinaNavigation::moveLateral(0.3, 4000);
				 boost::this_thread::sleep(boost::posix_time::milliseconds(6000));
				 JustinaNavigation::moveLateral(-0.3, 4000);
				 boost::this_thread::sleep(boost::posix_time::milliseconds(6000));*/
				///}
				JustinaVision::startFaceRecognition();
				bool recognized = false;
				float timeOut = 10000.0;
				std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;

				boost::posix_time::ptime curr;
				boost::posix_time::ptime prev =
						boost::posix_time::second_clock::local_time();
				boost::posix_time::time_duration diff;

				std::cout << "Response of what do you see:" << std::endl;
				std::cout << "Success:" << (long int) srv.response.success
						<< std::endl;
				std::cout << "Args:" << srv.response.args << std::endl;

				do {
					boost::this_thread::sleep(
							boost::posix_time::milliseconds(100));
					JustinaVision::facRecognize();
					JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);

					///El robot se mueve a una nueva posicion
					JustinaNavigation::moveLateral(0.3, 4000);
					boost::this_thread::sleep(
							boost::posix_time::milliseconds(6000));

					for (int i = 0; i < lastRecognizedFaces.size(); i++) {
						if (lastRecognizedFaces[i].id == "peter") {
							robert++;
							if (i == 0) {
								robertCI++;
							} else {
								robertCD++;
							}
						} else if (lastRecognizedFaces[i].id == "john") {
							arthur++;
							if (i == 0) {
								arthurCI++;
							} else {
								arthurCD++;
							}
						} else if (lastRecognizedFaces[i].id == "unknown") {
							other++;
							if (i == 0) {
								otherCI++;
							} else {
								otherCD++;
							}
						}
					}

					curr = boost::posix_time::second_clock::local_time();
					ros::spinOnce();
				} while (ros::ok()
						&& (curr - prev).total_milliseconds() < timeOut
						&& srv.response.args == "what_see_yes");

				JustinaManip::hdGoTo(0, -0.4, 5000);
				boost::this_thread::sleep(
						boost::posix_time::milliseconds(2000));
				JustinaManip::hdGoTo(0, 0.0, 5000);

				prev = boost::posix_time::second_clock::local_time();

				do {
					boost::this_thread::sleep(
							boost::posix_time::milliseconds(100));
					JustinaVision::facRecognize();
					JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);

					///El robot se mueve a una nueva posicion
					JustinaNavigation::moveLateral(-0.3, 4000);
					boost::this_thread::sleep(
							boost::posix_time::milliseconds(6000));
					//JustinaManip::hdGoTo(0, -0.4, 5000);

					for (int i = 0; i < lastRecognizedFaces.size(); i++) {
						if (lastRecognizedFaces[i].id == "peter") {
							robert++;
							if (i == 0) {
								robertCI++;
							} else {
								robertCD++;
							}
						} else if (lastRecognizedFaces[i].id == "john") {
							arthur++;
							if (i == 0) {
								arthurCI++;
							} else {
								arthurCD++;
							}
						} else if (lastRecognizedFaces[i].id == "unknown") {
							other++;
							if (i == 0) {
								otherCI++;
							} else {
								otherCD++;
							}
						}
					}

					curr = boost::posix_time::second_clock::local_time();
					ros::spinOnce();
				} while (ros::ok()
						&& (curr - prev).total_milliseconds() < timeOut
						&& srv.response.args == "what_see_yes");

				JustinaManip::hdGoTo(0, -0.4, 5000);
				boost::this_thread::sleep(
						boost::posix_time::milliseconds(2000));
				JustinaManip::hdGoTo(0, 0.0, 5000);
				//tasks.syncNavigate("open_table", 120000);
			}	///condicion de reconocimiento de rostros

			if (arthurCI != arthurCD && arthurCI > arthurCD && robert > 0) {
				std::cout << "John esta a la Izquerda" << std::endl;
				ss << "john izquierda";
				JustinaHRI::waitAfterSay("John is in the left", 1000);
			} else if (arthurCI != arthurCD && arthurCI < arthurCD
					&& robert > 0) {
				std::cout << "John esta a la Derecha" << std::endl;
				ss << "john derecha";
				JustinaHRI::waitAfterSay("john is in the right", 1000);
			} else {
				if (arthur > 0) {
					std::cout << "John esta SOLO" << std::endl;
					ss << "john solo";
					JustinaHRI::waitAfterSay(
							"john is the only person I can see", 1000);
				} else
					ss << "john nil";
			}

			if (robertCI != robertCD && robertCI > robertCD && arthur > 0) {
				std::cout << "Peter esta a la Izquerda" << std::endl;
				ss << " peter izquierda";
				JustinaHRI::waitAfterSay("Peter is in the left", 1000);
			} else if (robertCI != robertCD && robertCI < robertCD
					&& arthur > 0) {
				std::cout << "Peter esta a la Derecha" << std::endl;
				ss << " peter derecha";
				JustinaHRI::waitAfterSay("Peter is in the right", 1000);
			} else {
				if (robert > 0) {
					std::cout << "Peter esta SOLO" << std::endl;
					ss << " peter solo";
					JustinaHRI::waitAfterSay(
							"Peter is the only person I can see", 1000);
				} else {
					ss << " peter nil";
				}
			}

			std::string s = ss.str();
			responseModify.params = s;
			responseModify.successful = 1;
			if (srv.response.args == "what_see_yes") {
				command_response_pub.publish(responseModify);
			}

			//std::cout << "Vector: " << lastRecognizedFaces[0].id << std::endl;
			std::cout << "peter times: " << robert << std::endl;
			std::cout << "john times: " << arthur << std::endl;

			//std::cout << "unknown times: " << other << std::endl;

			std::cout << "peterIzquierda times: " << robertCI << std::endl;
			//std::cout << "unknowmIzquierda times: " << otherCI << std::endl;
			std::cout << "ArthurIzquierda times: " << arthurCI << std::endl;

			std::cout << "peterDerecha times: " << robertCD << std::endl;
			//std::cout << "unknownDerecha times: " << otherCD << std::endl;
			std::cout << "johnDerecha times: " << arthurCD << std::endl;

			/*if(lastRecognizedFaces.size()>0)
			 recognized = true;
			 else
			 recognized = false;*/

			//command_response_pub.publish(responseMsg);
			JustinaVision::stopFaceRecognition();
			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;

			if (srv.response.args == "what_see_yes") {

				//if(objectsids.size()>0)
				//	objectsids.erase(objectsids.begin());
				boost::this_thread::sleep(
						boost::posix_time::milliseconds(4000));
				JustinaTasks::sayAndSyncNavigateToLoc("open_table", 120000);

				JustinaHRI::waitAfterSay(
						"I am going to search objects on the table", 1000);
				JustinaManip::hdGoTo(0, -0.9, 5000);

				objectsids.clear();

				std::map<std::string, int> countObj;
				bool finishMotion = false, dir = true;
				float dis = 0.0, inc = 0.3;
				countObj["soup"] = 0;
				countObj["stevia"] = 0;
				countObj["milk"] = 0;
				countObj["juice"] = 0;

				do {
					boost::this_thread::sleep(
							boost::posix_time::milliseconds(3000));
					std::vector<vision_msgs::VisionObject> recognizedObjects;
					std::cout << "Find a object " << std::endl;
					bool found = 0;
					for (int j = 0; j < 10; j++) {
						std::cout << "Test object" << std::endl;
						found = JustinaVision::detectObjects(recognizedObjects);
						int indexFound = 0;
						if (found) {
							found = false;
							for (int i = 0; i < recognizedObjects.size(); i++) {
								vision_msgs::VisionObject vObject =
										recognizedObjects[i];
								std::cout << "object:  " << vObject.id
										<< std::endl;
								std::map<std::string, int>::iterator it =
										countObj.find(vObject.id);
								if (it != countObj.end())
									it->second = it->second + 1;
							}
						}
					}
					if (dir)
						dis += 0.4;
					else
						dis -= 0.8;
					JustinaNavigation::moveLateral(dis, 2000);
					if (dis >= 0.4)
						dir = false;
					if (dis <= -0.4)
						finishMotion = true;
				} while (!finishMotion);
				JustinaManip::hdGoTo(0, 0.0, 5000);
				responseObject.successful = 1;
				for (std::map<std::string, int>::iterator it = countObj.begin();
						it != countObj.end(); ++it) {
					std::stringstream ss;
					if (it->second > 0) {
						ss << it->first << " table";
						responseObject.params = ss.str();
						ss.str("");
						ss << it->first << " is on the table";
						JustinaHRI::waitAfterSay(ss.str(), 1000);
						command_response_pub.publish(responseObject);
						objectsids.push_back(it->first);
					} else {
						ss << it->first << " nil";
						responseObject.params = ss.str();
						command_response_pub.publish(responseObject);
					}
				}

				JustinaTasks::sayAndSyncNavigateToLoc("inspection", 120000);
			}				///termina recog objects

			if (srv.response.args == "what_see_yes") {
				JustinaHRI::waitAfterSay("I am ready for another question",
						1000);
			}

		} else {
			std::cout << testPrompt << "Failed to call service what do you see"
					<< std::endl;
			responseMsg.successful = 0;
			JustinaHRI::waitAfterSay("Repeate the question please", 1000);
		}
	} else {
		std::cout << testPrompt << "Needed services are not available :'("
				<< std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);
}

void callbackCmdDescribe(const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Describe ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseDescribe;
	responseDescribe.name = "cmd_world";
	responseDescribe.params = "what_see_yes";
	responseDescribe.id = msg->id;
	responseDescribe.successful = 1;

	std::vector<std::string> tokens;
	std::string str = msg->params;
	split(tokens, str, is_any_of(" "));

	if (tokens[3] == "nil" && tokens[1] == "nil") {
		JustinaHRI::waitAfterSay("There are no persons in the room", 1000);
		std::cout << "There are no persons in the room" << std::endl;
	} else {
		if (tokens[3] != "nil" && tokens[3] != "solo") {
			if (tokens[1] == "derecha") {
				JustinaHRI::waitAfterSay("peter is in the left of john", 1000);
				std::cout << "peter is in the left of john" << std::endl;
			}
			if (tokens[1] == "izquerda") {
				JustinaHRI::waitAfterSay("peter is in the right of john", 1000);
				std::cout << "peter is in the right of john" << std::endl;
			}

		}
		if (tokens[3] == "nil" && tokens[1] != "nil") {
			JustinaHRI::waitAfterSay("peter is the only person in the room",
					1000);
			std::cout << "peter is in the only person in the room" << std::endl;
		}

		if (tokens[1] != "nil" && tokens[1] != "solo") {
			if (tokens[3] == "derecha") {
				JustinaHRI::waitAfterSay("john is in the left of peter", 1000);
				std::cout << "john is in the left of peter" << std::endl;
			}
			if (tokens[3] == "izquerda") {
				JustinaHRI::waitAfterSay("john is in the right of peter", 1000);
				std::cout << "john is in the right of peter" << std::endl;
			}
		}
		if (tokens[1] == "nil" && tokens[3] != "nil") {
			JustinaHRI::waitAfterSay("john is the only person in the room",
					1000);
			std::cout << "john is the only person in the room" << std::endl;
		}
	}
	std::stringstream ss;

	for (int k = 0; k < objectsids.size(); k++) {
		ss.str("");
		ss << "There are a " << objectsids[k] << " in the table";
		std::cout << ss.str() << std::endl;
		JustinaHRI::waitAfterSay(ss.str(), 1000);

	}
	if (objectsids.size() == 0) {
		std::cout << "There are not objects in the room" << std::endl;
		JustinaHRI::waitAfterSay("there are not objects in the room", 1000);
	}

	command_response_pub.publish(responseDescribe);
}

void callbackCmdWhere(const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Where ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseDescribe;
	responseDescribe.name = "cmd_world";
	responseDescribe.params = "what_see_yes";
	responseDescribe.id = msg->id;
	responseDescribe.successful = 1;

	std::stringstream ss;

	std::vector<std::string> tokens;
	std::string str = msg->params;
	split(tokens, str, is_any_of(" "));

	if (tokens[1] == "nil") {
		ss.str("");
		ss << "The object " << tokens[0] << " is not on the table";
		std::cout << ss.str() << std::endl;
		JustinaHRI::waitAfterSay(ss.str(), 1000);
	} else if (tokens[1] == "table") {
		ss.str("");
		ss << "The " << tokens[0] << " is on the " << tokens[1];
		std::cout << ss.str() << std::endl;
		JustinaHRI::waitAfterSay(ss.str(), 1000);
	} else if (tokens[1] == "nil" && tokens[2] != "nobody") {
		ss.str("");
		ss << tokens[2] << " have the " << tokens[0];
		std::cout << ss.str() << std::endl;
		JustinaHRI::waitAfterSay(ss.str(), 1000);
	}
	else if(tokens[1] == "droped")
	{
		ss.str("");
		ss << tokens[2] << " have the " << tokens[0];
		std::cout << ss.str() << std::endl;
		JustinaHRI::waitAfterSay(ss.str(), 1000);
	}

	command_response_pub.publish(responseDescribe);
}

void callbackCmdTakeOrder(
		const planning_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Take order ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	bool success = true;
	startSignalSM = true;

	//if(!runSMCLIPS)
	//	success = false;
	success = success
			& ros::service::waitForService(
					"/planning_open_challenge/wait_command", 50000);
	if (success) {
		JustinaHRI::waitAfterSay("Yes what is your oreder", 1000);
		planning_msgs::planning_cmd srv;
		srv.request.name = "test_wait";
		srv.request.params = "Ready";
		std::cout << "hola" << std::endl;
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
			success = JustinaTasks::findPerson(tokens[1]);
			ss << responseMsg.params;
		} else {
			geometry_msgs::Pose pose;
			success = JustinaTasks::findObject(tokens[0], pose);
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

	bool success = true;
	success = JustinaTasks::alignWithTable(0.35);
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

	bool success = JustinaTasks::dropObject();

	if (success)
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
		std::cout << "person" << std::endl;
	} else {
		success = JustinaTasks::sayAndSyncNavigateToLoc(tokens[1], 120000);
		std::cout << "inspection" << std::endl;
	}
	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "open_challenge_test");
	ros::NodeHandle n;

	srvCltWhatSee = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_open_challenge/what_see");
	ros::Subscriber subCmdWorld = n.subscribe(
			"/planning_open_challenge/cmd_world", 1, callbackCmdWorld);
	ros::Subscriber subCmdDescribe = n.subscribe(
			"/planning_open_challenge/cmd_describe", 1, callbackCmdDescribe);
	ros::Subscriber subCmdTakeOrder = n.subscribe(
			"/planning_open_challenge/cmd_order", 1, callbackCmdTakeOrder);
	ros::Subscriber subCmdSpeech = n.subscribe(
			"/planning_open_challenge/cmd_speech", 1, callbackCmdSpeech);
	ros::Subscriber subCmdInterpret = n.subscribe(
			"/planning_open_challenge/cmd_int", 1, callbackCmdInterpret);
	ros::Subscriber subCmdConfirmation = n.subscribe(
			"/planning_open_challenge/cmd_conf", 1, callbackCmdConfirmation);
	ros::Subscriber subCmdGetTasks = n.subscribe(
			"/planning_open_challenge/cmd_task", 1, callbackCmdGetTasks);
	ros::Subscriber subCmdExplain = n.subscribe(
			"/planning_open_challenge/cmd_explain", 1,
			callbackCmdExplainThePlan);
	ros::Subscriber subCmdWhere = n.subscribe(
			"/planning_open_challenge/cmd_where", 1, callbackCmdWhere);
	ros::Subscriber subCmdDisponible = n.subscribe(
			"/planning_open_challenge/cmd_disp", 1, callbackCmdDisponible);
	ros::Subscriber subCmdHappen = n.subscribe(
			"/planning_open_challenge/cmd_happen", 1, callbackCmdHappen);

	ros::Subscriber subCmdNavigation = n.subscribe(
			"/planning_open_challenge/cmd_goto", 1, callbackCmdNavigation);
	ros::Subscriber subCmdAnswer = n.subscribe(
			"/planning_open_challenge/cmd_answer", 1, callbackCmdAnswer);
	ros::Subscriber subCmdFindObject = n.subscribe(
			"/planning_open_challenge/cmd_find_object", 1,
			callbackCmdFindObject);
	ros::Subscriber subCmdAskFor = n.subscribe(
			"/planning_open_challenge/cmd_ask_for", 1, callbackAskFor);
	ros::Subscriber subCmdStatusObject = n.subscribe(
			"/planning_open_challenge/cmd_status_object", 1,
			callbackStatusObject);
	ros::Subscriber subCmdMoveActuator = n.subscribe(
			"/planning_open_challenge/cmd_move_actuator", 1,
			callbackMoveActuator);
	ros::Subscriber subCmdDrop = n.subscribe(
			"/planning_open_challenge/cmd_drop", 1, callbackDrop);
	ros::Subscriber subCmdUnknown = n.subscribe(
			"/planning_open_challenge/cmd_unknown", 1, callbackUnknown);

	srvCltGetTasks = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_open_challenge/get_task");
	srvCltInterpreter = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_open_challenge/interpreter");
	srvCltWaitConfirmation = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_open_challenge/confirmation");
	srvCltWaitForCommand = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_open_challenge/wait_command");
	srvCltAnswer = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_open_challenge/answer");
	srvCltExplain = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_open_challenge/plan_explain");
	srvCltDisponible = n.serviceClient<planning_msgs::planning_cmd>(
			"/planning_open_challenge/disponible");

	command_response_pub = n.advertise<planning_msgs::PlanningCmdClips>(
			"/planning_open_challenge/command_response", 1);

	std::string locationsFilePath = "";
	for (int i = 0; i < argc; i++) {
		std::string strParam(argv[i]);
		if (strParam.compare("-f") == 0)
			locationsFilePath = argv[++i];
	}

	JustinaHRI::setNodeHandle(&n);
	JustinaHardware::setNodeHandle(&n);
	JustinaKnowledge::setNodeHandle(&n);
	JustinaManip::setNodeHandle(&n);
	JustinaNavigation::setNodeHandle(&n);
	JustinaTasks::setNodeHandle(&n);
	JustinaTools::setNodeHandle(&n);
	JustinaVision::setNodeHandle(&n);

	ros::Rate rate(10);
	state = SM_INIT;

	while (ros::ok()) {
		switch (state) {
		case SM_INIT:
			if (startSignalSM) {
				JustinaHRI::waitAfterSay(
						"Hellow my name is Justina, I'm ready for the open chanlenge",
						1000);
				state = SM_NAVIGATE_TO_THE_LOCATION;
			}
			std::cout << "state:" << state << std::endl;
			break;
		case SM_NAVIGATE_TO_THE_LOCATION:
			//tasks.syncSpeech("I'am going to the table.", 30000, 2000);
			//tasks.syncMove(0.5, 0.0, 3000);

			std::cout << "GPSRTest.->First try to move" << std::endl;
			if (!JustinaTasks::sayAndSyncNavigateToLoc("inspection", 120000)) {
				std::cout << "GPSRTest.->Second try to move" << std::endl;
				if (!JustinaTasks::sayAndSyncNavigateToLoc("inspection",
						120000)) {
					std::cout << "GPSRTest.->Third try to move" << std::endl;
					if (JustinaTasks::sayAndSyncNavigateToLoc("inspection",
							120000)) {
						JustinaTasks::alignWithTable(0.35);
						//tasks.syncSpeech("I'm ready for a spoken command", 30000, 2000);
						state = SM_SEND_INIT_CLIPS;
					}
				} else {
					JustinaTasks::alignWithTable(0.35);
					//tasks.syncSpeech("I'm ready for a spoken command", 30000, 2000);
					state = SM_SEND_INIT_CLIPS;
				}
			} else {
				JustinaTasks::alignWithTable(0.35);
				//tasks.syncSpeech("I'm ready for a spoken command", 30000, 2000);
				state = SM_SEND_INIT_CLIPS;
			}

			JustinaTasks::alignWithTable(0.35);
			JustinaHRI::waitAfterSay("I'am ready for user questions.", 1000);
			state = SM_SEND_INIT_CLIPS;
			break;
		case SM_SEND_INIT_CLIPS:
			JustinaVision::startQRReader();
			initMsg.successful = 0;
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
