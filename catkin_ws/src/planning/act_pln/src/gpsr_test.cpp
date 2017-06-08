#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "knowledge_msgs/PlanningCmdClips.h"
#include "knowledge_msgs/planning_cmd.h"

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
#include <map>

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
knowledge_msgs::PlanningCmdClips initMsg;

// This is for the attemps for a actions
std::string lastCmdName = "";
std::string currentName = "";
int numberAttemps = 0;
int cantidad = 0;

ros::ServiceClient srvCltGetTasks;
ros::ServiceClient srvCltInterpreter;
ros::ServiceClient srvCltWaitConfirmation;
ros::ServiceClient srvCltWaitForCommand;
ros::ServiceClient srvCltAnswer;
ros::ServiceClient srvCltAskName;

void validateAttempsResponse(knowledge_msgs::PlanningCmdClips msg) {
	lastCmdName = msg.name;
	if (msg.successful == 0
			&& (msg.name.compare("move_actuator") == 0
					|| msg.name.compare("find_object") == 0
					|| msg.name.compare("status_object") == 0
					|| msg.name.compare("many_obj") == 0)) {
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

void callbackCmdSpeech(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Speech ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
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
		knowledge_msgs::planning_cmd srv;
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
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command interpreter ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("/planning_clips/spr_interpreter",
			5000);
	if (success) {
		knowledge_msgs::planning_cmd srv;
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
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command confirmation ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("spg_say", 5000);
	success = success & ros::service::waitForService("/planning_clips/confirmation",5000);
	if (success) {
		std::string to_spech = responseMsg.params;
		boost::replace_all(to_spech, "_", " ");
		std::stringstream ss;
		ss << "Do you want me " << to_spech;
		std::cout << "------------- to_spech: ------------------ " << ss.str()
				<< std::endl;

		JustinaHRI::waitAfterSay(ss.str(), 2500);

		knowledge_msgs::planning_cmd srv;
		srv.request.name = "test_confirmation";
		srv.request.params = responseMsg.params;
		if (srvCltWaitConfirmation.call(srv)) {
			std::cout << "Response of confirmation:" << std::endl;
			std::cout << "Success:" << (long int) srv.response.success
					<< std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			if (srv.response.success)
				JustinaHRI::waitAfterSay("Ok i start to execute the command",
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

void callbackCmdGetTasks(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command get tasks ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("/planning_clips/get_task",
			5000);
	if (success) {
		knowledge_msgs::planning_cmd srv;
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
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Navigation ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
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

void callbackCmdAnswer(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {

	std::cout << testPrompt << "--------- Command answer a question ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::stringstream ss;
	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));

	std::map<int,std::string> weekdays;
	std::map<int,std::string> months;
	std::map<int,std::string> days;

  	weekdays[0]="sunday";
  	weekdays[1]="monday";
  	weekdays[2]="tuesday";
	weekdays[3]="wednesday";
	weekdays[4]="thursday";
	weekdays[5]="friday";
	weekdays[6]="saturday";

	months[0] = "january";
	months[1] = "february";
	months[2] = "march";
	months[3] = "april";
	months[4] = "may";
	months[5] = "june";
	months[6] = "july";
	months[7] = "august";
	months[8] = "september";
	months[9] = "october";
	months[10] = "november";
	months[11] = "december";

	days[1] = "first";
	days[2] = "second";
	days[3] = "third";
	days[4] = "fourth";
	days[5] = "fifth";
	days[6] = "sixth";
	days[7] = "seventh";
	days[8] = "eighth";
	days[9] = "ninth";
	days[10] = "tenth";
	days[11] = "eleventh";
	days[12] = "twelfth";
	days[13] = "thirteenth";
	days[14] = "fourteenth";
	days[15] = "fifteenth";
	days[16] = "sixteenth";
	days[17] = "seventeenth";
	days[18] = "eighteenth";
	days[19] = "nineteenth";
	days[20] = "twentieht";
	days[21] = "twenty first";
	days[22] = "twenty second";
	days[23] = "twenty third";
	days[24] = "twenty fourth";
	days[25] = "twenty fifth";
	days[26] = "twenty sixth";
	days[27] = "twenty seventh";
	days[28] = "twenty eighth";
	days[29] = "twenty ninth";
	days[30] = "thirtieth";
	days[31] = "thirty first";
	

	bool success = ros::service::waitForService("spg_say", 5000);
	if (success) {
		std::string param1 = tokens[1];
		if (param1.compare("a_question") == 0) {
			success = ros::service::waitForService("/planning_clips/answer",
					5000);
			if (success) {
				success = JustinaHRI::waitAfterSay(
						"I am waiting for the user question", 2000);
				knowledge_msgs::planning_cmd srv;
				srvCltAnswer.call(srv);
				if (srv.response.success)
					success = JustinaHRI::waitAfterSay(srv.response.args, 2000);
				else
					success = false;
			}
		} else if (param1.compare("your_name") == 0) {
			JustinaHRI::waitAfterSay("Hellow my name is justina", 2000);
		}else if (param1.compare("your_team_affiliation") == 0 || param1.compare("affiliation") == 0) {
			JustinaHRI::waitAfterSay("my team affiliation is the national autonomous university of mexico", 2000);
		}else if (param1.compare("your_team_country") == 0 || param1.compare("country") == 0) {
			JustinaHRI::waitAfterSay("My teams country is Mexico", 2000);
		} else if (param1.compare("your_team_name") == 0
				|| param1.compare("the_name_of_your_team") == 0 || param1.compare("name") == 0)  {
			JustinaHRI::waitAfterSay("Hello my team is pumas", 2000);
		} else if (param1.compare("introduce_yourself") == 0 || param1.compare("something_about_yourself") == 0) {
			JustinaHRI::waitAfterSay("I am going to introduce myself", 2000);
			JustinaHRI::waitAfterSay("My name is justina", 2000);
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
			ss << "The time is " << ltm->tm_hour << " hours" << ltm->tm_min << " minutes";
			JustinaHRI::waitAfterSay(ss.str(), 2000);
		} else if (param1.compare("what_day_is_tomorrow") == 0) {
			std::time_t now = time(0);
			std::tm *ltmnow = localtime(&now);
			std::cout << "Curr day :" << ltmnow->tm_mday << std::endl;
			ltmnow->tm_mday = ltmnow->tm_mday + 1;
			std::cout << "Curr month :" << ltmnow->tm_mon << std::endl;
			std::cout << "The day of month:" << ltmnow->tm_mday << std::endl;
			ss << "Tomorrow is " << months[ltmnow->tm_mon] << " " << days[ltmnow->tm_mday];
			JustinaHRI::waitAfterSay(ss.str(), 2000);
		}else if (param1.compare("the_day_of_the_week") == 0){
			ss.str("");
			std::time_t now = time(0);
			std::tm *ltmnow = localtime(&now);
			std::cout << "Curr day :" << ltmnow->tm_wday << std::endl;
			std::cout << "The day of week:" << ltmnow->tm_wday << std::endl;
			std::time_t day_week = std::mktime(ltmnow);
			std::cout << "Week day format :" << ltmnow->tm_wday << std::endl;
			ss << "The day of the week is " << weekdays[ltmnow->tm_wday];
			JustinaHRI::waitAfterSay(ss.str(), 2000);
		}else if (param1.compare("the_day_of_the_month") == 0 || param1.compare("what_day_is_today") == 0) {
			ss.str("");
			std::time_t now = time(0);
			std::tm *ltmnow = localtime(&now);
			std::cout << "Curr month :" << ltmnow->tm_mon << std::endl;
			std::cout << "The day of month:" << ltmnow->tm_mday << std::endl;
			ss << "Today is " << months[ltmnow->tm_mon] << " " << days[ltmnow->tm_mday];
			JustinaHRI::waitAfterSay(ss.str(), 2000);
		}else if (param1.compare("a_joke") == 0) {
			ss.str("");
			JustinaHRI::waitAfterSay("I am going to say a joke", 2000);
			JustinaHRI::waitAfterSay("What is the longest word in the English language", 2000);
			JustinaHRI::waitAfterSay("SMILES, there is a mile between the first and last letters", 2000);
			JustinaHRI::waitAfterSay("hee hee hee", 2000);
		}
		else if(param1.compare("tell_many_obj") == 0){
			ss.str("");
			ss << "I found " << cantidad << " "<< currentName;
			JustinaHRI::waitAfterSay(ss.str(), 2000);
		}
		else if(param1.compare("ask_name") == 0){
			ss.str("");
			JustinaHRI::waitAfterSay("Hello my name is Justina, what is your name", 2000);
			/// codigo para preguntar nombre Se usara un servicio
			bool success = ros::service::waitForService("spg_say", 5000);
			success = success & ros::service::waitForService("/planning_clips/ask_name",5000);
			if (success) {
				knowledge_msgs::planning_cmd srv;
				srv.request.name = "test_ask_name";
				srv.request.params = responseMsg.params;
				if (srvCltAskName.call(srv)) {
					std::cout << "Response of confirmation:" << std::endl;
					std::cout << "Success:" << (long int) srv.response.success << std::endl;
					std::cout << "Args:" << srv.response.args << std::endl;
					currentName = srv.response.args;
					if (srv.response.success){
						ss << "Hello " << srv.response.args;
						JustinaHRI::waitAfterSay(ss.str(), 2000);
					}
					else
						JustinaHRI::waitAfterSay("Could you repeat your name please", 2000);

					//responseMsg.params = srv.response.args;
					responseMsg.successful = srv.response.success;
				} else {
					std::cout << testPrompt << "Failed to call service of confirmation" << std::endl;
					responseMsg.successful = 0;
					JustinaHRI::waitAfterSay("Repeate the command please", 2000);
				}
			} else {
				std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
				responseMsg.successful = 0;
			}
		}
		else if(param1.compare("tell_name") == 0){
			ss.str("");
			ss << "Hello, the name of the person I found is " << currentName;
			JustinaHRI::waitAfterSay(ss.str(), 2000);
		}
	} else
		success = false;

	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;

	weekdays.clear();
	months.clear();
	days.clear();
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackCmdFindObject(
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command find a object ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
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
			if(tokens[1] == "no_location")
				success = JustinaTasks::followAPersonAndSayStop("stop follow me");
			else
				success = JustinaTasks::findAndFollowPersonToLoc(tokens[1]);
			ss << responseMsg.params;
		} else if (tokens[0] == "man_guide") {
			success = JustinaTasks::guideAPerson(tokens[1], 0);
			ss << responseMsg.params;
		} else if (tokens[0] == "specific") {
			success = JustinaTasks::findPerson();//success = JustinaTasks::findPerson(tokens[1])
			ss << "find_spc_person " << tokens[0] << " " << tokens[1];//ss << responseMsg.params;
		} else if (tokens[0] == "only_find"){
			bool withLeftOrRightArm;
			geometry_msgs::Pose pose;
			JustinaTasks::alignWithTable(0.35);
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			success = JustinaTasks::findObject(tokens[1], pose, withLeftOrRightArm);
			ss << tokens[1] << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z ;
		} else {
			geometry_msgs::Pose pose;
			bool withLeftOrRightArm;
			success = JustinaTasks::findObject(tokens[0], pose, withLeftOrRightArm);
			if(withLeftOrRightArm)
				ss << responseMsg.params << " " << pose.position.x << " " 
					<< pose.position.y << " " << pose.position.z << " left";
			else
				ss << responseMsg.params << " " << pose.position.x << " " 
					<< pose.position.y << " " << pose.position.z << " right";
		}
		responseMsg.params = ss.str();
	}
	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	if(tokens[0] == "only_find")
		command_response_pub.publish(responseMsg);
	else
		validateAttempsResponse(responseMsg);
	
}

void callbackFindCategory(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << testPrompt << "-------- Command Find Category--------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;

	std::map<std::string, std::string > catList;
	
	catList["pringles"] = "snacks";
	catList["senbei"] = "snacks";
	catList["peanuts"] = "snacks";
	catList["chips"] = "snacks";

	catList["chocolate_bar"] = "candies";
	catList["manju"] = "candies";
	catList["mints"] = "candies";
	catList["chocolate_egg"] = "candies";

	catList["noodles"] = "food";
	catList["apple"] = "food";
	catList["paprika"] = "food";
	catList["watermelon"] = "food";
	catList["sushi"] = "food";

	catList["tea"] = "drinks";
	catList["beer"] = "drinks";
	catList["coke"] = "drinks";
	catList["sake"] = "drinks";

	catList["shampoo"] = "toiletries";
	catList["soap"] = "toiletries";
	catList["cloth"] = "toiletries";
	catList["sponge"] = "toiletries";

	catList["bowl"] = "containers";
	catList["tray"] = "containers";
	catList["plate"] = "containers";


	JustinaHRI::waitAfterSay("I am looking for objects on the table", 1500);
	JustinaManip::hdGoTo(0, -0.9, 5000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	JustinaTasks::alignWithTable(0.35);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));


	std::map<std::string, int> countCat;
	float pos = 0.0, advance = 0.3, maxAdvance = 0.3;
	countCat["snacks"] = 0;
	countCat["candies"] = 0;
	countCat["food"] = 0;
	countCat["drinks"] = 0;
	countCat["toiletries"] = 0;
	countCat["containers"] = 0;

		//boost::this_thread::sleep(boost::posix_time::milliseconds(500));
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
					vision_msgs::VisionObject vObject = recognizedObjects[i];
					std::cout << "object:  " << vObject.id << std::endl;
					std::map<std::string, std::string>::iterator it = catList.find(vObject.id);
					if (it != catList.end()){
						std::map<std::string, int>::iterator ap = countCat.find(it->second);
						ap->second = ap->second + 1;
					}
				}
			}
		}

	std::map<std::string, int>::iterator catRes = countCat.find(tokens[0]);
	if(catRes->second > 0){
		ss << "I found the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(), 1000);
		ss.str("");
		ss << responseMsg.params << " " << catRes->second;
		cantidad = catRes->second;
		currentName = tokens[0];
		responseMsg.params = ss.str();
		responseMsg.successful = 1;
	}
	else {
		ss << "I can not find the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(),1000);
		ss.str("");
		ss << responseMsg.params << " " << 0;
		responseMsg.params = ss.str();
		responseMsg.successful = 0;
	}

	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackManyObjects(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << testPrompt << "-------- Command How Many Objects--------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;

	std::map<std::string, int > countObj;
	
	countObj["pringles"] = 0;
	countObj["senbei"] = 0;
	countObj["peanuts"] = 0;
	countObj["chips"] = 0;

	countObj["chocolate_bar"] = 0;
	countObj["manju"] = 0;
	countObj["mints"] = 0;
	countObj["chocolate_egg"] = 0;

	countObj["noodles"] = 0;
	countObj["apple"] = 0;
	countObj["paprika"] = 0;
	countObj["watermelon"] = 0;
	countObj["sushi"] = 0;

	countObj["tea"] = 0;
	countObj["beer"] = 0;
	countObj["coke"] = 0;
	countObj["sake"] = 0;

	countObj["shampoo"] = 0;
	countObj["soap"] = 0;
	countObj["cloth"] = 0;
	countObj["sponge"] = 0;

	countObj["bowl"] = 0;
	countObj["tray"] = 0;
	countObj["plate"] = 0;

	countObj["juice"] = 0;
	countObj["milk"] = 0;

	int arraySize = 0;
	int numObj = 0;

	JustinaHRI::waitAfterSay("I am looking a category on the table", 1500);
	JustinaManip::hdGoTo(0, -0.9, 5000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	JustinaTasks::alignWithTable(0.35);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	
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
					vision_msgs::VisionObject vObject = recognizedObjects[i];
					std::cout << "object:  " << vObject.id << std::endl;
					std::map<std::string, int>::iterator it = countObj.find(vObject.id);
					if (it != countObj.end()){
						it->second = it->second + 1;
						arraySize++;
					}
					std::cout << "ITERADOR: " << it->second << std::endl;
				}
				if(arraySize > numObj)
					numObj = arraySize;
				arraySize = 0;
			}
		}
	ss.str("");
	std::map<std::string, int>::iterator objRes = countObj.find(tokens[0]);
	currentName = tokens[0];
	if(numObj > 0){
		ss << "I found the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(), 1500);
		ss.str("");
		cantidad = numObj;
		ss << responseMsg.params << " " << cantidad;
		responseMsg.params = ss.str();
		responseMsg.successful = 1;
	}
	else {
		ss << "I can not find the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(),1000);
		ss.str("");
		ss << responseMsg.params << " " << 0;
		responseMsg.params = ss.str();
		responseMsg.successful = 0;
	}

	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
	
}

void callbackAskFor(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Ask for ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
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
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Status object ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
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
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Move actuator ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	bool armFlag = true;

	bool success = ros::service::waitForService("spg_say", 5000);
	//success = success & tasks.moveActuator(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()), tokens[0]);
	if(tokens[4] == "false")
			armFlag = false;
	success = success
			& JustinaTasks::moveActuatorToGrasp(atof(tokens[1].c_str()),
					atof(tokens[2].c_str()), atof(tokens[3].c_str()), armFlag,
					tokens[0]);
	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;

	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackDrop(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Drop ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;
	bool armFlag = true;
	bool succes;

	if(tokens[2] == "false")
			armFlag = false;

	if(tokens[0] == "person")
		succes = JustinaTasks::dropObject(tokens[1], armFlag, 30000);
	else if(tokens[0] == "object"){
		ss.str("");
		ss << "I am going to deliver the " << tokens[1];
		JustinaHRI::waitAfterSay(ss.str(), 2000);
		succes = JustinaTasks::placeObject(armFlag);
	}
	
	if (succes)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;

	validateAttempsResponse(responseMsg);
}

void callbackUnknown(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command unknown ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	responseMsg.successful = 1;
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}


void callbackAskPerson(
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command ask for person ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
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
		ss << "Hello, " << to_spech << " is your name";
		std::cout << "------------- to_spech: ------------------ " << ss.str()
				<< std::endl;

		JustinaHRI::waitAfterSay(ss.str(), 1500);

		knowledge_msgs::planning_cmd srv;
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
				boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
				JustinaNavigation::moveDistAngle(0, 1.57, 10000);
				boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
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

	srvCltGetTasks = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/get_task");
	srvCltInterpreter = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/spr_interpreter");
	srvCltWaitConfirmation = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/confirmation");
	srvCltWaitForCommand = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/wait_command");
	srvCltAnswer = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/answer");
	srvCltAskName = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/ask_name");

	ros::Subscriber subCmdSpeech = n.subscribe("/planning_clips/cmd_speech", 1, callbackCmdSpeech);
	ros::Subscriber subCmdInterpret = n.subscribe("/planning_clips/cmd_int", 1, callbackCmdInterpret);
	ros::Subscriber subCmdConfirmation = n.subscribe("/planning_clips/cmd_conf", 1, callbackCmdConfirmation);
	ros::Subscriber subCmdGetTasks = n.subscribe("/planning_clips/cmd_task", 1, callbackCmdGetTasks);

	ros::Subscriber subCmdNavigation = n.subscribe("/planning_clips/cmd_goto", 1, callbackCmdNavigation);
	ros::Subscriber subCmdAnswer = n.subscribe("/planning_clips/cmd_answer", 1, callbackCmdAnswer);
	ros::Subscriber subCmdFindObject = n.subscribe("/planning_clips/cmd_find_object", 1, callbackCmdFindObject);
	ros::Subscriber subCmdAskFor = n.subscribe("/planning_clips/cmd_ask_for", 1, callbackAskFor);
	ros::Subscriber subCmdStatusObject = n.subscribe("/planning_clips/cmd_status_object", 1, callbackStatusObject);
	ros::Subscriber subCmdMoveActuator = n.subscribe("/planning_clips/cmd_move_actuator", 1, callbackMoveActuator);
	ros::Subscriber subCmdDrop = n.subscribe("/planning_clips/cmd_drop", 1, callbackDrop);
	ros::Subscriber subCmdUnknown = n.subscribe("/planning_clips/cmd_unknown", 1, callbackUnknown);
	ros::Subscriber subAskPerson = n.subscribe("/planning_clips/cmd_ask_person", 1, callbackAskPerson);
	ros::Subscriber subFindCategory = n.subscribe("/planning_clips/cmd_find_category", 1, callbackFindCategory);
	ros::Subscriber subManyObjects = n.subscribe("/planning_clips/cmd_many_obj", 1, callbackManyObjects);

	command_response_pub = n.advertise<knowledge_msgs::PlanningCmdClips>("/planning_clips/command_response", 1);

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
			JustinaHRI::waitAfterSay("Now I can see that the door is open",
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
			//command_response_pub.publish(initMsg);
			// test for send mesage of type get task
			/*initMsg.name = "cmd_task";
			initMsg.id = 10;
			initMsg.params = "robot update_object_location location kitchen" ;
			initMsg.successful = true;
			boost::this_thread::sleep(boost::posix_time::milliseconds(400));
			ros::spinOnce();
			command_response_pub.publish(initMsg);
			boost::this_thread::sleep(boost::posix_time::milliseconds(400));
			ros::spinOnce();
			initMsg.name = "cmd_task";
			initMsg.id = 10;
			initMsg.params = "robot update_object_location location living_room 1" ;
			initMsg.successful = true;
			boost::this_thread::sleep(boost::posix_time::milliseconds(400));
			ros::spinOnce();
			command_response_pub.publish(initMsg);
			boost::this_thread::sleep(boost::posix_time::milliseconds(400));
			ros::spinOnce();*/
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
