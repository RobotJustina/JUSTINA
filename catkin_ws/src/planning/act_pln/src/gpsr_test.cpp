#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "knowledge_msgs/PlanningCmdClips.h"
#include "knowledge_msgs/planning_cmd.h"
#include "knowledge_msgs/StrQueryKDB.h"

#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaRepresentation.h"

#include <vector>
#include <ctime>
#include <map>

#define POS 0.0
#define ADVANCE 0.3 
#define MAXA 3.0
#define TIMEOUT_MEMORIZING 3000

using namespace boost::algorithm;

enum SMState {
	SM_INIT,
	SM_SAY_WAIT_FOR_DOOR,
	SM_WAIT_FOR_DOOR,
	SM_NAVIGATE_TO_THE_LOCATION,
	SM_INIT_SPEECH,
	SM_SEND_INIT_CLIPS,
	SM_RUN_SM_CLIPS,
    SM_RESET_CLIPS 
};

ros::Publisher command_response_pub;
ros::Publisher sendAndRunClips_pub;
ros::Publisher train_face_pub;
ros::Publisher pubStartTime; 
ros::Publisher pubResetTime;
std::string testPrompt;
SMState state = SM_INIT;
bool runSMCLIPS = false;
bool startSignalSM = false;
knowledge_msgs::PlanningCmdClips initMsg;

// This is for the attemps for a actions
std::string lastCmdName = "";
std::string currentName = "";
std::string objectName = "";
std::string categoryName = "";
int numberAttemps = 0;
int cantidad = 0;
int women;
int men;
int sitting;
int standing;
int lying;
ros::Time beginPlan;
bool fplan = false;
double maxTime = 180;
std::string cat_grammar= "gpsr_guadalajara.xml";

std::string microsoft_grammars[13];
std::string sphinx_grammars[13];
bool alternative_drink = true;
bool poket_reco = false;
std::string no_drink;
std::string prev_drink = "no_prev";
JustinaTasks::POSE poseRecog;

int num_speech_intents = 0;
std::vector<std::string> idsPerson;

ros::ServiceClient srvCltGetTasks;
ros::ServiceClient srvCltInterpreter;
ros::ServiceClient srvCltWaitConfirmation;
ros::ServiceClient srvCltWaitForCommand;
ros::ServiceClient srvCltAnswer;
ros::ServiceClient srvCltAskName;
ros::ServiceClient srvCltAskIncomplete;
ros::ServiceClient srvCltQueryKDB;
ros::ServiceClient srvEnableSphinx;

template <typename T>
std::pair<bool, int> findInVector( std::vector<T> & vecOfElements, const T & element){
    std::pair<bool, int> result;
    typename std::vector<T>::iterator it = std::find(vecOfElements.begin(), vecOfElements.end(), element);

    if(it != vecOfElements.end()){
        result.second = std::distance(vecOfElements.begin(), it);
        result.first = true;
    }
    else{
        result.second = -1;
        result.first = false;
    }

    return result;
}

void switchSpeechReco(int grammar_id, std::string speech){
    if (poket_reco){
        //use pocket sphinx
        //JustinaHRI::usePocketSphinx = true;
        JustinaHRI::enableGrammarSpeechRecognized(sphinx_grammars[grammar_id], 5.0);
	    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
        JustinaHRI::enableSpeechRecognized(false);
	    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
        if(speech != "")
            JustinaHRI::waitAfterSay(speech,5000);
        JustinaHRI::enableSpeechRecognized(true);
    }

    else{
        //use speech recognition of microsoft
        //JustinaHRI::usePocketSphinx = false;
        JustinaHRI::loadGrammarSpeechRecognized(microsoft_grammars[grammar_id]);
        if(speech != "")
            JustinaHRI::waitAfterSay(speech,5000);
    }
}

struct propObj{
    std::string obj;
    int value;
};

struct poseObj{
    float x;
    float y;
    float z;
};

bool compareUpward(propObj obj1, propObj obj2){
    return (obj1.value < obj2.value);
}

bool compareDownward(propObj obj1, propObj obj2){
    return (obj1.value > obj2.value);
}

bool lateralMov(float &pos, float &advance, float &maxAdvance){
    bool finishMotion = false;
		pos += advance;
		if ( pos == maxAdvance){
			JustinaNavigation::moveLateral(advance, 2000);
			boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
			advance = -2 * advance;
		}
		if (pos == -1 * maxAdvance){
			JustinaNavigation::moveLateral(advance, 2000);}
			boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
		if (pos == -3 *maxAdvance){
			JustinaNavigation::moveLateral(0.3, 2000);
			boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
			finishMotion = true;}
        return finishMotion;
}

void validateAttempsResponse(knowledge_msgs::PlanningCmdClips msg) {
	//lastCmdName = msg.name;
	if (msg.successful == 0
			&& (msg.name.compare("move_actuator") == 0
					//|| msg.name.compare("find_object") == 0
					|| msg.name.compare("status_object") == 0
					|| msg.name.compare("many_obj") == 0
					|| msg.name.compare("answer") == 0
					|| msg.name.compare("drop") == 0)) {
		if (msg.name.compare(lastCmdName) != 0)
			numberAttemps = 0;
		else if (numberAttemps == 0) {
			msg.successful = 1;
			numberAttemps = 0;
		} else
			numberAttemps++;
	}
	else if (msg.successful == 1){
		numberAttemps = 0;
	}
	
	lastCmdName = msg.name;
	command_response_pub.publish(msg);
}

bool validateContinuePlan(double currentTime, bool fplan)
{
	bool result = true;

	if (currentTime >= maxTime && fplan){
		std::stringstream ss;
		knowledge_msgs::StrQueryKDB srv;
		ss.str("");
		ss << "(assert (cmd_finish_plan 1))";
		srv.request.query = ss.str();
		if (srvCltQueryKDB.call(srv)) {
			std::cout << "Response of KBD Query:" << std::endl;
			std::cout << "TEST QUERY Args:" << srv.response.result << std::endl;
			result = false;
			beginPlan = ros::Time::now();
		} else {
			std::cout << testPrompt << "Failed to call service of KBD query"<< std::endl;
			result =  true;
		}
	}
	
	return result;

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
    JustinaManip::hdGoTo(0, 0.0, 5000);

	success = success
			& ros::service::waitForService("/planning_clips/wait_command",
					50000);
	if (success) {
		knowledge_msgs::planning_cmd srv;
		srv.request.name = "test_wait";
		srv.request.params = "Ready";
            
        switchSpeechReco(3, "");

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
    JustinaHRI::enableSpeechRecognized(false);
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
		ss << "Do you want me " << to_spech << ", say justina yes or justina no";
		std::cout << "------------- to_spech: ------------------ " << ss.str()
				<< std::endl;

		//JustinaHRI::waitAfterSay(ss.str(), 2500);

        switchSpeechReco(0, ss.str());

		knowledge_msgs::planning_cmd srv;
		srv.request.name = "test_confirmation";
		srv.request.params = responseMsg.params;
		if (srvCltWaitConfirmation.call(srv)) {
			std::cout << "Response of confirmation:" << std::endl;
			std::cout << "Success:" << (long int) srv.response.success
					<< std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			if (srv.response.success){
				JustinaHRI::waitAfterSay("Ok i start to execute the command", 2000);
                /*float currx, curry, currtheta;
                JustinaKnowledge::addUpdateKnownLoc("current_loc", currx, curry);*/
				beginPlan = ros::Time::now();
                std_msgs::Int32 timeout;
                timeout.data = 240000; //This is the time for restart clips
                pubStartTime.publish(timeout);
		num_speech_intents = 0;
			}
			else{
				num_speech_intents++;
				if (num_speech_intents > 3){
					JustinaHRI::waitAfterSay("please use a Q R code and I try to understand the command ", 5000);
				}
				else
					JustinaHRI::waitAfterSay("Repeate the command please", 2000);
			}

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
        JustinaHRI::enableSpeechRecognized(false);
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackCmdSpeechGenerator(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Speech Generator-----" << std::endl;
	std::cout << "name: " << msg->name << std::endl;
	std::cout << "params: " << msg->params << std::endl;
	
	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::stringstream ss;
        std::vector<std::string> tokens;
        std::string str = responseMsg.params;
        split(tokens, str, is_any_of("_"));
	
	
	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;

	ss << tokens[0];
	for(int i=1 ; i<tokens.size(); i++)
		ss << " "<< tokens[i];
	
	JustinaHRI::waitAfterSay(ss.str(), 10000);
	
	responseMsg.successful = 1;

	command_response_pub.publish(responseMsg);
	

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

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;

	bool success = true;
	bool nfp = true;	
	if (tokens[1] != "arena" && tokens[1] != "exitdoor")
		nfp = validateContinuePlan(d.toSec(), fplan);

	if (tokens[1] == "person") {
		success = true;
	} else {
        JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
		if (nfp)
			success = JustinaTasks::sayAndSyncNavigateToLoc(tokens[1], 120000);
	}
	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	if (nfp)
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
	
	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;

	int count_attemps = 0;
	std::string lastReco;
	std::string answer;
	bool response_question = false;

	bool success = ros::service::waitForService("spg_say", 5000);
	if (success) {
		std::string param1 = tokens[1];
		if (param1.compare("a_question") == 0) {
			/*success = ros::service::waitForService("/planning_clips/answer",
					5000);
			JustinaHRI::loadGrammarSpeechRecognized("questions.xml");
			if (success) {
				success = JustinaHRI::waitAfterSay(
						"I am waiting for the user question", 2000);
				knowledge_msgs::planning_cmd srv;
				srvCltAnswer.call(srv);
				if (srv.response.success){
					success = JustinaHRI::waitAfterSay(srv.response.args, 2000);
					responseMsg.successful = 1;
				}
				else
					responseMsg.successful = 0;*/
			//JustinaHRI::loadGrammarSpeechRecognized("questions.xml");
			while(!response_question && count_attemps < 3){
				switchSpeechReco(4, "Tell me your question please");
				//JustinaHRI::waitAfterSay("Tell me your question please", 2000);
				JustinaHRI::waitForSpeechRecognized(lastReco,400);
				JustinaHRI::waitForSpeechRecognized(lastReco,10000);
				if(!JustinaKnowledge::comparePredQuestion(lastReco,answer))
				{
					std::cout << "no match with any question" << std::endl;
					responseMsg.successful = 0;
					JustinaHRI::waitAfterSay("I am sorry, I did not understand you", 2000);
					count_attemps++;
				}
				else{
					response_question = true;
					success = JustinaHRI::waitAfterSay(answer, 5000);
				}
	
			}
			responseMsg.successful = 1;

		} else if (param1.compare("your_name") == 0) {
			JustinaHRI::waitAfterSay("Hellow my name is justina", 2000);
			responseMsg.successful = 1;
		}else if (param1.compare("your_team_affiliation") == 0 || param1.compare("affiliation") == 0) {
			JustinaHRI::waitAfterSay("my team affiliation is the national autonomous university of mexico", 2000);
			responseMsg.successful = 1;
		}else if (param1.compare("your_team_country") == 0 || param1.compare("country") == 0) {
			JustinaHRI::waitAfterSay("My teams country is Mexico", 2000);
			responseMsg.successful = 1;
		} else if (param1.compare("your_team_name") == 0
				|| param1.compare("the_name_of_your_team") == 0 || param1.compare("name") == 0)  {
			JustinaHRI::waitAfterSay("Hello my team is pumas", 2000);
			responseMsg.successful = 1;
		} else if (param1.compare("introduce_yourself") == 0 || param1.compare("something_about_yourself") == 0) {
			JustinaHRI::waitAfterSay("I am going to introduce myself", 2000);
			JustinaHRI::waitAfterSay("My name is justina", 2000);
			JustinaHRI::waitAfterSay("i am from Mexico city", 2000);
			JustinaHRI::waitAfterSay("my team is pumas", 2000);
			JustinaHRI::waitAfterSay(
					"of the national autonomous university of mexico", 2000);
			responseMsg.successful = 1;
		} 
		/*else if (param1.compare("the_day") == 0
				|| param1.compare("the_time") == 0) {
			ss.str("");
			//std::locale::global(std::locale("de_DE.utf8"));
			//std::locale::global(std::locale("en_us.utf8"));
			time_t now = time(0);
			char* dt = ctime(&now);
			std::cout << "Day:" << dt << std::endl;
			JustinaHRI::waitAfterSay(dt, 2000);
			responseMsg.successful = 1;
		}*/ else if (param1.compare("what_time_is_it") == 0 || param1.compare("the_time") == 0) {
			ss.str("");
			std::time_t now = time(0);
			std::tm *ltm = localtime(&now);
			ss << "The time is " << ltm->tm_hour << " hours," << ltm->tm_min << " minutes";
			JustinaHRI::waitAfterSay(ss.str(), 2000);
			responseMsg.successful = 1;
		} else if (param1.compare("what_day_is_tomorrow") == 0) {
			std::time_t now = time(0);
			std::tm *ltmnow = localtime(&now);
			std::cout << "Curr day :" << ltmnow->tm_mday << std::endl;
			ltmnow->tm_mday = ltmnow->tm_mday + 1;
			std::cout << "Curr month :" << ltmnow->tm_mon << std::endl;
			std::cout << "The day of month:" << ltmnow->tm_mday << std::endl;
			ss << "Tomorrow is " << months[ltmnow->tm_mon] << " " << days[ltmnow->tm_mday];
			JustinaHRI::waitAfterSay(ss.str(), 2000);
			responseMsg.successful = 1;
		}else if (param1.compare("the_day_of_the_week") == 0 || param1.compare("the_day") == 0){
			ss.str("");
			std::time_t now = time(0);
			std::tm *ltmnow = localtime(&now);
			std::cout << "Curr day :" << ltmnow->tm_wday << std::endl;
			std::cout << "The day of week:" << ltmnow->tm_wday << std::endl;
			std::time_t day_week = std::mktime(ltmnow);
			std::cout << "Week day format :" << ltmnow->tm_wday << std::endl;
			ss << "Today is " << weekdays[ltmnow->tm_wday];
			JustinaHRI::waitAfterSay(ss.str(), 2000);
			responseMsg.successful = 1;
		}else if (param1.compare("the_day_of_the_month") == 0 || param1.compare("what_day_is_today") == 0) {
			ss.str("");
			std::time_t now = time(0);
			std::tm *ltmnow = localtime(&now);
			std::cout << "Curr month :" << ltmnow->tm_mon << std::endl;
			std::cout << "The day of month:" << ltmnow->tm_mday << std::endl;
			ss << "Today is " << weekdays[ltmnow->tm_wday] << " ,"<< months[ltmnow->tm_mon] << " " << days[ltmnow->tm_mday];
			JustinaHRI::waitAfterSay(ss.str(), 2000);
			responseMsg.successful = 1;
		}else if (param1.compare("a_joke") == 0) {
			ss.str("");
			JustinaHRI::waitAfterSay("I am going to say a joke", 2000);
			JustinaHRI::waitAfterSay("What is the longest word in the English language", 2000);
			JustinaHRI::waitAfterSay("SMILES, there is a mile between the first and last letters", 2000);
			JustinaHRI::waitAfterSay("hee hee hee", 2000);
			responseMsg.successful = 1;
		}
		else if(param1.compare("tell_many_obj") == 0){
			ss.str("");
			ss << "I found " << cantidad << " "<< currentName;
			std::cout << ss.str() << std::endl;
			JustinaHRI::waitAfterSay(ss.str(), 2000);
			responseMsg.successful = 1;
		}
		else if(param1.compare("tell_what") == 0){
			ss.str("");
			if(objectName == "none"){
				ss << "I did not find objects";
				JustinaHRI::waitAfterSay(ss.str(), 2000);
				responseMsg.successful = 1;
			}
			else{
				ss << "The " << objectName << " is the " << currentName << " object I found"; 
				std::cout << ss.str() << std::endl;
				JustinaHRI::waitAfterSay(ss.str(), 2000);
				responseMsg.successful = 1;
			}
		}
		else if(param1.compare("tell_what_cat") == 0){
			ss.str("");
			if(objectName == "none"){
				ss << "I did not find the " << categoryName;
				JustinaHRI::waitAfterSay(ss.str(), 2000);
				responseMsg.successful = 1;
			}
			else{
				ss << "The " << objectName << " is the " << currentName << " " << categoryName <<" I found"; 
				std::cout << ss.str() << std::endl;
				JustinaHRI::waitAfterSay(ss.str(), 2000);
				responseMsg.successful = 1;
			}
		}
        else if(param1.compare("tell_what_three_cat") == 0){
            ss.str("");
            ss << "(assert (cmd_get_speech 1))";
            std::string query;
            JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);
		    boost::replace_all(query, "_", " ");
            JustinaHRI::waitAfterSay(query, 2000);
			responseMsg.successful = 1;
        }
		else if(param1.compare("tell_gender_pose") == 0){
			ss.str("");
			if (currentName == "no_gender_pose")
				ss << "I did not found any person";
			else
				ss << "I found a "<< currentName << " person";
			JustinaHRI::waitAfterSay(ss.str(), 2000);
			responseMsg.successful = 1;
		}
		else if(param1.compare("tell_how_many_people") == 0){
			ss.str("");
			if (currentName == "men" || currentName == "boys" || currentName == "male")
				ss << "I found " << men << " " << currentName;
			else if(currentName == "women" || currentName == "girls" || currentName == "female")
				ss << "I found " << women << " " << currentName;
			else if (currentName == "sitting")
				ss << "I found " << sitting << " " << currentName << " people";
			else if (currentName == "standing")
				ss << "I found " << standing << " " << currentName << " people";
			else if (currentName == "lying")
				ss << "I found " << lying << " " << currentName << " people";
			
			JustinaHRI::waitAfterSay(ss.str(), 2000);
			responseMsg.successful = 1;
		}
		else if(param1.compare("ask_name") == 0){
			ss.str("");
			if(numberAttemps == 0){
    			std::string lastReco;
                std::string name;
			
		    	int timeoutspeech = 10000;
    			bool conf = false;
		    	int count = 0;
    			//std::vector<std::string> tokens1;
			
			JustinaHRI::waitAfterSay("Hello my name is Justina", 10000);
            while(!conf && count < 5){
                switchSpeechReco(2, "tell me what is your name please");
                JustinaHRI::waitForSpeechRecognized(lastReco,400);
                if(JustinaHRI::waitForSpeechRecognized(lastReco,10000)){
                    if(JustinaRepresentation::stringInterpretation(lastReco, name))
                        std::cout << "last int: " << name << std::endl;
                        ss.str("");
                        ss << "is " << name << " your name";
                        switchSpeechReco(0, ss.str());
                        JustinaHRI::waitForSpeechRecognized(lastReco,400);
                        
                        JustinaHRI::waitForSpeechRecognized(lastReco,10000);
                        if(lastReco == "robot yes" || lastReco == "justina yes")
                            conf = true;
                        count++;
                }
            }
            if(conf){
                ss.str("");
                ss << "Hello " << name << " thank you";
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                currentName = name;
                responseMsg.successful = 1;
            }
            else{
                currentName = "ask_name_no";
                JustinaHRI::waitAfterSay("Sorry I did not understand you", 5000);
                responseMsg.successful = 1;
            }
		    }
		}
		else if(param1.compare("tell_name") == 0){
			ss.str("");
			if (currentName == "ask_name_no"){
				ss << "I find the person, but i did not understand the persons name";
				JustinaHRI::waitAfterSay(ss.str(), 10000);
				responseMsg.successful = 1;
				
			}
			else {
				ss << "Hello, the name of the person I found is " << currentName;
				JustinaHRI::waitAfterSay(ss.str(), 10000);
				responseMsg.successful = 1;
			}
		}
	} else
		success = false;

	/*if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;*/

	std::cout << "TEST FOR SUCCES VALUE: " << success << std::endl;
	//JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);

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

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	bool nfp = validateContinuePlan(d.toSec(), fplan);
	int timeout = (fplan == true) ? (maxTime - (int)d.toSec() )*1000 : maxTime * 1000;
	std::cout << "TIMEOUT: " << timeout <<std::endl;
	
	bool success = ros::service::waitForService("spg_say", 5000);
	if (success && nfp) {
		std::cout << testPrompt << "find: " << tokens[0] << std::endl;

		ss.str("");
		if (tokens[0] == "person") {
			success = JustinaTasks::findPerson("", -1, JustinaTasks::NONE, false, tokens[1]);
			ss << responseMsg.params << " " << 1 << " " << 1 << " " << 1;
		} else if (tokens[0] == "man") {
			//JustinaHRI::loadGrammarSpeechRecognized("follow_confirmation.xml");
            switchSpeechReco(5, "");
			if(tokens[1] == "no_location")
				success = JustinaTasks::followAPersonAndRecogStop("stop follow me");
			else
				success = JustinaTasks::findAndFollowPersonToLoc(tokens[1]);
			ss << responseMsg.params;
		} else if (tokens[0] == "man_guide") {
			JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
			boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
			success = JustinaTasks::guideAPerson(tokens[1], timeout);
			ss << responseMsg.params;
		} else if (tokens[0] == "specific") {
			success = JustinaTasks::findPerson(tokens[1], -1, JustinaTasks::NONE, false, tokens[2]);//success = JustinaTasks::findPerson(tokens[1])
			ss << "find_spc_person " << tokens[0] << " " << tokens[1] << " " << tokens[2];//ss << responseMsg.params;
        }else if (tokens[0] == "specific_eegpsr"){
            success = JustinaTasks::findPerson(tokens[1], -1, JustinaTasks::NONE, true, tokens[2]);//success = JustinaTasks::findPerson(tokens[1])
			ss << "find_spc_person " << tokens[0] << " " << tokens[1] << " " << tokens[2];//ss << responseMsg.params;
		} else if (tokens[0] == "only_find"){
			bool withLeftOrRightArm;
			ss.str("");
			ss << "I am looking for " << tokens[1] << " on the " << tokens[2];
			geometry_msgs::Pose pose;
			JustinaTasks::alignWithTable(0.42);
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			JustinaHRI::waitAfterSay(ss.str(), 2500);
            JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 8000);
			success = JustinaTasks::findObject(tokens[1], pose, withLeftOrRightArm);
            if(!success)
                JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 8000);
			ss.str("");
			if(withLeftOrRightArm)
				ss << tokens[1] << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " left only_find";
			else
				ss << tokens[1] << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " right only_find";
		} else if (tokens[0] == "abspose"){
            ss.str("");
            ss << "I am looking for objects on the " << tokens[1];
            JustinaHRI::waitAfterSay(ss.str(), 2500);
            JustinaManip::hdGoTo(0, -0.9, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaTasks::alignWithTable(0.42);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            int numObj = 0;
            int contador = 0;
			geometry_msgs::Pose pose;

            ss.str("");
			ss <<"object 2 2 2 left";
        
            JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 4000);
            do{
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                std::vector<vision_msgs::VisionObject> recognizedObjects;
                std::cout << "Find a object " << std::endl;
                bool found = 0;
                    found = JustinaVision::detectObjects(recognizedObjects);
                    if (found) {
                        ss.str("");
                        found = false;
                        vision_msgs::VisionObject aux_vObject = recognizedObjects[0];
                        for(int i = 0; i < recognizedObjects.size() - 1; i++){
                            if(tokens[2] == "right_most" && aux_vObject.pose.position.y > recognizedObjects[i+1].pose.position.y && recognizedObjects[i+1].pose.position.x < 1.0)
                                aux_vObject = recognizedObjects[i+1];
                            if(tokens[2] == "left_most" && aux_vObject.pose.position.y < recognizedObjects[i+1].pose.position.y && recognizedObjects[i+1].pose.position.x < 1.0)
                                aux_vObject = recognizedObjects[i+1];
                        }
                        vision_msgs::VisionObject vObject = aux_vObject;
                        pose = aux_vObject.pose;
                        std::cout << "object left:  " << vObject.id << std::endl;
                        if(tokens[2] == "left_most")
                            ss << vObject.id << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " left";
                        if(tokens[2] == "right_most")
                            ss << vObject.id << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " right";
                        numObj = recognizedObjects.size();
                    }
                    contador++;
            } while( numObj<3 && contador < 10);
            
            success = (numObj == 0)?false:true;

            if(!success)
                JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 4000);
		    
            responseMsg.params = ss.str();

        } else if (tokens[0] == "relpose"){
            ss.str("");
            ss << "I am looking for the object " << tokens[2] << " the " << tokens[3];
            JustinaHRI::waitAfterSay(ss.str(), 2500);
            JustinaManip::hdGoTo(0, -0.9, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaTasks::alignWithTable(0.42);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            int contador = 0;
            bool ref_obj = false;
            int ref_obj_index = 0;
			geometry_msgs::Pose ref_obj_pose;
            float rel_obj_dist = 1000.0;

            ss.str("");
			ss <<"object 2 2 2 left";

            JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 4000);
            do{
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                std::vector<vision_msgs::VisionObject> recognizedObjects;
                std::cout << "Find a object " << std::endl;
                bool found = 0;
                    found = JustinaVision::detectObjects(recognizedObjects);
                    if (found && recognizedObjects.size()> 2) {
                        found = false;
                        for(int i = 0; i < recognizedObjects.size(); i++){
                            if(recognizedObjects[i].id == tokens[3] ){
                                ref_obj = true;
                                ref_obj_index = i;
                                ref_obj_pose = recognizedObjects[i].pose;
                            }
                        }
                        if(ref_obj){
                            ref_obj = false;
                            std::cout << "INIT TO SEARCH CONDITION OF RELPOS" << std::endl;
                            if(tokens[2] == "at_the_left_of"){
                                for(int i = 0; i < recognizedObjects.size(); i++){
                                    std::cout << "AT THE LEFT OF" << std::endl;
                                    if (recognizedObjects[i].pose.position.y > ref_obj_pose.position.y && recognizedObjects[i].pose.position.x < 1.5 
                                            && rel_obj_dist > fabs(recognizedObjects[i].pose.position.y - ref_obj_pose.position.y)){
                                        ss.str("");
                                        ss << recognizedObjects[i].id << " " << recognizedObjects[i].pose.position.x 
                                            << " " << recognizedObjects[i].pose.position.y << " " << recognizedObjects[i].pose.position.z << " left";
                                        ref_obj = true;
                                        std::cout << ss.str() << std::endl;
                                        rel_obj_dist = fabs(recognizedObjects[i].pose.position.y - ref_obj_pose.position.y);
                                    }
                                }
                            }
                            else if(tokens[2] == "at_the_right_of"){
                                for(int i = 0; i < recognizedObjects.size(); i++){
                                    if (recognizedObjects[i].pose.position.y < ref_obj_pose.position.y && recognizedObjects[i].pose.position.x < 1.0
                                            && rel_obj_dist > fabs(recognizedObjects[i].pose.position.y - ref_obj_pose.position.y)){
                                        ss.str("");
                                        ss << recognizedObjects[i].id << " " << recognizedObjects[i].pose.position.x 
                                            << " " << recognizedObjects[i].pose.position.y << " " << recognizedObjects[i].pose.position.z << " right";
                                        ref_obj = true;
                                        rel_obj_dist = fabs(recognizedObjects[i].pose.position.y - ref_obj_pose.position.y);
                                    }
                                }
                            }
                            else if(tokens[2] == "on_top_of" || tokens[2] == "above"){
                                rel_obj_dist = 0.0;
                                for(int i = 0; i < recognizedObjects.size(); i++){
                                    if (recognizedObjects[i].pose.position.z > ref_obj_pose.position.z && recognizedObjects[i].pose.position.x < 1.0
                                            && rel_obj_dist < fabs(recognizedObjects[i].pose.position.z - ref_obj_pose.position.z)){
                                        ss.str("");
                                        ss << recognizedObjects[i].id << " " << recognizedObjects[i].pose.position.x 
                                            << " " << recognizedObjects[i].pose.position.y << " " << recognizedObjects[i].pose.position.z << " right";
                                        ref_obj = true;
                                        rel_obj_dist = fabs(recognizedObjects[i].pose.position.z - ref_obj_pose.position.z);
                                    }
                                }
                            }
                            else if(tokens[2] == "under"){
                                rel_obj_dist = 0.0;
                                for(int i = 0; i < recognizedObjects.size(); i++){
                                    if (recognizedObjects[i].pose.position.z < ref_obj_pose.position.z && recognizedObjects[i].pose.position.x < 1.0
                                            && rel_obj_dist < fabs(recognizedObjects[i].pose.position.z - ref_obj_pose.position.z)){
                                        ss.str("");
                                        ss << recognizedObjects[i].id << " " << recognizedObjects[i].pose.position.x 
                                            << " " << recognizedObjects[i].pose.position.y << " " << recognizedObjects[i].pose.position.z << " right";
                                        ref_obj = true;
                                        rel_obj_dist = fabs(recognizedObjects[i].pose.position.z - ref_obj_pose.position.z);
                                    }
                                }
                            }
                            else if(tokens[2] == "behind"){
                                for(int i = ref_obj_index + 1; i < recognizedObjects.size(); i++){
                                    if(recognizedObjects[i].pose.position.x < 1.5 
                                            && rel_obj_dist > fabs(recognizedObjects[i].pose.position.y - ref_obj_pose.position.y)){
                                        ss.str("");
                                        ss << recognizedObjects[ref_obj_index + 1].id << " " << recognizedObjects[ref_obj_index + 1].pose.position.x 
                                        << " " << recognizedObjects[ref_obj_index + 1].pose.position.y 
                                        << " " << recognizedObjects[ref_obj_index + 1].pose.position.z << " right";
                                        ref_obj = true;
                                        rel_obj_dist = fabs(recognizedObjects[i].pose.position.y - ref_obj_pose.position.y);
                                    }
                                }
                            }
                        }
                    }
                    contador++;
            } while(!ref_obj && contador < 10);

            if(!ref_obj)
                JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 4000);
            
            success = ref_obj;
            responseMsg.params = ss.str();

        
        } else {
			geometry_msgs::Pose pose;
			bool withLeftOrRightArm;
			bool finishMotion = false;
			float pos = POS, advance = ADVANCE, maxAdvance = MAXA;
			do{
                JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 8000);
				success = JustinaTasks::findObject(tokens[0], pose, withLeftOrRightArm);
                JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 8000);
				finishMotion = true;
                //finishMotion = lateralMov(pos, advance, maxAdvance);
			}while(!finishMotion && !success);

			if(withLeftOrRightArm)
				ss << responseMsg.params << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " left";
			else
				ss << responseMsg.params << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " right";
		}
		responseMsg.params = ss.str();
	}
	//JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	if(tokens[0] == "only_find" || tokens[0] == "pose"){
		if(nfp) command_response_pub.publish(responseMsg);}
	else
		{if(nfp) validateAttempsResponse(responseMsg);}
	
}

void callbackCmdFollowToTaxi(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
    std::cout << testPrompt << "----------- Command Follow to taxi" << std::endl;
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
    ros::Rate loop(10);
    
    //JustinaHRI::waitAfterSay("Tell me, here is my taxi, when we reached the taxi, please tell me, follow me, for start following you", 12000, 300);
    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
    int cont_z=0;
    bool reco_follow = false;
    bool frontal_legs = false;
    bool finish_follow = false;
    bool userConfirmation = false;

    std::string lastRecoSpeech;
    
	//JustinaHRI::loadGrammarSpeechRecognized("follow_confirmation.xml");
    switchSpeechReco(6, "Tell me, here is my taxi, when we reached the taxi, please tell me, follow me, for start following you");
    while(!reco_follow){
        if(JustinaHRI::waitForSpecificSentence("follow me" , 15000)){
            reco_follow = true;
        }
        else                    
            cont_z++;    		

        if(cont_z>3){
            JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
            JustinaHRI::waitAfterSay("Please repeat the command", 5000, 300);
            JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
            cont_z=0;
        }
    }
    
    JustinaHRI::waitAfterSay("Human, please put in front of me", 3000, 300);
    JustinaHRI::enableLegFinder(true);
    ros::spinOnce();
    
    while(!frontal_legs){
        if(JustinaHRI::frontalLegsFound()){
            std::cout << "NavigTest.->Frontal legs found!" << std::endl;
            JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
            JustinaHRI::waitAfterSay("I found you, i will start to follow you human, please walk", 10000, 300);
            JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
            JustinaHRI::startFollowHuman();
            ros::spinOnce();
            loop.sleep();
            JustinaHRI::startFollowHuman();
            frontal_legs = true;
        }
        ros::spinOnce();
    }

    while(!finish_follow){
        if(JustinaHRI::waitForSpecificSentence("here is my taxi", 7000)){
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
	            JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
                JustinaHRI::waitAfterSay("is it the taxi, please tell me robot yes, or robot no", 10000, 300);
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                JustinaHRI::waitForUserConfirmation(userConfirmation, 5000);
                if(userConfirmation){
                    JustinaHRI::stopFollowHuman();
                    JustinaHRI::enableLegFinder(false);
                    //JustinaKnowledge::addUpdateKnownLoc("car_location");	
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("I stopped", 2000, 300);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    cont_z=8;
                    finish_follow = true;
                    break;
                }
                else{
	                JustinaHRI::loadGrammarSpeechRecognized("follow_confirmation.xml");
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("Ok, please walk", 3000, 300);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                }
        }
        if(!JustinaHRI::frontalLegsFound()){
            JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
            JustinaHRI::waitAfterSay("I lost you, please put in front of me again", 5500, 300);
            JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
            JustinaHRI::stopFollowHuman();
            JustinaHRI::enableLegFinder(false);
            ros::spinOnce();
            frontal_legs = false;
            JustinaHRI::enableLegFinder(true);
            ros::spinOnce();
            while(!frontal_legs){
                if(JustinaHRI::frontalLegsFound()){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("I found you, please walk", 4000, 300);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    JustinaHRI::startFollowHuman();
                    ros::spinOnce();
                    loop.sleep();
                    JustinaHRI::startFollowHuman();
                    frontal_legs = true;
                }
                ros::spinOnce();
            }
        }        
        ros::spinOnce();
    }
	
    //JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);

    responseMsg.successful = 1;
    command_response_pub.publish(responseMsg);
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

	bool finishMotion = false;
	float pos = POS, advance = ADVANCE, maxAdvance = MAXA;

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	
	ss.str("");
	ss << "I am looking for " << tokens[0] << " on the " << tokens[1];
	JustinaHRI::waitAfterSay(ss.str(), 2500);
	JustinaManip::hdGoTo(0, -0.9, 5000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
	JustinaTasks::alignWithTable(0.42);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	int numObj  = 0;
	std::string query;
	std::vector<std::string> objects;

    JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 4000);
	do{
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		std::vector<vision_msgs::VisionObject> recognizedObjects;
		std::cout << "Find a object " << std::endl;
		bool found = false;
		for (int j = 0; j < 10; j++) {
			std::cout << "Test object" << std::endl;
            numObj = 0;
            objects.clear();
			found = JustinaVision::detectObjects(recognizedObjects);
			if (found) {
				found = false;
				for (int i = 0; i < recognizedObjects.size(); i++) {
					vision_msgs::VisionObject vObject = recognizedObjects[i];
					std::cout << "object:  " << vObject.id << std::endl;
                    ss.str("");
                    ss << "(assert (cmd_simple_category " << vObject.id <<" 1))";
                    JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);
					//std::map<std::string, std::string>::iterator it = catList.find(vObject.id);
                    if(query == tokens[0])
                       numObj++;
			            objects.push_back(vObject.id);
				}
			}
		}
		finishMotion = true;
        //finishMotion = lateralMov(pos, advance, maxAdvance);
	}while(!finishMotion && numObj<1);
    JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 4000);

	ss.str("");
	currentName = tokens[0];
	if(numObj > 0){
		ss << "I found the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(), 2500);
		ss.str("");
		ss << "I found ";
		for (int i = 0; i < objects.size(); i++){
			if(i == objects.size() - 1 && i != 0)
				ss << " and the " << objects.at(i);
			else
				ss << "the " << objects.at(i) << ", ";
		}
		ss.str();
		ss << "I found " << numObj << " " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(), 2500);
		ss.str("");
		ss << responseMsg.params << " " << numObj;
		cantidad = numObj;
		currentName = tokens[0];
		responseMsg.params = ss.str();
		responseMsg.successful = 1;
	}
	else {
		ss << "I can not find the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(), 2500);
		ss.str("");
		cantidad = 0;
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

	int numObj = 0;

	bool finishMotion = false;
	float pos = POS, advance = ADVANCE, maxAdvance = MAXA;

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	
	ss.str("");
	ss << "I am looking for the " << tokens[0];
	JustinaHRI::waitAfterSay(ss.str(), 2500);
	
	JustinaManip::hdGoTo(0, -0.9, 5000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
	JustinaTasks::alignWithTable(0.42);

    JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 4000);
	do{
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		std::vector<vision_msgs::VisionObject> recognizedObjects;
		std::cout << "Find a object " << std::endl;
		bool found = false;
		for (int j = 0; j < 10; j++) {
            numObj = 0;
			std::cout << "Test object" << std::endl;
			found = JustinaVision::detectObjects(recognizedObjects);
			if (found) {
				found = false;
				for (int i = 0; i < recognizedObjects.size(); i++) {
					vision_msgs::VisionObject vObject = recognizedObjects[i];
					std::cout << "object:  " << vObject.id << std::endl;
					//std::map<std::string, int>::iterator it = countObj.find(vObject.id);
					if (vObject.id == tokens[0])
						numObj++;
				}
			}
		}
		finishMotion = true;
        //finishMotion = lateralMov(pos, advance, maxAdvance);
	}while (!finishMotion);
    JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 4000);

	ss.str("");
	currentName = tokens[0];
	if(numObj > 0){
		ss << "I found the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(), 2500);
		ss.str("");
		cantidad = numObj;
		ss << responseMsg.params << " " << cantidad;
		responseMsg.params = ss.str();
		responseMsg.successful = 1;
	}
	else {
		ss << "I can not find the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(),2500);
		ss.str("");
		cantidad = 0;
		ss << responseMsg.params << " " << 0;
		responseMsg.params = ss.str();
		responseMsg.successful = 0;
	}

	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackOpropObject(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command The Oprop Object on the placement ---------" << std::endl;
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


	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	
	std::string prop;
	std::string opropObj;
	std::vector<std::string> objects;
    std::vector<int> obj_index;
    std::vector<propObj> prop_obj;
    std::vector<poseObj> pose_obj;
	currentName = tokens[1];
	categoryName = tokens[2];
    int obj_ind = 0;
    std::pair<bool, int> element_index;

	bool finishMotion = false;
	float pos = POS, advance = ADVANCE, maxAdvance = MAXA;

    std::string query;
    std::string category;
    
    category = tokens[2];
    if(category == "nil")
        category = "objects";

	JustinaHRI::waitAfterSay("I am looking for objects", 2500);
	JustinaManip::hdGoTo(0, -0.9, 5000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
	JustinaTasks::alignWithTable(0.42);
	std::vector<vision_msgs::VisionObject> recognizedObjects;
    	
    JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 4000);
	do{	
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		std::cout << "Find a object " << std::endl;
		bool found = 0;
		for (int j = 0; j < 10; j++) {
			std::cout << "Test object" << std::endl;
			found = JustinaVision::detectObjects(recognizedObjects);
			if (found) {
				found = false;
				for (int i = 0; i < recognizedObjects.size(); i++) {
					vision_msgs::VisionObject vObject = recognizedObjects[i];
                    poseObj pObj;
                    pObj.x = vObject.pose.position.x;
                    pObj.y = vObject.pose.position.y;
                    pObj.z = vObject.pose.position.z;
					std::cout << "object:  " << vObject.id << std::endl;
                    std::pair<bool, int> element_in_vector = findInVector<std::string>(objects, vObject.id); 
					if (tokens[2] == "nil" && !element_in_vector.first){
						objects.push_back(vObject.id);
                        obj_index.push_back(i);
                        pose_obj.push_back(pObj); 
						std::cout << "OBJETO: " << vObject.id << std::endl;
					}
					if (tokens[2] != "nil" && !element_in_vector.first){
                        ss.str("");
                        ss << "(assert (cmd_simple_category " << vObject.id << " 1))";
				        JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);
                        if(query == tokens[2]){
    						objects.push_back(vObject.id);
                            obj_index.push_back(i);
                            pose_obj.push_back(pObj); 
					    	std::cout << "OBJETO: " << vObject.id << std::endl;
                        }
					}
						
				}
			}
		}
		finishMotion = true;
        //finishMotion = lateralMov(pos, advance, maxAdvance);
	}while(!finishMotion);
	
	
	if(objects.size() == 0){
		objectName = "none";
		responseMsg.successful = 0;
        ss.str("");
        ss << tokens[0] << " " << tokens[1] << " " << tokens[2] << " " 
            << "I_am_sorry,_i_can_not_find_the_three_" << tokens[1] << "_" << category;
	}

	else if (objects.size() == 1){
		std::cout << "There are only one object" << std::endl;
		objectName = objects.at(0);
		responseMsg.successful = 1;
        ss.str("");
        ss << tokens[0] << " " << tokens[1] << " " << tokens[2] << " " 
            << "The_" << tokens[1] << "_" << category <<"_that_i_found_is_";
        propObj obj_aux;
        obj_aux.obj = objects.at(0);
        obj_aux.value = 1;
        prop_obj.push_back(obj_aux);
        obj_ind = 0;
	}

    if (objects.size() > 1 && tokens[0] != "color" && tokens[0] != "abspose"){
        propObj obj_aux;
        for(int i = 0; i < objects.size(); i++){
            ss.str("");
            ss << "(assert (cmd_get_prop_value " << objects.at(i)<< " " << tokens[1] << " 1))";
            JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);
            obj_aux.obj = objects.at(i);
            obj_aux.value = std::atoi(query.c_str());
            prop_obj.push_back(obj_aux);
        }
        if(tokens[1] == "biggest" || tokens[1] == "heaviest" || tokens[1] == "largest" || tokens[1] == "tallest"){
            std::sort(prop_obj.begin(), prop_obj.end(), compareDownward);
        }
        else{
            std::sort(prop_obj.begin(), prop_obj.end(), compareUpward);
        }
        objectName = prop_obj.at(0).obj;
        element_index = findInVector<std::string>(objects, objectName);
        obj_ind = element_index.second;
        responseMsg.successful = 1;
        ss.str("");
        ss << tokens[0] << " " << tokens[1] << " " << tokens[2] << " " 
            << "The_" << tokens[1] << "_"<< category <<"_that_i_found_are_";
    }

    if (objects.size() > 0 && tokens[0] == "color"){
        int i = 0;
        bool fcolor = false;
        while( !fcolor && i < objects.size()){
            ss.str("");
            ss << "(assert (cmd_compare_color " << objects.at(i) << " " << tokens[1] << " 1))";
            JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);
            if(query == "true")
                fcolor = true;
            i++;
        }

        if(fcolor){
            objectName = objects.at(i-1);
            obj_ind = i-1;
            responseMsg.successful = 1;

        }
        else
            objectName = "none";

    }

    if (objects.size() > 1 && tokens[0] == "abspose"){
            //std::pair<bool, int> element_index = findInVector<std::string>(objects, objects.at(0));
            //int index = obj_index.at(0);
            //vision_msgs::VisionObject aux_vObject = recognizedObjects[index];
            std::string aux_obj;
            aux_obj = objects.at(0);
            obj_ind = 0;
            for(int i = 1; i < objects.size(); i++){
                //element_index = findInVector<std::string>(objects, objects.at(i));
                //index = obj_index.at(i);
                if(tokens[1] == "right_most" && pose_obj[i-1].y > pose_obj[i].y && pose_obj[i].x < 1.0){
                    aux_obj = objects[i];
                    obj_ind = i;
                }
                if(tokens[1] == "left_most" && pose_obj[i-1].y < pose_obj[i].y && pose_obj[i].x < 1.0){
                    aux_obj = objects[i];
                    obj_ind = i;
                }
            }
            objectName = aux_obj;
            responseMsg.successful = 1;
    }

    if(tokens[0] == "find_three_obj"){
        int i = 0;
        JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 4000);
        while (i < 3 && i < prop_obj.size()){
                if (i == 2)
                    ss << "and_the_" << prop_obj.at(i).obj;
                else if (i == 1 && prop_obj.size() == 2)
                    ss << "and_the_" << prop_obj.at(i).obj;
                else 
                    ss << "the_" << prop_obj.at(i).obj << ",_";
                i++;
        }
        responseMsg.params = ss.str();
    }

    if(tokens[0] == "for_grasp" || tokens[0] == "abspose" || tokens[0] == "color" || tokens[0] == "property"){
        if (objectName == "none"){
            JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 4000);
            ss.str("");
			ss <<"object 2 2 2 left";
            responseMsg.params = ss.str();
            responseMsg.successful = 0;
        }
        else{
            //std::pair<bool, int> element_index = findInVector<std::string>(objects, objectName);
            //int ind = obj_index.at(element_index.second);
            ss.str("");
            std::cout << "objectName: " << objectName << " recoObject: " << objects[obj_ind] << std::endl;
            if(pose_obj[obj_ind].y > 0){
                ss << objectName << " " << pose_obj[obj_ind].x << " " 
                    << pose_obj[obj_ind].y << " " << pose_obj[obj_ind].z << " left";
            }
            else{
                ss << objectName << " " << pose_obj[obj_ind].x << " " 
                    << pose_obj[obj_ind].y << " " << pose_obj[obj_ind].z << " right";
                
            }
            responseMsg.params = ss.str();
            responseMsg.successful = 1;
        }
    }

    objects.clear();
    pose_obj.clear();
    obj_index.clear();
    prop_obj.clear();

	command_response_pub.publish(responseMsg);
}

void callbackGesturePerson(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Find the Gender,Gesture or Pose Person ---------" << std::endl;
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

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	bool nfp = validateContinuePlan(d.toSec(), fplan);
	
	if(tokens[0] == "waving"){
		std::cout << "Searching waving person" << std::endl;
		if(nfp) JustinaTasks::findGesturePerson(tokens[0], tokens[1]);
	}
	else if (tokens[0] == "rising_right_arm" || tokens[0] == "raising_their_right_arm"){
		std::cout << "Searching rising_right_arm person" << std::endl;
		if(nfp) JustinaTasks::findGesturePerson("right_hand_rised", tokens[1]);
	}
	else if (tokens[0] == "rising_left_arm" || tokens[0] == "raising_their_left_arm"){
		std::cout << "Searching rising_left_arm person" << std::endl;
		if(nfp) JustinaTasks::findGesturePerson("left_hand_rised", tokens[1]);
	}
	else if (tokens[0] == "pointing_to_the_right"){
		std::cout << "Searching pointing_right person" << std::endl;
		if(nfp) JustinaTasks::findGesturePerson("pointing_right", tokens[1]);
	}
	else if (tokens[0] == "pointing_to_the_left"){
		std::cout << "Searching pointing_left person" << std::endl;
		if(nfp) JustinaTasks::findGesturePerson("pointing_left", tokens[1]);
	}
	else if (tokens[0] == "sitting"){
		std::cout << "Searching sitting person" << std::endl;
		if(nfp) JustinaTasks::findPerson("", -1, JustinaTasks::SITTING, false, tokens[1]);
	}
	else if (tokens[0] == "standing"){
		std::cout << "Searching standing person" << std::endl;
		if(nfp) JustinaTasks::findPerson("", -1, JustinaTasks::STANDING, false, tokens[1]);
	}
	else if (tokens[0] == "lying"){
		std::cout << "Searching lying person" << std::endl;
		if(nfp) JustinaTasks::findPerson("", -1, JustinaTasks::LYING, false, tokens[1]);
	}
	else if (tokens[0] == "man"|| tokens[0] == "boy" || tokens[0] == "male_person"  || tokens[0] == "male"){
		std::cout << "Searching man person" << std::endl;
		if(nfp) JustinaTasks::findPerson("", 1, JustinaTasks::NONE, false, tokens[1]);
	}
	else if (tokens[0] == "woman" || tokens[0] == "girl" || tokens[0] == "female_person" || tokens[0] == "female"){
		std::cout << "Searching woman person" << std::endl;
		if(nfp) JustinaTasks::findPerson("", 0, JustinaTasks::NONE, false, tokens[1]);
	}

	
	//success = JustinaTasks::findPerson();

	if(nfp) command_response_pub.publish(responseMsg);
}

void callbackGPPerson(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Find which gender or pose have the person ---------" << std::endl;
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
	std::string gender;
	bool success;

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;

	ss.str("");	
	if(tokens[0] == "gender"){
		std::cout << "Searching person gender" << std::endl;
		success = JustinaTasks::tellGenderPerson(gender, tokens[1]);
		if (success){
			ss << "you are a " << gender; 
			std::cout << "Genero " << gender << std::endl; 
			currentName = gender;
			JustinaHRI::waitAfterSay(ss.str(), 10000);
			}
		else
			currentName = "no_gender_pose";
	}
	else if (tokens[0] == "pose"){std::cout << "Searching person pose" << std::endl;
			//JustinaTasks::findPerson("", -1, JustinaTasks::NONE, false, tokens[1]);
			JustinaTasks::POSE poseRecog;
            		poseRecog = JustinaTasks::NONE;
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            		success = JustinaTasks::findYolo(idsPerson, poseRecog, JustinaTasks::NONE, tokens[1]);
			if(poseRecog == JustinaTasks::NONE || poseRecog == JustinaTasks::STANDING)
				currentName = "standing";
			else if (poseRecog == JustinaTasks::SITTING)
				currentName = "sitting";
			else if (poseRecog == JustinaTasks::LYING)
				currentName = "lying";
			ss << "you are " << currentName;
			JustinaHRI::waitAfterSay(ss.str(), 10000);
	}

	command_response_pub.publish(responseMsg);
}

void callbackGPCrowd(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Find which gender or pose have the crowd ---------" << std::endl;
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

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	
	women = 0;
	men = 0;
	sitting = 0;
	standing = 0;
	lying = 0;
	
	JustinaTasks::findCrowd(men, women, sitting, standing, lying, tokens[1]);
	
	currentName = tokens[0];

	ss.str("");	
	if(tokens[0] == "men"){std::cout << "Searching person men" << std::endl;
		ss << "I found " << men << " men";
		JustinaHRI::waitAfterSay(ss.str(), 10000);
	}
	else if (tokens[0] == "women"){std::cout << "Searching women in the crowd" << std::endl;
		ss << "I found " << women << " women";
		JustinaHRI::waitAfterSay(ss.str(), 10000);}
	else if (tokens[0] == "boys"){std::cout << "Searching boys in the crowd" << std::endl;
		ss << "I found " << men << " boys";
		JustinaHRI::waitAfterSay(ss.str(), 10000);}
	else if (tokens[0] == "girls"){std::cout << "Searching girls in the crowd" << std::endl;
		ss << "I found " << women << " girls";
		JustinaHRI::waitAfterSay(ss.str(), 10000);}
	else if (tokens[0] == "male"){std::cout << "Searching male in the crowd" << std::endl;
		ss << "I found " << men << " male";
		JustinaHRI::waitAfterSay(ss.str(), 10000);}
	else if (tokens[0] == "famale"){std::cout << "Searching female in the crowd" << std::endl;
		ss << "I found " << men << " female";
		JustinaHRI::waitAfterSay(ss.str(), 10000);}
	else if (tokens[0] == "sitting"){std::cout << "Searching sitting in the crowd" << std::endl;
		ss << "I found " << sitting << " sitting people";
		JustinaHRI::waitAfterSay(ss.str(), 10000);}
	else if (tokens[0] == "standing"){std::cout << "Searching standing in the crowd" << std::endl;
		ss << "I found " << standing << " standing people";
		JustinaHRI::waitAfterSay(ss.str(), 10000);}
	else if (tokens[0] == "lying"){std::cout << "Searching lying in the crowd" << std::endl;
		ss << "I found " << lying << " lying people";
		JustinaHRI::waitAfterSay(ss.str(), 10000);}


	command_response_pub.publish(responseMsg);
}

void callbackCmdAskIncomplete(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command ask for incomplete information ---------" << std::endl;
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

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	
	bool conf = false;
	int count  = 0;
	std::string lastReco;
	std::string name;
	responseMsg.successful = 0;

    JustinaHRI::waitAfterSay("I have incomplete information, I need your help please", 10000);    
	ss.str("");
	if(tokens[0] == "follow_place_origin" || tokens[0] == "gesture_place_origin" || tokens[0] == "place_destiny")
		JustinaHRI::waitAfterSay(" in order to response my question, Say for instance, at the center table", 10000);
	if(tokens[0] == "object")
		JustinaHRI::waitAfterSay(" in order to response my question, Say for instance, i want pringles", 10000);
	
    while(!conf && count < 3){
	
	ss.str("");
	if(tokens[0] == "follow_place_origin"){
		//JustinaHRI::loadGrammarSpeechRecognized("incomplete_place.xml");
		ss << "Well, tell me where can i find " << tokens[2]; 
		//JustinaHRI::waitAfterSay(ss.str(), 10000);}	
        switchSpeechReco(7, ss.str());}
	if(tokens[0] == "gesture_place_origin"){
		//JustinaHRI::loadGrammarSpeechRecognized("incomplete_place.xml");
		ss << "Well, tell me where can i find a " << tokens[2] << " person"; 
		//JustinaHRI::waitAfterSay(ss.str(), 10000);}	
        switchSpeechReco(7, ss.str());}
	if(tokens[0] == "object"){
		//JustinaHRI::loadGrammarSpeechRecognized("incomplete_object.xml");
		ss << "Well, tell me what " << tokens[2] << " do you want me to look for";
		//JustinaHRI::waitAfterSay(ss.str(), 10000);}
        switchSpeechReco(8, ss.str());}
	if(tokens[0] == "place_destiny"){
		//JustinaHRI::loadGrammarSpeechRecognized("incomplete_place.xml");
        ss << "Well, tell me what place you want me to guide " << tokens[2];
		//JustinaHRI::waitAfterSay(ss.str(), 10000);}
        switchSpeechReco(7, ss.str());}
	ss.str("");

        JustinaHRI::waitForSpeechRecognized(lastReco,400);
        if(JustinaHRI::waitForSpeechRecognized(lastReco,10000)){
            if(JustinaRepresentation::stringInterpretation(lastReco, name))
                std::cout << "last int: " << name << std::endl;
                ss.str("");
		if(tokens[0] == "follow_place_origin" || tokens[0] == "gesture_place_origin")
			ss << "can i find " << tokens[2] << " in the " << name << ", say robot yes or robot no";
		else if(tokens[0] == "object")
			ss << "Do you want i look for the " << name << ", say robot yes or robot no";
		else if(tokens[0] == "place_destiny")
			ss << "Do you want i guide " << tokens[2] << " to the " << name << ", say robot yes or robot no";
		//JustinaHRI::waitAfterSay(ss.str(), 2000);
		//change grammar
        switchSpeechReco(0, ss.str());
		//JustinaHRI::loadGrammarSpeechRecognized("confirmation.xml");
                JustinaHRI::waitForSpeechRecognized(lastReco,400);
                
                JustinaHRI::waitForSpeechRecognized(lastReco,10000);
                if(lastReco == "robot yes" || lastReco == "justina yes"){
			conf = true;
			ss.str("");
			if(tokens[0] == "follow_place_origin" || tokens[0] == "gesture_place_origin")
				ss << "Well i will find the person in the " << name;
			else if(tokens[0] == "object")
				ss << "well i try to find the " << name << " in its default location";
			else if(tokens[0] == "place_destiny")
				ss << "well i will guide her to the " << name;
			JustinaHRI::waitAfterSay(ss.str(), 2000);
			ss.str("");
			ss << msg->params << " " << name;
			responseMsg.params = ss.str();
			responseMsg.successful = 1;
		}
		else{
			ss.str("");
			ss << msg->params << " nil";
			responseMsg.params = ss.str();
			responseMsg.successful = 0;
		}
                count++;
        }
    }

	command_response_pub.publish(responseMsg);
	//JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
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

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	
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

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	
	bool success = JustinaTasks::alignWithTable(0.42);

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
	std::stringstream ss;

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	
	bool success = ros::service::waitForService("spg_say", 5000);
	//success = success & tasks.moveActuator(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()), tokens[0]);
	if(tokens[4] == "false")
			armFlag = false;

	ss << "I'm going to grasp the " << tokens[0];
	//JustinaHRI::waitAfterSay(ss.str(), 10000);
	
	//success = success & JustinaTasks::moveActuatorToGrasp(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()), armFlag, tokens[0], true);
    JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 8000);
    JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
    success = success && JustinaTasks::graspObject(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()), armFlag, tokens[0], true);
    JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
	if (success)
		responseMsg.successful = 1;
	else{
		ss.str("");
		ss << "I did not grasp the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(), 100000);
		responseMsg.successful = 0;
	}

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

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	bool nfp = validateContinuePlan(d.toSec(), fplan);
	
	if(tokens[2] == "false")
			armFlag = false;

	if(tokens[0] == "person" && nfp)
		succes = JustinaTasks::dropObject(tokens[1], armFlag, 30000);
	else if(tokens[0] == "object" && nfp ){
		ss.str("");
		ss << "I am going to deliver the " << tokens[1];
		JustinaHRI::waitAfterSay(ss.str(), 2000);
		succes = JustinaTasks::placeObject(armFlag);
		(armFlag) ? JustinaManip::laGoTo("home", 6000) : JustinaManip::raGoTo("home", 6000);
	}
	
	if (succes)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;

	if(nfp) validateAttempsResponse(responseMsg);
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

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
    std::string lastReco;
	
	responseMsg.successful = 0;
	JustinaManip::startHdGoTo(0, 0.0);
    
    std::string to_spech = responseMsg.params;
    boost::replace_all(to_spech, "_", " ");
    std::stringstream ss;
    ss << "Hello, I am Justina, Well, Is your name, " << to_spech << ", say robot yes or robot no";
    //JustinaHRI::waitAfterSay(ss.str(), 1500);
    //ss << "Well, " << to_spech << " is your name";
    std::cout << "------------- to_spech: ------------------ " << ss.str() << std::endl;

    //JustinaHRI::waitAfterSay(ss.str(), 10000);
    switchSpeechReco(0, ss.str());

    JustinaHRI::waitForSpeechRecognized(lastReco,10000);
    if(lastReco == "robot yes" || lastReco == "justina yes"){
        responseMsg.successful = true;
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
        responseMsg.successful = false;
    }

	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackCmdTaskConfirmation( const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
    std::cout << testPrompt << "--------- Command confirm task -------" << std::endl;
    std::cout << "name: " << msg->name << std::endl;
    std::cout << "params: " << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;

    std::string lastRecoSpeech;    

    std::string to_spech = responseMsg.params;
    boost::replace_all(to_spech, "_", " ");
    std::stringstream ss;

    std::string lastReco;

    ss << to_spech;
    std::cout << "------------- to_spech: ------------------ " << ss.str()
        << std::endl;
    //JustinaHRI::waitAfterSay(ss.str(), 2000);
    switchSpeechReco(0, ss.str());

    JustinaHRI::waitForSpeechRecognized(lastReco,10000);
    if(lastReco == "robot yes" || lastReco == "justina yes"){
        responseMsg.successful = true;
        responseMsg.params = "conf";
    }
    else{
        responseMsg.successful = false;
        responseMsg.params = "conf";
		JustinaNavigation::moveDistAngle(0, 1.57, 10000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
    }

    command_response_pub.publish(responseMsg);

}

/// eegpsr category II callbacks
void callbackManyPeople(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command How Many People ---------"
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

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	
	women = 0;
	men = 0;
	sitting = 0;
	standing = 0;
	lying = 0;
	
	JustinaTasks::findCrowd(men, women, sitting, standing, lying, tokens[1]);
	
	currentName = tokens[0];

	ss.str("");	
	if(tokens[0] == "men"){std::cout << "Searching person men" << std::endl;
		ss << "I found " << men << " men";
		JustinaHRI::waitAfterSay(ss.str(), 10000);
	    ss.str("");	
		ss << str << " I_found_" << men << "_men";
	}
	else if (tokens[0] == "women"){std::cout << "Searching women in the crowd" << std::endl;
		ss << "I found " << women << " women";
		JustinaHRI::waitAfterSay(ss.str(), 10000);
	    ss.str("");
		ss << str << " I_found_" << women << "_women";
    }	
	else if (tokens[0] == "children"){std::cout << "Searching childre in the crowd" << std::endl;
		ss << "I found " << men + women << " children";
		JustinaHRI::waitAfterSay(ss.str(), 10000);
        ss.str("");
		ss << str << " I_found_" << men + women << "_children";
    }
	else if (tokens[0] == "elders"){std::cout << "Searching elders in the crowd" << std::endl;
		ss << "I found " << women + men << " elders";
		JustinaHRI::waitAfterSay(ss.str(), 10000);
        ss.str("");
		ss << str << " I_found_" << women + men << "_elders";
    }
	else if (tokens[0] == "man"){std::cout << "Searching male in the crowd" << std::endl;
		ss << "I found " << men + women << " people";
		JustinaHRI::waitAfterSay(ss.str(), 10000);
        ss.str();
		ss << str << " I_found_" << men + women << "_people";
    }
    else if (tokens[0] == "people"){
        ss << "I found " << men + women << " people";
        JustinaHRI::waitAfterSay(ss.str(), 10000);
        ss.str("");
        ss << str << " I_found_" << men + women << "_people";
    }
    
    responseMsg.params = ss.str();
//	command_response_pub.publish(responseMsg);

	responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackAmountPeople(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Amount of people ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackAskAndOffer(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command ask and offer drink or eat ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
    std::vector<std::string> tokens;
    std::vector<std::string> tokens1;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;
    std::stringstream ss1;
    std::string lastRecoSpeech;
    int timeoutspeech = 10000;
    bool conf = false;
    int intentos = 0;

    while(intentos < 3 && !conf){
        ss.str("");
        ss << "Please tell me which drink, do you want";
        JustinaHRI::loadGrammarSpeechRecognized("restaurant_beverage.xml");

        if(tokens[1] == "eat"){
            ss.str("");
            ss << "Please tell me what you want to eat";
            JustinaHRI::loadGrammarSpeechRecognized("eegpsr_food.xml");
        }

        JustinaHRI::waitAfterSay(ss.str(), 5000);

        if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeoutspeech)){
            split(tokens1, lastRecoSpeech, is_any_of(" "));
            ss1.str("");
            if(tokens1.size() == 4)
               ss1 << "do you want " << tokens1[3];
            else if (tokens1.size() == 5)
                ss1 << "do you want " << tokens1[3] << " " << tokens1[4];

            JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
            JustinaHRI::waitAfterSay(ss1.str(), 5000);    
            
            knowledge_msgs::planning_cmd srv;
            srv.request.name = "test_confirmation";
            srv.request.params = responseMsg.params;
            if (srvCltWaitConfirmation.call(srv)) {
                std::cout << "Response of confirmation:" << std::endl;
                std::cout << "Success:" << (long int) srv.response.success
                    << std::endl;
                std::cout << "Args:" << srv.response.args << std::endl;
                //responseMsg.params = "conf";
                //responseMsg.successful = srv.response.success;
            } else {
                std::cout << testPrompt << "Failed to call service of confirmation"
                    << std::endl;
                //responseMsg.successful = 0;
                JustinaHRI::waitAfterSay("Repeate the command please", 1000);
            }
            
            if( (long int) srv.response.success == 1 ){
                JustinaHRI::waitAfterSay("Ok i will remember your order", 5000);
                std_msgs::String res1;
                ss1.str("");
                if(tokens1.size() == 4)
                    ss1 << "(assert (cmd_add_order " << tokens[1] << " " << tokens1[3] << " 1))";
                else if(tokens1.size() == 5)
                    ss1 << "(assert (cmd_add_order " << tokens[1] << " " << tokens1[3] << "_" << tokens1[4] << " 1))";
                res1.data = ss1.str();
                sendAndRunClips_pub.publish(res1);
                conf = true;
            }
            else{
                intentos++;
                JustinaHRI::waitAfterSay("Sorry I did not understand you", 5000);
            }

       }
    }

    //if (intentos > 2 && !conf)
        JustinaHRI::waitAfterSay("Thank you", 5000);
    
	responseMsg.successful = 1;
	validateAttempsResponse(responseMsg);
	//JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
	//command_response_pub.publish(responseMsg);
}

void callbackFindEPerson(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Find Endurance Person ---------"
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

	bool fp  = false;

    if(tokens.size() == 2){
        ///buscar solo una persona
        //fp = JustinaTasks::findSkeletonPerson(JustinaTasks::NONE, tokens[1]);
            if(tokens[0] == "waving"){
                std::cout << "Searching waving person" << std::endl;
                fp = JustinaTasks::findGesturePerson(tokens[0], tokens[1]);
            }
            else if (tokens[0] == "raising_their_right_arm"){
                std::cout << "Searching raising_their_right_arm person" << std::endl;
                fp = JustinaTasks::findGesturePerson("right_hand_rised", tokens[1]);
            }
            else if (tokens[0] == "raising_their_left_arm"){
                std::cout << "Searching rising_their_left_arm person" << std::endl;
                fp = JustinaTasks::findGesturePerson("left_hand_rised", tokens[1]);
            }
            else if (tokens[0] == "pointing_to_the_right"){
                std::cout << "Searching pointing1right person" << std::endl;
                fp = JustinaTasks::findGesturePerson("pointing_right", tokens[1]);
            }
            else if (tokens[0] == "pointing_to_the_left"){
                std::cout << "Searching pointing_left person" << std::endl;
                fp = JustinaTasks::findGesturePerson("pointing_left", tokens[1]);
            }
            else if(tokens[0] == "standing"){
                std::cout << "Searching standing person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::STANDING, tokens[1]);
            }
            else if(tokens[0] == "sitting"){
                std::cout << "searching sitting person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::SITTING, tokens[1]);
            }
            else if(tokens[0] == "lying"){
                std::cout << "searching lying person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::LYING, tokens[1]);
            }
            else{
                std::cout << "Searching a pose, color or outfit person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::NONE, tokens[1]);
            }
    }

    if(tokens.size() == 3){
        ///buscar persona con gesto
            if(tokens[1] == "waving"){
                std::cout << "Searching waving person" << std::endl;
                fp = JustinaTasks::findGesturePerson(tokens[1], tokens[2]);
            }
            else if (tokens[1] == "raising_their_right_arm"){
                std::cout << "Searching raising_their_right_arm person" << std::endl;
                fp = JustinaTasks::findGesturePerson("right_hand_rised", tokens[2]);
            }
            else if (tokens[1] == "raising_their_left_arm"){
                std::cout << "Searching rising_their_left_arm person" << std::endl;
                fp = JustinaTasks::findGesturePerson("left_hand_rised", tokens[2]);
            }
            else if (tokens[1] == "pointing_to_the_right"){
                std::cout << "Searching pointing1right person" << std::endl;
                fp = JustinaTasks::findGesturePerson("pointing_right", tokens[2]);
            }
            else if (tokens[1] == "pointing_to_the_left"){
                std::cout << "Searching pointing_left person" << std::endl;
                fp = JustinaTasks::findGesturePerson("pointing_left", tokens[2]);
            }
            else if(tokens[1] == "standing"){
                std::cout << "Searching standing person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::STANDING, tokens[2]);
            }
            else if(tokens[1] == "sitting"){
                std::cout << "searching sitting person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::SITTING, tokens[2]);
            }
            else if(tokens[1] == "lying"){
                std::cout << "searching lying person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::LYING, tokens[2]);
            }
            else{
                std::cout << "Searching a pose, color or outfit person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::NONE, tokens[2]);
            }

        //buscar persona con pose
        
        //buscar persona con color
        
        //buscarpersona con outfit
    }
    
    if(tokens.size() == 4){

        //buscar persona con color outfit
        //std::cout << "Searching a color outfit person" << std::endl;
        //JustinaTasks::findSkeletonPerson(JustinaTasks::NONE, tokens[3]);
            if(tokens[1] == "waving"){
                std::cout << "Searching waving person" << std::endl;
                fp = JustinaTasks::findGesturePerson(tokens[1], tokens[3]);
            }
            else if (tokens[1] == "raising_their_right_arm"){
                std::cout << "Searching raising_their_right_arm person" << std::endl;
                fp = JustinaTasks::findGesturePerson("right_hand_rised", tokens[3]);
            }
            else if (tokens[1] == "raising_their_left_arm"){
                std::cout << "Searching rising_their_left_arm person" << std::endl;
                fp = JustinaTasks::findGesturePerson("left_hand_rised", tokens[3]);
            }
            else if (tokens[1] == "pointing_to_the_right"){
                std::cout << "Searching pointing1right person" << std::endl;
                fp = JustinaTasks::findGesturePerson("pointing_right", tokens[3]);
            }
            else if (tokens[1] == "pointing_to_the_left"){
                std::cout << "Searching pointing_left person" << std::endl;
                fp = JustinaTasks::findGesturePerson("pointing_left", tokens[3]);
            }
            else if(tokens[1] == "standing"){
                std::cout << "Searching standing person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::STANDING, tokens[3]);
            }
            else if(tokens[1] == "sitting"){
                std::cout << "searching sitting person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::SITTING, tokens[3]);
            }
            else if(tokens[1] == "lying"){
                std::cout << "searching lying person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::LYING, tokens[3]);
            }
            else{
                std::cout << "Searching a pose, color or outfit person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::NONE, tokens[3]);
            }

    }

    if(tokens.size() == 5){
        //std::cout << "Searching ei person" << std::endl;
        //JustinaTasks::findSkeletonPerson(JustinaTasks::NONE, tokens[4]);
            if(tokens[1] == "waving"){
                std::cout << "Searching waving person" << std::endl;
                fp = JustinaTasks::findGesturePerson(tokens[1], tokens[4]);
            }
            else if (tokens[1] == "raising_their_right_arm"){
                std::cout << "Searching raising_their_right_arm person" << std::endl;
                fp = JustinaTasks::findGesturePerson("right_hand_rised", tokens[4]);
            }
            else if (tokens[1] == "raising_their_left_arm"){
                std::cout << "Searching rising_their_left_arm person" << std::endl;
                fp = JustinaTasks::findGesturePerson("left_hand_rised", tokens[4]);
            }
            else if (tokens[1] == "pointing_to_the_right"){
                std::cout << "Searching pointing1right person" << std::endl;
                fp = JustinaTasks::findGesturePerson("pointing_right", tokens[4]);
            }
            else if (tokens[1] == "pointing_to_the_left"){
                std::cout << "Searching pointing_left person" << std::endl;
                fp = JustinaTasks::findGesturePerson("pointing_left", tokens[4]);
            }
            else if(tokens[1] == "standing"){
                std::cout << "Searching standing person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::STANDING, tokens[4]);
            }
            else if(tokens[1] == "sitting"){
                std::cout << "searching sitting person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::SITTING, tokens[4]);
            }
            else if(tokens[1] == "lying"){
                std::cout << "searching lying person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::LYING, tokens[4]);
            }
            else{
                std::cout << "Searching a pose, color or outfit person" << std::endl;
                fp = JustinaTasks::findSkeletonPerson(JustinaTasks::NONE, tokens[4]);
            }
    }

	if (!fp){
		ss.str("");
		ss << "I can not find the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(), 5000);
		//JustinaNavigation::moveDistAngle(0, 1.57 ,10000);
	}
	//responseMsg.successful = 1;
	responseMsg.successful = fp;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackScanPerson(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command scan person ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

    JustinaHRI::waitAfterSay("Please look at me, I try to make a description of you", 5000);
	vision_msgs::VisionFaceObjects recognizedFaces;
    
    //JustinaVision::startFaceRecognition();
    
    do{
    recognizedFaces = JustinaVision::getFaces();  
    }while(recognizedFaces.recog_faces.size() < 1);

    std_msgs::String res1;
    std::stringstream ss1;
    ss1.str("");

	//for(int i=0; i<recognizedFaces.recog_faces.size(); i++)
	//{
		if(recognizedFaces.recog_faces[0].gender==0){
			JustinaHRI::waitAfterSay("I realize you are a woman", 5000);
            ss1 << "(assert (cmd_person_description woman ";
        }
			
		if(recognizedFaces.recog_faces[0].gender==1){
			JustinaHRI::waitAfterSay("I realize you are a man", 5000);
            ss1 << "(assert (cmd_person_description man ";
        }
	//}
    
    std::cout << "X: " << recognizedFaces.recog_faces[0].face_centroid.x 
              << " Y: " << recognizedFaces.recog_faces[0].face_centroid.y
              << " Z: " << recognizedFaces.recog_faces[0].face_centroid.z << std::endl;


    if(recognizedFaces.recog_faces[0].face_centroid.z > 1.7){
        JustinaHRI::waitAfterSay("I would say that you are tall and a young person", 5000);
        ss1 << "tall young ";
    }
    else{
        JustinaHRI::waitAfterSay("I would say that you are small and a young person", 5000);
        ss1 << "small young ";
    }

    JustinaHRI::waitAfterSay("And i think your complexion is slim", 5000);

    ss1 << "slim 1))";
    
    JustinaVision::startFaceRecognition(false);
    
    res1.data = ss1.str();
    sendAndRunClips_pub.publish(res1);
    
	responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackFindRemindedPerson(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
    std::cout << testPrompt << "-------- Command find reminded person -------"
        << std::endl;
    std::cout << "name: " << msg->name << std::endl;
    std::cout << "params: " << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;
    
    std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;

    int person_name = 0;
    float timeOut = 15000.0;
    std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;

    boost::posix_time::ptime curr;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    JustinaVision::startFaceRecognition(true);
    do {
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);

            ///El robot se mueve a una nueva posicion
            //JustinaNavigation::moveLateral(-0.3, 4000);
            JustinaManip::hdGoTo(0.0, 0, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            //JustinaManip::hdGoTo(0, -0.4, 5000);

            for (int i = 0; i < lastRecognizedFaces.size(); i++) {
                        if (lastRecognizedFaces[i].id == tokens[0]) {
                            person_name++;
                        } 
                    }

                    curr = boost::posix_time::second_clock::local_time();
                    ros::spinOnce();
        }while (ros::ok() && (curr - prev).total_milliseconds() < timeOut);
   
    std::cout << tokens[0] << " times: " << person_name << std::endl;

    responseMsg.successful = 0;
    if(person_name > 4){
        responseMsg.successful = 1;
        ss.str("");
        ss << "Hello " << tokens[0] << ", i find you";
        JustinaHRI::waitAfterSay(ss.str(), 6000);
    }

    command_response_pub.publish(responseMsg);
}

void callbackRemindPerson(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command remind person ---------"
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

    JustinaVision::startFaceRecognition(true);

    std_msgs::String person_name;
    person_name.data = tokens[0];

    for(int i=0; i < 15; i++){
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        //JustinaVision::facRecognize();
        train_face_pub.publish(person_name);
    }


    JustinaVision::startFaceRecognition(true);
	responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackCmdGetBag(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command get luggage ---------"
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

    ss.str("");
    ss << "I will help you to carry your " << tokens[0]; 
    JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
    ss.str("");
    ss << "please put the " << tokens[0] << " in my gripper";
    JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
            
    JustinaManip::raGoTo("navigation", 3000);
    JustinaTasks::detectObjectInGripper(tokens[0], false, 7000);
    JustinaHRI::waitAfterSay("thank you", 5000, 0);

	responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

void callbackCmdOfferDrink(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command offer drink to person ---------"
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
    std::string lastReco;
    std::string lastInt;

    bool drink_conf = false;
    bool name_conf = false;
    std::string drink;
    std::string name;
	std::string query;
    int count  = 0;
    
    bool success = ros::service::waitForService("spg_say", 5000);
    //success = success & ros::service::waitForService("/planning_clips/confirmation", 5000);

    while(!drink_conf && count < 3){
        switchSpeechReco(1, "tell me what drink do you want");
        JustinaHRI::waitForSpeechRecognized(lastReco,400);
        if(JustinaHRI::waitForSpeechRecognized(lastReco,10000)){
            if(JustinaRepresentation::stringInterpretation(lastReco, drink))
                std::cout << "last int: " << drink << std::endl;
                ss.str("");
                ss << "Do you want " << drink;
                count++;

		if (!alternative_drink && (no_drink == drink || prev_drink == drink)){
			count--;
			continue;
		}
                
                switchSpeechReco(0, ss.str());
                JustinaHRI::waitForSpeechRecognized(lastReco,400);

                JustinaHRI::waitForSpeechRecognized(lastReco,10000);
                if(lastReco == "robot yes" || lastReco == "justina yes"){
                    drink_conf = true;
			if (alternative_drink){
			    ss.str("");
                ss << "(assert (status-object-on-location " << drink << " 1))";    
				JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);
				if (query != "on_the_bar"){
					ss.str("");
					ss << "I am sorry, the " << drink << " is out of stock , please choose another one";
					no_drink = drink;
					drink_conf = false;
					JustinaHRI::waitAfterSay(ss.str(), 2000);
					alternative_drink = false;
					count--;
				}
			}
		}
                //count++;
        }
    }
        count = 0;
        drink_conf = false;
        ss.str("");
	prev_drink = drink;
        ss << "Ok a " << drink << " for you";
        JustinaHRI::waitAfterSay(ss.str(),5000);
    
    while(!drink_conf && count < 3){
        switchSpeechReco(2, "tell me what is your name please");
        JustinaHRI::waitForSpeechRecognized(lastReco,400);
        if(JustinaHRI::waitForSpeechRecognized(lastReco,10000)){
            if(JustinaRepresentation::stringInterpretation(lastReco, name))
                std::cout << "last int: " << name << std::endl;
                ss.str("");
                ss << "is " << name << " your name";
                switchSpeechReco(0, ss.str());
                JustinaHRI::waitForSpeechRecognized(lastReco,400);
                
                JustinaHRI::waitForSpeechRecognized(lastReco,10000);
                if(lastReco == "robot yes" || lastReco == "justina yes")
                    drink_conf = true;
                count++;
        }
    }
        
        ss.str("");
        ss << "Ok " << name << " I will bring you a " << drink << " from the bar";
        JustinaHRI::waitAfterSay(ss.str(),5000);

    ss.str("");
    ss << "(assert (give-drink-to-person " << drink << " " << name << "))";
    JustinaRepresentation::sendAndRunCLIPS(ss.str());
    JustinaHRI::enableSpeechRecognized(false);
    
    JustinaHRI::enableSpeechRecognized(true);

	responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackCmdTrainPerson(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command  Train person ---------"
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
    bool finish_train = false;
    int count = 0;
    
	if (poseRecog == JustinaTasks::SITTING || poseRecog == JustinaTasks::LYING)
        JustinaManip::hdGoTo(0.0, -0.6, 5000);
    else
        JustinaManip::hdGoTo(0.0, 0.0, 5000);

    JustinaHRI::waitAfterSay("guest please not move, and look at me", 6000);
    JustinaVision::faceTrain(tokens[0], 4);
    
    while(!finish_train && count < 4){
        if(JustinaVision::waitForTrainingFace(TIMEOUT_MEMORIZING)){
                finish_train = true;
        }
        count++;
    }
    JustinaManip::hdGoTo(0.0, 0.0, 5000);
    JustinaHRI::waitAfterSay("thank you", 6000);
	
    //JustinaNavigation::moveDistAngle(0, 1.57, 10000);
	//boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
	
    responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}


void callbackCmdGetOrderObject(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command get objects from the guest orders ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
    	std::vector<std::string> tokens;
    	std::vector<std::string> tokens1;
	std::string str = responseMsg.params;
	split(tokens1, str, is_any_of(" "));
	boost::replace_all(tokens1[0], "!", " ");
	split(tokens, tokens1[0], is_any_of(" "));
	std::stringstream ss;
    int attemps = 0;
	bool success = false;
    bool la = false;
    bool ra = false;

    while(!success && attemps<4){
        JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
        success = JustinaTasks::sayAndSyncNavigateToLoc(tokens1[tokens1.size() - 1], 120000);
        attemps++;
    }

   for (int i = 1; i < tokens.size()-1; i++){
        ss.str("");
        ss << "Barman I need a " << tokens[i]; 
        JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
        ss.str("");
        ss << "please put the " << tokens[i] << " in my gripper";
        JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
        if(!ra){

            JustinaManip::raGoTo("navigation", 3000);
            JustinaTasks::detectObjectInGripper(tokens[i], false, 7000);
            ra = true;
            ss.str("");
            ss << "(assert (set_object_arm " << tokens[i] << " false))";
            JustinaRepresentation::sendAndRunCLIPS(ss.str());
            JustinaHRI::waitAfterSay("thank you barman", 5000, 0);
        }
        if(!la && tokens.size() > 3){
            JustinaManip::laGoTo("navigation", 3000);
            JustinaTasks::detectObjectInGripper(tokens[i], false, 7000);
            la = true;
            ss << "(assert (set_object_arm " << tokens[i] << " true))";
            JustinaRepresentation::sendAndRunCLIPS(ss.str());
        }

   } 
	
    responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackCmdDeliverOrder(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command deliver object ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
    std::vector<std::string> tokens;
    std::vector<std::string> tokens1;
	std::string str = responseMsg.params;
	split(tokens1, str, is_any_of(" "));
	boost::replace_all(tokens1[0], "!", " ");
	split(tokens, tokens1[0], is_any_of(" "));
	std::stringstream ss;
    
    std::string name;
    std::string arm;

    int attemps = 0;
	bool success = false;
    bool armFlag = false;
    
    while(!success && attemps<4){
        JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
        success = JustinaTasks::sayAndSyncNavigateToLoc(tokens1[tokens1.size()-1], 120000);
        attemps++;
    }
    success = false;
   for (int i = 1; i < tokens.size() -1 ; i++){
       ss.str("");
       ss << "(assert (get_person " << tokens[i] << "))";
       JustinaRepresentation::strQueryKDB(ss.str(), name, 1000);
       if(name != "None"){
            ss.str("");
            ss << name << " please look at me, I try to find you";
            JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
            while(!success && attemps <4){
    			success = JustinaTasks::findPerson(name, -1, JustinaTasks::NONE, true, tokens[tokens.size()-1]);
                attemps++;
            }
            if(success){
                ss.str("");
                ss << "(assert (get_arm " << tokens[i] << "))" ;
                JustinaRepresentation::strQueryKDB(ss.str(), arm, 1000);
                armFlag = (arm == "true") ? true : false;
		        success = JustinaTasks::dropObject(tokens[i], armFlag, 30000);
                ss.str("");
                ss << name << " enjoy the " << tokens[i];
                JustinaHRI::waitAfterSay(ss.str(), 4000, 0);
            }
            else{
                ss.str("");
                ss << "I am sorry, I cant find you " << name;
                JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
            }
       }
   } 
	
    responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackCmdIntroducePerson(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Introduce Person ---------"
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

    if(tokens[0] == "person"){
            std::cout << "Introduce to person" << std::endl;
            switchSpeechReco(0, "");
            JustinaTasks::introduceTwoPeople(tokens[1], "def_loc", tokens[2], tokens[3], false);
    }
    else if (tokens[0] == "people"){
            std::cout << "Introduce to people" << std::endl;
    }
	
    responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

void callbackCmdMakeQuestion(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Make Question ---------"
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
	
    responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

void callbackCmdCleanUp(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Clean Up ---------"
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

    int count = 0;
    int obj_count = 0;
    bool help = false;
    std::string obj_name;
    std::string lastReco;
    std::string query;
    std::string to_speech;
   
    ss.str("");
    ss << "For this task I need your help, Do you want to help me, say robot yes or robot no";
    while (!help && count < 3){
        switchSpeechReco(0, ss.str());
        JustinaHRI::waitForSpeechRecognized(lastReco,400);

        JustinaHRI::waitForSpeechRecognized(lastReco,10000);
        if(lastReco == "robot yes" || lastReco == "justina yes")
            help = true;
        count++;
    }
    count = 0;
    if(!help){
        JustinaHRI::waitAfterSay("I am sorry If you dont help me I can not do this task", 10000);
        responseMsg.successful = 0;
    }
    else{
        while(obj_count < 3){
        
            if(obj_count<1)
                JustinaHRI::waitAfterSay("Thank you, Please guide me to the first object", 10000);
            else
                JustinaHRI::waitAfterSay("Ready, Please guide me to the next object", 10000);
        
            //follow    
            switchSpeechReco(5, "");
            //JustinaHRI::loadGrammarSpeechRecognized("follow_confirmation.xml");
            JustinaTasks::followAPersonAndRecogStop("stop follow me");
            
            count = 0;
            help = false;
            
            //ask object name
            JustinaHRI::waitAfterSay("I can not recognize the object", 10000);
            while(!help && count < 3){
                JustinaHRI::waitAfterSay("Please listen, for known objects say for instance, this is the apple", 10000);
                JustinaHRI::waitAfterSay("for unknown objects say, this is an unknown object, Please tell me the object's name", 10000);
                switchSpeechReco(10, "");
                JustinaHRI::waitForSpeechRecognized(lastReco, 4000);
                if(JustinaHRI::waitForSpeechRecognized(lastReco, 10000)){
                    if(JustinaRepresentation::stringInterpretation(lastReco, obj_name))
                        std::cout << "last int: " << obj_name << std::endl;
                        ss.str("");
                        ss << "This object is the " << obj_name << ", say robot yes o robot no";
                        switchSpeechReco(0, ss.str());
                        JustinaHRI::waitForSpeechRecognized(lastReco,400);

                    JustinaHRI::waitForSpeechRecognized(lastReco,10000);
                    if(lastReco == "robot yes" || lastReco == "justina yes"){
                        help = true;

                    }
                    count++;
                }
            }
            to_speech = obj_name;
		    boost::replace_all(to_speech, "_", " ");

            //grasp object
            ss.str("");
            ss << "I can not grasp the " << to_speech;
            JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
            ss.str("");
            ss << "please put the " << to_speech << " in my gripper";
            JustinaHRI::waitAfterSay(ss.str(), 5000, 0);

            JustinaManip::raGoTo("navigation", 3000);
            JustinaTasks::detectObjectInGripper(obj_name, false, 7000);
            JustinaHRI::waitAfterSay("thank you", 5000, 0);
            
            //count = 0;
            //help = false;
            //ss.str("");
            //ss << "(assert (set_object_arm " << obj_name << " false))";
            //JustinaRepresentation::sendAndRunCLIPS(ss.str());
            JustinaHRI::waitAfterSay("thank you", 5000, 0);
            to_speech = tokens[0];
		    boost::replace_all(to_speech, "_", " ");
            //ss.str("");
            //ss << "please wait for me in the center of the " << to_speech << ", I will need your help again";
            //JustinaHRI::waitAfterSay(ss.str(), 1000);
            if(obj_name == "unknown_object"){
                //task for put garbage into the bin
                JustinaHRI::waitAfterSay("I still can not put the unknown object into the bin", 2000);
                JustinaHRI::waitAfterSay("Please help me", 2000);
			    JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
			    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
			    JustinaTasks::guideAPerson("bed", 120000); //cambiar para guiar a un bin
			    
                
                JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
			    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                JustinaTasks::dropObject("unknown_object", false, 30000);

                JustinaHRI::waitAfterSay("Please put the garbage into the bin", 5000);
               
                if(obj_count < 2)
                    JustinaHRI::waitAfterSay("When you are ready we can continue", 5000);
			    
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

            }
            else{
                // task for put objects in default location
                ss.str("");
                ss << "please wait for me in the center of the " << to_speech << ", I will need your help again";
                JustinaHRI::waitAfterSay(ss.str(), 1000);
                
                ss.str("");
                ss << "(assert (get_obj_default_loc " << obj_name << " 1))";
                JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);

                JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
                JustinaTasks::sayAndSyncNavigateToLoc(query, 120000);
                ss.str("");
                ss << "I am going to deliver the " << obj_name;
                JustinaHRI::waitAfterSay(ss.str(), 2000);
                JustinaTasks::placeObject(false); // right arm
                JustinaManip::raGoTo("home", 6000);

                if(count < 2)
                    JustinaTasks::sayAndSyncNavigateToLoc(tokens[0], 120000);
                
            }
            obj_count++;
        }// end while
        responseMsg.successful = 1;
    }// end else
	
    JustinaHRI::waitAfterSay("I finish the task, thanks for your help", 5000);
    //responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

void callbackCmdTakeOutGarbage(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Take out the garbage ---------"
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

    bool help = false;
    int count = 0;
    int bag_count = 0;
    std::string lastReco;
    std::vector<std::string> bins;

    //change for the real bin locations
    bins.push_back("bed");
    bins.push_back("bed");
    
    ss.str("");
    ss << "For this task I need your help, Do you want to help me, say robot yes or robot no";
    while (!help && count < 3){
        switchSpeechReco(0, ss.str());
        JustinaHRI::waitForSpeechRecognized(lastReco,400);

        JustinaHRI::waitForSpeechRecognized(lastReco,10000);
        if(lastReco == "robot yes" || lastReco == "justina yes")
            help = true;
        count++;
    }
    count = 0;
    if(!help){
        JustinaHRI::waitAfterSay("I am sorry if you dont help me I can not do this task", 10000);
        responseMsg.successful = 0;
    }
    else{

        while(bag_count < 2){
            if (bag_count == 0)
                JustinaHRI::waitAfterSay("Thank you, go for the first bag", 10000);
            else 
                JustinaHRI::waitAfterSay("Go for the next bag", 10000);
            
            //guide to the bin
            JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaTasks::guideAPerson(bins.at(bag_count), 120000); //cambiar para guiar a un bin

            JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                
            //grasp object
            ss.str("");
            ss << "I can not grasp the bag into the bin";
            JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
            ss.str("");
            ss << "please put the bag in my gripper";
            JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
            if(bag_count == 0){
                JustinaManip::raGoTo("navigation", 3000);
                JustinaTasks::detectObjectInGripper("bag", false, 7000);
            }
            else{
                JustinaManip::laGoTo("navigation", 3000);
                JustinaTasks::detectObjectInGripper("bag", true, 7000);
            }

            bag_count++;
        }//end while
        
        JustinaHRI::waitAfterSay("thanks for you help, Now i will take out the garbage", 5000, 0);

        //put bags into collection zone
        JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
        JustinaTasks::sayAndSyncNavigateToLoc("exit", 120000); //change for real collection zone
        
        JustinaManip::laGoTo("place_bag_floor", 4000);
        JustinaManip::startLaOpenGripper(0.7);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        JustinaManip::laGoTo("home", 4000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        JustinaManip::startLaOpenGripper(0);
        
        JustinaManip::raGoTo("place_bag_floor", 4000);
        JustinaManip::startRaOpenGripper(0.7);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        JustinaManip::raGoTo("home", 4000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        JustinaManip::startRaOpenGripper(0);

        
    }//end else
    
    JustinaHRI::waitAfterSay("I finish the task", 5000, 0);

    
    responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

void callbackCmdGuideToTaxi(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Guide to taxi ---------"
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

	std::string lastReco;
	std::vector<std::string> idsUmbrella;
	idsUmbrella.push_back("umbrella");
	
	bool leave_conf = false;
	bool guide_to_taxi = true;
	bool findUmbrella = false;
	int count = 0;

	if(tokens[1] != "ask_for_leave")
		leave_conf = true;
    
	while(!leave_conf && count < 3){
		ss.str("");
		ss << "Do you want to go, tell me robot yes o robot no";
		switchSpeechReco(0, ss.str());
		JustinaHRI::waitForSpeechRecognized(lastReco,400);
		
		JustinaHRI::waitForSpeechRecognized(lastReco,10000);
		if(lastReco == "robot yes" || lastReco == "justina yes")
		    leave_conf = true;
		else
			JustinaHRI::waitAfterSay("Sorry I did not understand you", 10000);
		count++;
	}

	if (!leave_conf){
		guide_to_taxi = false;
		responseMsg.successful = 0;
                JustinaHRI::say("Ok I understand, please enjoy the party");
        	ros::Duration(1.0).sleep();
	}
	else{
		
		JustinaHRI::say("I am going to guide you to the coat rack");
		ros::Duration(1.0).sleep();
		JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);
		ros::Duration(1.0).sleep();
		JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	    
		JustinaTasks::guideAPerson("entrance", 300000, 1.5);
		
		JustinaHRI::say("It is rainning outside and I think we will need an umbrella");
		ros::Duration(1.0).sleep();
		JustinaHRI::say("Please human take the umbrella, it is close to the coat rack");
		ros::Duration(1.0).sleep();
		JustinaHRI::say("hey guest, do not forget to take your coat");
		ros::Duration(2.0).sleep();

		JustinaHRI::say("ready, now i will take you outside to guide you to the taxi");
		ros::Duration(1.0).sleep();

		JustinaNavigation::moveDistAngle(0.0, 3.14159, 2000);/// revizar este giro, creo que hay que quitarlo
		ros::Duration(1.0).sleep();
		JustinaHRI::say("Do not forget use the umbrella to protect us");
		ros::Duration(1.0).sleep();
		JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

		JustinaTasks::guideAPerson("exitdoor", 300000, 1.5);

		JustinaHRI::say("wait here with me I am looking for the taxi driver");
		ros::Duration(1.0).sleep();
                
		count = 0;
		while (!findUmbrella && count < 3){
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			findUmbrella = JustinaTasks::findAndGuideYolo(idsUmbrella);
			count++;
		}

		if(findUmbrella){
			JustinaManip::hdGoTo(0.0, 0.0, 1000);
			JustinaHRI::waitAfterSay("Hello Taxi driver, my name is Justina, I came here with a guest that want to go home", 12000);
			ros::Duration(1.0).sleep();
			/*JustinaHRI::say("Hey guest i hope you have a nice trip, could you please lend me the umbrella");
			ros::Duration(1.5).sleep();
			JustinaHRI::say("please close the umbrella and put in my gripper");
			ros::Duration(1.5).sleep();
			JustinaTasks::detectObjectInGripper("umbrella", true, 7000);
			ros::Duration(1.0).sleep();*/
			ss.str("");
			ss << "Hey guest i hope you have a nice trip, Good bye ";
			JustinaHRI::say(ss.str());
			ros::Duration(1.0).sleep();
			JustinaHRI::say("hey taxi driver, please drive carefully, good bye");
			ros::Duration(1.5).sleep();
			responseMsg.successful = 1;
			
		}
		else{
			ss.str("");
			ss << "I am sorry i cand find the taxi driver";
			JustinaHRI::say(ss.str());
			responseMsg.successful = 0;
		}
		
	}

	
	//responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

void callbackAskInc(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Ask for incomplete information ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
    	
	std::vector<std::string> tokens;
    	std::vector<std::string> tokens1;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;
    	std::stringstream ss1;
	std::stringstream ss2;
    	std::string lastRecoSpeech;
    	int timeoutspeech = 10000;
    	bool conf = false;
    	int intentos = 0;
	
	ss.str("");
	if(tokens[1] == "origin")
		ss << "I am sorry, I dont know where is the " << tokens[0];
	else
		ss << "I am sorry, I dont know the destiny location to guide the " << tokens[0];
	
	
	JustinaHRI::waitAfterSay(ss.str(), 5000);

    	while(intentos < 5 && !conf){
        ss.str("");
	if(tokens[1] == "origin")
	        ss << "Please tell me where can i find the " << tokens[0];
	else
		ss << "Please tell me what is the destiny location";
        JustinaHRI::loadGrammarSpeechRecognized("incomplete_place.xml");

        JustinaHRI::waitAfterSay(ss.str(), 5000);

        if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeoutspeech)){
            split(tokens1, lastRecoSpeech, is_any_of(" "));
            ss1.str("");
	    ss2.str("");
            if(tokens1.size() == 3){
		if(tokens[1] == "origin")
               		ss1 << "is the " << tokens[0] << " in the " << tokens1[2];
		else
			ss1 << "is the " << tokens1[2] << " the destiny location";

			ss2 << tokens1[2];
		}
		else if(tokens1.size() == 4){
			if(tokens[1] == "origin")
               			ss1 << "is the " << tokens[0] << " in the " << tokens1[2] << " " << tokens1[3];
			else
				ss1 << "is the " << tokens1[2] << " " << tokens1[3] << " the destiny location";
			ss2 << tokens1[2] << "_" << tokens1[3];
			
		}

            JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
            JustinaHRI::waitAfterSay(ss1.str(), 5000);    
            
            knowledge_msgs::planning_cmd srv;
            srv.request.name = "test_confirmation";
            srv.request.params = responseMsg.params;
            if (srvCltWaitConfirmation.call(srv)) {
                std::cout << "Response of confirmation:" << std::endl;
                std::cout << "Success:" << (long int) srv.response.success
                    << std::endl;
                std::cout << "Args:" << srv.response.args << std::endl;
                //responseMsg.params = "conf";
                //responseMsg.successful = srv.response.success;
            } else {
                std::cout << testPrompt << "Failed to call service of confirmation"
                    << std::endl;
                //responseMsg.successful = 0;
                JustinaHRI::waitAfterSay("Sorry i did not understand you", 1000);
            }
            
            if( (long int) srv.response.success == 1 ){
		ss.str("");
		ss << "Ok i will look for the " << tokens[0] << " in the " << tokens1[2] << ", thank you";
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                std_msgs::String res1;
                ss1.str("");
		ss1 << tokens[0] << " " << tokens[1] << " " << ss2.str();
		responseMsg.params = ss1.str();
                conf = true;
            }
            else{
                intentos++;
                JustinaHRI::waitAfterSay("Sorry I did not understand you", 5000);
            }

       }
    }
	responseMsg.successful = 1;
	if(!conf){
		ss1.str("");
		responseMsg.successful = 0;
		ss1 << tokens[0] << " " <<tokens[1] << " kitchen";
		responseMsg.params = ss1.str();
	}
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackGetPersonDescription(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Ask for the person description ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
    	std::vector<std::string> tokens1;
	std::string gesture;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;
    	std::stringstream ss1;
	std::stringstream ss2;
    	std::string lastRecoSpeech;
    	int timeoutspeech = 10000;
    	bool conf = false;
    	int intentos = 0;
	
	ss.str("");
	ss << "Please introduce " << tokens[0] << " to me, let me ask you some questions";
	JustinaHRI::waitAfterSay(ss.str(), 5000);

    ss1.str("");

    ss1 << tokens[0] << " " << tokens[1];
    	
	while(intentos < 5 && !conf){
        ss.str("");
        ss << "Please tell me if " << tokens[0] << " is making a waving, pointing, or raising his arm";
        JustinaHRI::loadGrammarSpeechRecognized("description_gesture.xml");

        JustinaHRI::waitAfterSay(ss.str(), 5000);

        if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeoutspeech)){
            split(tokens1, lastRecoSpeech, is_any_of(" "));
            ss.str("");
	    ss2.str("");
            if(tokens1.size() == 3){
               ss << "is " << tokens[0] << " " << tokens1[2];
		gesture = tokens1[2];
		ss2 << "waving";
	}
	    else if(tokens1.size() == 6 && tokens1[2] == "pointing"){
		ss << "pointing_to_the_" << tokens1[5];
		gesture = ss.str();
		ss.str("");
		ss << "is " << tokens[0] << " pointing to the " << tokens1[5];
		ss2 << "pointing to the " << tokens1[5];
		}
	    else if(tokens1.size() == 6 && tokens1[2] == "raising"){
		ss << "raising_their_" << tokens1[4] << "_arm";
		gesture = ss.str();
		ss.str("");
		ss << "is " << tokens[0] << " raising their " << tokens1[4] << " arm";
		ss2 << "raising their " << tokens1[4] << " arm";
		}
            else
                continue;

            JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
            JustinaHRI::waitAfterSay(ss.str(), 5000);
            
            knowledge_msgs::planning_cmd srv;
            srv.request.name = "test_confirmation";
            srv.request.params = responseMsg.params;
            if (srvCltWaitConfirmation.call(srv)) {
                std::cout << "Response of confirmation:" << std::endl;
                std::cout << "Success:" << (long int) srv.response.success
                    << std::endl;
                std::cout << "Args:" << srv.response.args << std::endl;
                //responseMsg.params = "conf";
                //responseMsg.successful = srv.response.success;
            } else {
                std::cout << testPrompt << "Failed to call service of confirmation"
                    << std::endl;
                //responseMsg.successful = 0;
                JustinaHRI::waitAfterSay("Sorry i did not understand you", 1000);
            }
            
            if( (long int) srv.response.success == 1 ){
		ss.str("");
		ss << "Ok, " << tokens[0] << " is " << ss2.str();
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                std_msgs::String res1;
                //ss1.str("");
		ss1 << " " << gesture;
		responseMsg.params = ss1.str();
                conf = true;
            }
            else{
                intentos++;
                JustinaHRI::waitAfterSay("Sorry I did not understand you", 5000);
            }

       }
    }
	conf = false;
	intentos = 0;
    	
	while(intentos < 5 && !conf){
        ss.str("");
        ss << "Please tell me if " << tokens[0] << " is lying down, sitting or standing";
        JustinaHRI::loadGrammarSpeechRecognized("description_pose.xml");

        JustinaHRI::waitAfterSay(ss.str(), 5000);

        if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeoutspeech)){
            split(tokens1, lastRecoSpeech, is_any_of(" "));
            ss.str("");
            if(tokens1.size() == 3)
               ss << "is " << tokens[0] << " " << tokens1[2];
            else
                continue;

            JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
            JustinaHRI::waitAfterSay(ss.str(), 5000);    
            
            knowledge_msgs::planning_cmd srv;
            srv.request.name = "test_confirmation";
            srv.request.params = responseMsg.params;
            if (srvCltWaitConfirmation.call(srv)) {
                std::cout << "Response of confirmation:" << std::endl;
                std::cout << "Success:" << (long int) srv.response.success
                    << std::endl;
                std::cout << "Args:" << srv.response.args << std::endl;
                //responseMsg.params = "conf";
                //responseMsg.successful = srv.response.success;
            } else {
                std::cout << testPrompt << "Failed to call service of confirmation"
                    << std::endl;
                //responseMsg.successful = 0;
                JustinaHRI::waitAfterSay("Sorry i did not understand you", 1000);
            }
            
            if( (long int) srv.response.success == 1 ){
		ss.str("");
		ss << "Ok, " << tokens[0] << " is " << tokens1[2];
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                std_msgs::String res1;
                //ss1.str("");
		ss1 << " " << tokens1[2];
		responseMsg.params = ss1.str();
                conf = true;
            }
            else{
                intentos++;
                JustinaHRI::waitAfterSay("Sorry I did not understand you", 5000);
            }

       }
    }

	conf = false;
	intentos = 0;

    	while(intentos < 5 && !conf){
        ss.str("");
        ss << "Please tell me if " << tokens[0] << " is tall or short";
        JustinaHRI::loadGrammarSpeechRecognized("description_hight.xml");

        JustinaHRI::waitAfterSay(ss.str(), 5000);

        if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeoutspeech)){
            split(tokens1, lastRecoSpeech, is_any_of(" "));
            ss.str("");
            if(tokens1.size() == 3)
               ss << "is " << tokens[0] << " " << tokens1[2];
            else
                continue;

            JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
            JustinaHRI::waitAfterSay(ss.str(), 5000);    
            
            knowledge_msgs::planning_cmd srv;
            srv.request.name = "test_confirmation";
            srv.request.params = responseMsg.params;
            if (srvCltWaitConfirmation.call(srv)) {
                std::cout << "Response of confirmation:" << std::endl;
                std::cout << "Success:" << (long int) srv.response.success
                    << std::endl;
                std::cout << "Args:" << srv.response.args << std::endl;
                //responseMsg.params = "conf";
                //responseMsg.successful = srv.response.success;
            } else {
                std::cout << testPrompt << "Failed to call service of confirmation"
                    << std::endl;
                //responseMsg.successful = 0;
                JustinaHRI::waitAfterSay("Sorry i did not understand you", 1000);
            }
            
            if( (long int) srv.response.success == 1 ){
		ss.str("");
		ss << "Ok, " << tokens[0] << " is " << tokens1[2];
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                std_msgs::String res1;
                //ss1.str("");
		ss1 << " " << tokens1[2];
		responseMsg.params = ss1.str();
                conf = true;
            }
            else{
                intentos++;
                JustinaHRI::waitAfterSay("Sorry I did not understand you", 5000);
            }

       }
    }

	conf = true;
	intentos = 0;

    	while(intentos < 5 && !conf){
        ss.str("");
        ss << "Please tell me if " << tokens[0] << " is a man or a woman";
        JustinaHRI::loadGrammarSpeechRecognized("description_gender.xml");

        JustinaHRI::waitAfterSay(ss.str(), 5000);

        if(JustinaHRI::waitForSpeechRecognized(lastRecoSpeech, timeoutspeech)){
            split(tokens1, lastRecoSpeech, is_any_of(" "));
            ss.str("");
            if(tokens1.size() == 3)
               ss << "is " << tokens[0] << " a " << tokens1[2];
            else
                continue;

            JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
            JustinaHRI::waitAfterSay(ss.str(), 5000);    
            
            knowledge_msgs::planning_cmd srv;
            srv.request.name = "test_confirmation";
            srv.request.params = responseMsg.params;
            if (srvCltWaitConfirmation.call(srv)) {
                std::cout << "Response of confirmation:" << std::endl;
                std::cout << "Success:" << (long int) srv.response.success
                    << std::endl;
                std::cout << "Args:" << srv.response.args << std::endl;
                //responseMsg.params = "conf";
                //responseMsg.successful = srv.response.success;
            } else {
                std::cout << testPrompt << "Failed to call service of confirmation"
                    << std::endl;
                //responseMsg.successful = 0;
                JustinaHRI::waitAfterSay("Sorry i did not understand you", 1000);
            }
            
            if( (long int) srv.response.success == 1 ){
		ss.str("");
		ss << "Ok, " << tokens[0] << " is a "  << tokens1[2];
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                std_msgs::String res1;
                //ss1.str("");
		ss1 << " " << tokens1[2];
		responseMsg.params = ss1.str();
                conf = true;
            }
            else{
                intentos++;
                JustinaHRI::waitAfterSay("Sorry I did not understand you", 5000);
            }

       }
    }

	responseMsg.params = ss1.str();
	responseMsg.successful = 1;
    //JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackCmdRPoseObj(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Make RPose object ---------"
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

    std::string lastReco;
    std::string obj_name = "coke";
    bool help = false;
    int count = 0;

    JustinaHRI::waitAfterSay("Thank you, Please come with me", 10000);
            
    //guide to the rpose object
    JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    JustinaTasks::guideAPerson(tokens[1], 120000); //cambiar para guiar a un bin

    JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        
    //grasp object
    ss.str("");
    ss << "I can not grasp the object at the " << tokens[1] << "'s " << tokens[2] ;
    JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
    ss.str("");
    ss << "please put the object in my gripper";
    JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
    JustinaManip::laGoTo("navigation", 3000);
    JustinaTasks::detectObjectInGripper("object", true, 7000);
            
    JustinaHRI::waitAfterSay("I can not recognize the object", 10000);
    while(!help && count < 3){
        JustinaHRI::waitAfterSay("Please tell me the object's name", 10000);
        switchSpeechReco(10, "say for instance, this is the apple");
        JustinaHRI::waitForSpeechRecognized(lastReco, 4000);
        if(JustinaHRI::waitForSpeechRecognized(lastReco, 10000)){
            if(JustinaRepresentation::stringInterpretation(lastReco, obj_name))
                std::cout << "last int: " << obj_name << std::endl;
                ss.str("");
                ss << "This object is the " << obj_name << ", say justina yes o justina no";
                switchSpeechReco(0, ss.str());
                JustinaHRI::waitForSpeechRecognized(lastReco,400);

            JustinaHRI::waitForSpeechRecognized(lastReco,10000);
            if(lastReco == "robot yes" || lastReco == "justina yes"){
                help = true;
            }
            count++;
        }
    }

    ss.str("");
    ss << "Ok this is the " << obj_name << ", please wait me in the initial point";

    JustinaHRI::waitAfterSay(ss.str(), 10000);

    ss.str("");
    ss << obj_name << " 2 2 2 left";

	responseMsg.params = ss.str();
    responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

void callbackCmdPourinObj(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Make pouring object ---------"
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
    
    int count = 0;
    int obj_count = 0;
    bool help = false;
	bool armFlag;
    std::string obj_name;
    std::string lastReco;
    std::string query;
    std::string to_speech;
   
    ss.str("");
    ss << "For this task I need your help, Do you want to help me, say robot yes or robot no";
    while (!help && count < 3){
        switchSpeechReco(0, ss.str());
        JustinaHRI::waitForSpeechRecognized(lastReco,400);

        JustinaHRI::waitForSpeechRecognized(lastReco,10000);
        if(lastReco == "robot yes" || lastReco == "justina yes")
            help = true;
        count++;
    }
    count = 0;
    if(!help){
        JustinaHRI::waitAfterSay("I am sorry If you dont help me I can not do this task", 100000);
        responseMsg.successful = 0;
    }
    else{
        //ask object name
        //JustinaHRI::waitAfterSay("I can not recognize the object", 10000);
        help =false;
        while(!help && count < 3){
            ss.str("");
            ss << "Tell me what you want I pourin in the " << tokens[0];
            switchSpeechReco(11, ss.str());
            JustinaHRI::waitForSpeechRecognized(lastReco, 4000);
            if(JustinaHRI::waitForSpeechRecognized(lastReco, 10000)){
                if(JustinaRepresentation::stringInterpretation(lastReco, obj_name))
                    std::cout << "last int: " << obj_name << std::endl;
                    ss.str("");
                    ss << "You want I pourin the " << obj_name << " in the " << tokens[0];
                    JustinaHRI::waitAfterSay(ss.str(), 10000);
                    switchSpeechReco(0, "say justina yes or justina no");
                    JustinaHRI::waitForSpeechRecognized(lastReco,400);

                JustinaHRI::waitForSpeechRecognized(lastReco,10000);
                if(lastReco == "robot yes" || lastReco == "justina yes"){
                    help = true;
                }
                count++;
            }
        }
        if(help){
            to_speech = obj_name;
            boost::replace_all(to_speech, "_", " ");
            ss.str("");
            ss << "Ok, go for the  " << to_speech;
            JustinaHRI::waitAfterSay(ss.str(), 100000);
            
            //get default location
            ss.str("");
            ss << "(assert (get_obj_default_loc " << obj_name << " 1))";
            JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);

            //guide to the bin
            JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaTasks::guideAPerson(query, 120000); //guiar al objeto
            JustinaHRI::waitAfterSay("please wait", 2500);

            //JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
            //boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            
            //try to grasp object
			geometry_msgs::Pose pose;
			bool grasp = false;
            ss.str("");
            ss << "I am looking for the" << to_speech;
            JustinaHRI::waitAfterSay(ss.str(), 2500);
            JustinaManip::hdGoTo(0, -0.9, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaTasks::alignWithTable(0.42);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            
            JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 8000);
			grasp = JustinaTasks::findObject(obj_name, pose, armFlag);
            JustinaManip::torsoGoTo(0.1, 0.0, 0.0, 8000);

            if(grasp){
                JustinaManip::torsoGoTo(0.0, 0.0, 0.0, 8000);
                JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
                grasp = JustinaTasks::graspObject(pose.position.x, pose.position.y, pose.position.z, armFlag, obj_name, true);
                JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
            }
            if(!grasp){
                armFlag = false;
                //grasp object
                ss.str("");
                ss << "I can not grasp the " << to_speech;
                JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
                ss.str("");
                ss << "please put the " << to_speech << " in my gripper";
                JustinaHRI::waitAfterSay(ss.str(), 5000, 0);

                JustinaManip::raGoTo("navigation", 3000);
                JustinaTasks::detectObjectInGripper(obj_name, armFlag, 7000);
                JustinaHRI::waitAfterSay("thank you", 5000, 0);
            }
            
            ss.str("");
            ss << "Now go for the  " << tokens[0];
            JustinaHRI::waitAfterSay(ss.str(), 100000);
            
            //get default location
            ss.str("");
            ss << "(assert (get_obj_default_loc " << tokens[0] << " 1))";
            JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);

            //guide to the bin
            JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaTasks::guideAPerson(query, 120000); //guiar al objeto
            JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            
            ss.str("");
            ss << "Here we can take the " << tokens[0] << ", but I need your help";
            JustinaHRI::waitAfterSay(ss.str(), 100000);
            ss.str("");
            ss << "I can not pour the " << to_speech << " into the" << tokens[0];
            JustinaHRI::waitAfterSay(ss.str(), 100000);
            ss.str("");
            ss << "I will give you the " << to_speech;
            JustinaHRI::waitAfterSay(ss.str(), 100000);
			boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaTasks::dropObject(obj_name, false, 30000);
            ss.str("");
            ss << "Please pour the " << to_speech << " into the" << tokens[0];
            JustinaHRI::waitAfterSay(ss.str(), 100000);
            
            JustinaHRI::waitAfterSay("Enjoy", 100000);
            JustinaTasks::dropObject(obj_name, false, 30000);
            JustinaHRI::waitAfterSay("If you are ready we can continue", 100000);

            help = false;
            count  = 0;

            while(!help && count < 3){
                    JustinaHRI::waitAfterSay("are your ready", 10000);
                    switchSpeechReco(0, "say justina yes or justina no");
                    JustinaHRI::waitForSpeechRecognized(lastReco,400);

                     JustinaHRI::waitForSpeechRecognized(lastReco,10000);
                     if(lastReco == "robot yes" || lastReco == "justina yes")
                         help = true;
                     count++;
            }
            
            JustinaHRI::waitAfterSay("I will return to the initial point, Thanks for your help ", 100000);

        }
        else{
            JustinaHRI::waitAfterSay("I am sorry, I did not understand you", 100000);
        }


    }


    responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
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
	srvCltAskIncomplete = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/ask_incomplete");
	srvCltQueryKDB = n.serviceClient<knowledge_msgs::StrQueryKDB>("/planning_clips/str_query_KDB");
	srvEnableSphinx = n.serviceClient<knowledge_msgs::sphinxConf>("/pocketsphinx/enable_speech_reco");

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
	ros::Subscriber subPropObj = n.subscribe("/planning_clips/cmd_prop_obj", 1, callbackOpropObject);
	ros::Subscriber subGesturePerson = n.subscribe("/planning_clips/cmd_gesture_person", 1, callbackGesturePerson);
	ros::Subscriber subGPPerson = n.subscribe("/planning_clips/cmd_gender_pose_person", 1, callbackGPPerson);
	ros::Subscriber subGPCrowd = n.subscribe("/planning_clips/cmd_gender_pose_crowd", 1, callbackGPCrowd);
	ros::Subscriber subSpeechGenerator = n.subscribe("/planning_clips/cmd_speech_generator", 1, callbackCmdSpeechGenerator);
	ros::Subscriber subAskIncomplete = n.subscribe("/planning_clips/cmd_ask_incomplete", 1, callbackCmdAskIncomplete);
    ros::Subscriber subCmdTaskConfirmation = n.subscribe("/planning_clips/cmd_task_conf", 1, callbackCmdTaskConfirmation);
    ros::Subscriber subCmdGetBag = n.subscribe("/planning_clips/cmd_get_bag", 1, callbackCmdGetBag);
    ros::Subscriber subCmdFollowToTaxi = n.subscribe("/planning_clips/cmd_follow_to_taxi", 1, callbackCmdFollowToTaxi);
    ros::Subscriber subCmdOfferDrink = n.subscribe("/planning_clips/cmd_offer_drink", 1, callbackCmdOfferDrink);
    ros::Subscriber subCmdTrainPerson = n.subscribe("/planning_clips/cmd_train_person", 1, callbackCmdTrainPerson);
    ros::Subscriber subCmdGetOrderObject = n.subscribe("/planning_clips/cmd_get_order_object", 1, callbackCmdGetOrderObject);
    ros::Subscriber subCmdDeliverOrder = n.subscribe("/planning_clips/cmd_deliver_order", 1, callbackCmdDeliverOrder);
    ros::Subscriber subCmdIntroducePerson = n.subscribe("/planning_clips/introduce_person", 1, callbackCmdIntroducePerson);
    ros::Subscriber subCmdMakeQuestion = n.subscribe("/planning_clips/make_question", 1, callbackCmdMakeQuestion);
    ros::Subscriber subCmdGuideToTaxi = n.subscribe("/planning_clips/guide_to_taxi", 1, callbackCmdGuideToTaxi);
    ros::Subscriber subCmdCleanUp = n.subscribe("/planning_clips/clean_up", 1, callbackCmdCleanUp);
    ros::Subscriber subCmdTakeOutGarbage = n.subscribe("/planning_clips/take_out_garbage", 1, callbackCmdTakeOutGarbage);

    /// EEGPSR topícs category II Montreal
    ros::Subscriber subManyPeople = n.subscribe("/planning_clips/cmd_many_people", 1, callbackManyPeople);
    ros::Subscriber subAmountPeople = n.subscribe("/planning_clips/cmd_amount_people", 1, callbackAmountPeople);
    ros::Subscriber subAskAndOffer = n.subscribe("/planning_clips/cmd_ask_and_offer", 1, callbackAskAndOffer);
    ros::Subscriber subFindEPerson = n.subscribe("/planning_clips/cmd_find_e_person", 1, callbackFindEPerson);
    ros::Subscriber subScanPerson = n.subscribe("/planning_clips/cmd_scan_person", 1, callbackScanPerson);
    ros::Subscriber subRemindPerson = n.subscribe("/planning_clips/cmd_remind_person", 1, callbackRemindPerson);
    ros::Subscriber subFindRemindPerson = n.subscribe("/planning_clips/cmd_find_reminded_person", 1, callbackFindRemindedPerson);
    ros::Subscriber subAskInc = n.subscribe("/planning_clips/cmd_ask_inc", 1, callbackAskInc);
    ros::Subscriber subGetPersonDescription = n.subscribe("planning_clips/cmd_get_person_description", 1, callbackGetPersonDescription); 
    ros::Subscriber subRPoseObj = n.subscribe("planning_clips/rpose_obj", 1, callbackCmdRPoseObj); 
    ros::Subscriber subPourinObj = n.subscribe("planning_clips/pourin_obj", 1, callbackCmdPourinObj); 

	command_response_pub = n.advertise<knowledge_msgs::PlanningCmdClips>("/planning_clips/command_response", 1);
    sendAndRunClips_pub = n.advertise<std_msgs::String>("/planning_clips/command_sendAndRunCLIPS", 1);
    train_face_pub = n.advertise<std_msgs::String>("/vision/face_recognizer/run_face_trainer", 1);

    pubStartTime = n.advertise<std_msgs::Int32>("/planning/start_time", 1); 
    pubResetTime = n.advertise<std_msgs::Empty>("/planning/restart_time", 1);

	JustinaHRI::setNodeHandle(&n);
	JustinaHardware::setNodeHandle(&n);
	JustinaKnowledge::setNodeHandle(&n);
	JustinaManip::setNodeHandle(&n);
	JustinaNavigation::setNodeHandle(&n);
	JustinaTasks::setNodeHandle(&n);
	JustinaTools::setNodeHandle(&n);
	JustinaVision::setNodeHandle(&n);
	JustinaRepresentation::setNodeHandle(&n);
	
	JustinaRepresentation::initKDB("", false, 20000);
    JustinaRepresentation::initKDB("/gpsr_2019/gpsr.dat", false, 20000);
    	idsPerson.push_back("person");

	if (argc > 3){
		std::cout << "FPLAN FLAG: " << argv[3] << std::endl;
		fplan = atoi(argv[3]);
		maxTime = atof(argv[4]);
		cat_grammar = argv[5];
        poket_reco = atoi(argv[6]);
		std::cout << "FPLAN FLAG: " << fplan << std::endl;
		std::cout << "MAX TIME: " << maxTime << std::endl;
		std::cout << "Grammar: " << cat_grammar << std::endl;}
        
        JustinaHRI::usePocketSphinx = poket_reco;

        microsoft_grammars[0] = "confirmation.xml";
        microsoft_grammars[1] = "what_drink.xml";
        microsoft_grammars[2] = "name_response.xml";
        microsoft_grammars[3] = cat_grammar;
        microsoft_grammars[4] = "questions.xml";
        microsoft_grammars[5] = "follow_confirmation.xml";
        microsoft_grammars[6] = "follow_taxi.xml";
        microsoft_grammars[7] = "incomplete_place.xml";
        microsoft_grammars[8] = "incomplete_object.xml";
        microsoft_grammars[9] = "order_food.xml";
        microsoft_grammars[10] = "this_object.xml";
        microsoft_grammars[11] = "obj_pour.xml";
        /*microsoft_grammars[10] = "description_gesture.xml";
        microsoft_grammars[11] = "description_pose.xml";
        microsoft_grammars[12] = "description_hight.xml";
        microsoft_grammars[12] = "description_gender.xml";*/

        sphinx_grammars[0] = "confirmation";
        sphinx_grammars[1] = "order_drink";
        sphinx_grammars[2] = "people_names";
        sphinx_grammars[3] = "gpsr_grammar";
        sphinx_grammars[4] = "questions";
        sphinx_grammars[5] = "follow_me";
        sphinx_grammars[6] = "follow_taxi";
        sphinx_grammars[7] = "incomplete_place";
        sphinx_grammars[8] = "incomplete_object";
        sphinx_grammars[9] = "order_food";
        sphinx_grammars[10] = "this_object";
        sphinx_grammars[11] = "obj_pour";
        /*sphinx_grammars[10] = "description_gesture";
        sphinx_grammars[11] = "description_pose";
        sphinx_grammars[12] = "description_hight";
        sphinx_grammars[13] = "description_gender";*/

        if(!poket_reco){
            knowledge_msgs::sphinxConf sp_srv;
            sp_srv.request.flag = false;
            srvEnableSphinx.call(sp_srv);
        }
    
	while (ros::ok()) {

		switch (state) {
		case SM_INIT:
			if (startSignalSM) {
				JustinaHRI::waitAfterSay("I am ready for the gpsr test", 4000);
                if(poket_reco){
                    JustinaHRI::loadGrammarSpeechRecognized("people_names", "/grammars/pre_sydney/people_names.jsgf");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    JustinaHRI::loadGrammarSpeechRecognized("order_drink", "/grammars/pre_sydney/order_drink.jsgf");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    JustinaHRI::loadGrammarSpeechRecognized("confirmation", "/grammars/pre_sydney/commands.jsgf");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    JustinaHRI::loadGrammarSpeechRecognized("gpsr_grammar", "/grammars/pre_sydney/gpsr/gpsr.jsgf");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    JustinaHRI::loadGrammarSpeechRecognized("questions", "/grammars/pre_sydney/gpsr/questions.jsgf");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    JustinaHRI::loadGrammarSpeechRecognized("follow_me", "/grammars/pre_sydney/gpsr/follow_me.jsgf");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    JustinaHRI::loadGrammarSpeechRecognized("follow_taxi", "/grammars/pre_sydney/gpsr/follow_taxi.jsgf");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    JustinaHRI::loadGrammarSpeechRecognized("incomplete_place", "/grammars/pre_sydney/gpsr/incomplete_place.jsgf");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    JustinaHRI::loadGrammarSpeechRecognized("incomplete_object", "/grammars/pre_sydney/gpsr/incomplete_object.jsgf");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    JustinaHRI::loadGrammarSpeechRecognized("order_food", "/grammars/pre_sydney/gpsr/order_food.jsgf");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    JustinaHRI::loadGrammarSpeechRecognized("this_object", "/grammars/pre_sydney/gpsr/this_object.jsgf");
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    JustinaHRI::enableSpeechRecognized(false);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                }
				//state = SM_SAY_WAIT_FOR_DOOR;
				state =  SM_INIT_SPEECH;
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
			JustinaHRI::waitAfterSay("Now I can see that the door is open",4000);
			std::cout << "GPSRTest.->First try to move" << std::endl;
            JustinaNavigation::moveDist(1.0, 4000);
            JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
			if (!JustinaTasks::sayAndSyncNavigateToLoc("arena", 120000)) {
				std::cout << "GPSRTest.->Second try to move" << std::endl;
				if (!JustinaTasks::sayAndSyncNavigateToLoc("arena", 120000)) {
					std::cout << "GPSRTest.->Third try to move" << std::endl;
					if (JustinaTasks::sayAndSyncNavigateToLoc("arena", 120000)) {
						//JustinaHRI::waitAfterSay("please tell me robot yes for confirm the command", 10000);
						//JustinaHRI::waitAfterSay("please tell me robot no for repeat the command", 10000);
						JustinaHRI::waitAfterSay("I am ready for recieve a command", 10000);
						state = SM_SEND_INIT_CLIPS;
					}
				} else {
					//JustinaHRI::waitAfterSay("please tell me robot yes for confirm the command", 10000);
					//JustinaHRI::waitAfterSay("please tell me robot no for repeat the command", 10000);
					JustinaHRI::waitAfterSay("I am ready for recieve a command", 10000);
					state = SM_SEND_INIT_CLIPS;
				}
			} else {
				//JustinaHRI::waitAfterSay("please tell me robot yes for confirm the command", 10000);
				//JustinaHRI::waitAfterSay("please tell me robot no for repeat the command", 10000);
				JustinaHRI::waitAfterSay("I am ready for recieve a command", 10000);
				state = SM_SEND_INIT_CLIPS;
			}
			break;
		case SM_INIT_SPEECH:
			//JustinaHRI::waitAfterSay("please tell me robot yes for confirm the command", 10000);
			//JustinaHRI::waitAfterSay("please tell me robot no for repeat the command", 10000);
			JustinaHRI::waitAfterSay("I am ready for recieve a command", 10000);
			state = SM_SEND_INIT_CLIPS;
		break;
		case SM_SEND_INIT_CLIPS:
			JustinaVision::startQRReader();
			initMsg.successful = false;
			runSMCLIPS = true;
			command_response_pub.publish(initMsg);
			state = SM_RUN_SM_CLIPS;
			break;
		case SM_RUN_SM_CLIPS:
            if(JustinaTasks::tasksStop()){
                // ALL HERE IS TO RESET CLIPS
                std_msgs::Empty msg;
                pubResetTime.publish(msg);
                JustinaHardware::stopRobot();
                state = SM_RESET_CLIPS;
            }
			break;
        case SM_RESET_CLIPS:
            JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
                
		JustinaHRI::waitAfterSay("I am sorry the time is over", 5000); 
		std_msgs::String res1;
        	std::stringstream ss;
                ss.str("");
                ss << "(assert (cmd_stop_eegpsr 1))";
                res1.data = ss.str();
                sendAndRunClips_pub.publish(res1);   
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                state = SM_RUN_SM_CLIPS; 
        break;
		}

		rate.sleep();
		ros::spinOnce();
	}

	JustinaVision::stopQRReader();

	return 0;

}
