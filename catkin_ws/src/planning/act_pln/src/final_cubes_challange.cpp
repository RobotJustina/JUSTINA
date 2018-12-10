#include "ros/ros.h"

#include "knowledge_msgs/PlanningCmdClips.h"
#include "knowledge_msgs/planning_cmd.h"

#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"

#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaRepresentation.h"

#include <vector>

using namespace boost::algorithm;

#define MAX_ATTEMPTS_RECOG 3
#define MAX_ATTEMPTS_CONF 3

enum SMState {
    SM_INIT,
    SM_SAY_WAIT_FOR_DOOR,
    SM_WAIT_FOR_DOOR,
    SM_NAVIGATE_TO_THE_INIT,
    SM_INSTRUCTIONS,
    SM_WAIT_FOR_OPERATOR,
    SM_MEMORIZING_OPERATOR,
    SM_WAIT_FOR_LEGS_FOUND,
    SM_FOLLOWING_PHASE,
    SM_BRING_THE_PYRAMID,
    SM_BRING_THE_PYRAMID_TAKE,
    SM_BRING_THE_PYRAMID_CONF,
    SM_NAVIGATE_TO_THE_LOCATION,
    SM_PLACE_BLOCK,
    SM_LOOK_CUBES_CONFIG,
    SM_SEND_INIT_CLIPS,
    SM_RUN_SM_CLIPS
};

std::vector<std::string> objectsids;
ros::Publisher command_response_pub;
ros::Publisher sendAndRunClips_pub;
ros::Publisher simulated_pub;
SMState nextState;
std::string testPrompt;
bool hasBeenInit;

bool runSMCLIPS = false;
bool startSignalSM = false;
knowledge_msgs::PlanningCmdClips initMsg;

// This is for the attemps for a actions
std::string lastCmdName = "";
int numberAttemps = 0;

int cont_z = 0;
int minDelayAfterSay = 0;
int maxDelayAfterSay = 300;
bool follow_start = false;
std::vector<std::string> validCommandsStop;
std::vector<std::string> validCommandsTake;
std::string lastRecoSpeech;
bool userConfirmation = false;
std::string block;
std::stringstream ss;
int attemptsConfLoc = 0;
int attemptsRecogLoc = 0;
bool recog =false;
int contChances=0;
bool withLeftArm = false;
vision_msgs::VisionFaceObjects faces;

ros::ServiceClient srvCltGetTasks;
ros::ServiceClient srvCltInterpreter;
ros::ServiceClient srvCltWaitConfirmation;
ros::ServiceClient srvCltWaitForCommand;
ros::ServiceClient srvCltAnswer;
ros::ServiceClient srvCltWhatSee;
ros::ServiceClient srvCltExplain;
ros::ServiceClient srvCltDisponible;

vision_msgs::VisionFaceObjects recognizeFaces (float timeOut, bool &recognized)
{
    recognized = false;
    int previousSize = 20;
    int sameValue = 0;
    boost::posix_time::ptime curr;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff;
    vision_msgs::VisionFaceObjects lastRecognizedFaces;

    do
    {
        lastRecognizedFaces = JustinaVision::getFaces();
        
        if(previousSize == 1)
            sameValue ++;
        
        if (sameValue == 3)
            recognized = true;

        else
        {
            previousSize = lastRecognizedFaces.recog_faces.size();
            recognized = false;
        }

        curr = boost::posix_time::second_clock::local_time();
        ros::spinOnce();
    }while(ros::ok() && (curr - prev).total_milliseconds()< timeOut && !recognized);

    std::cout << "recognized:" << recognized << std::endl;
    return lastRecognizedFaces;
}

void validateAttempsResponse(knowledge_msgs::PlanningCmdClips msg) {
    lastCmdName = msg.name;
    if (msg.successful == 0
            && (msg.name.compare("move_actuator") == 0
                || msg.name.compare("find_object") == 0)) {
        if (msg.name.compare(lastCmdName) != 0)
            numberAttemps = 0;
        else if (numberAttemps == 2) {
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
        const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
    std::cout << testPrompt << "--------- Command interpreter ---------"
        << std::endl;
    std::cout << "name:" << msg->name << std::endl;
    std::cout << "params:" << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;

    bool success = ros::service::waitForService(
            "/planning_clips/interpreter_open", 5000);
    if (success) {
        knowledge_msgs::planning_cmd srv;
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
                    ss << srv.response.args << " " << tokens[2] << "_block "
                        << tokens[8] << "_block";
		else if (tokens.size() == 6)
		    ss << srv.response.args << " " << tokens[2] << " "
			<< tokens[5];
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
        const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
    std::cout << testPrompt << "--------- Command Allowed ---------"
        << std::endl;
    std::cout << "name:" << msg->name << std::endl;
    std::cout << "params:" << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;

    std::stringstream ss;

    std::vector<std::string> tokens;
    std::vector<std::string> tokens1;
    std::string str = msg->params;
    split(tokens, str, is_any_of(" "));

    responseMsg.successful = 1;

    if (tokens[0] == "nil" || tokens[0] == "droped" || tokens[1] == "pyramid" || tokens[1] == "no_grap") {

        bool success;
        success = ros::service::waitForService(
                "/planning_clips/disponible", 5000);
        if (success) {
            std::cout << "------------- No Disponible: ------------------ "
                << std::endl;

            if (tokens[2] == "found")
                JustinaTasks::sayAndSyncNavigateToLoc("dining_room", 12000, false);

            knowledge_msgs::planning_cmd srv;
            srv.request.name = "test_disponible";
            srv.request.params = responseMsg.params;
            if (srvCltDisponible.call(srv)) {
                std::cout << "Response of confirmation:" << std::endl;
                std::cout << "Success:" << (long int) srv.response.success
                    << std::endl;
                std::cout << "Args:" << srv.response.args << std::endl;

                if(tokens[0] == "nil")
                    JustinaHRI::waitAfterSay("I can not complete your order because the object is not on the table",1500);
                else if(tokens[0] == "table")
                    std::cout << "the object is on the table" << std::endl;

                else if (tokens[0] == "nil" && tokens[0] != "nobody" )
                {
                    int count = 0;
                    for(int i = 0; i < 10 && ros::ok(); i++){
                        vision_msgs::VisionFaceObjects lastRecognizedFaces = tokens[3].compare("") == 0? JustinaVision::getFaces(): JustinaVision::getFaceRecognition(tokens[3]);
                        if(lastRecognizedFaces.recog_faces.size() > 0)
                            count++;
                    }
                    ss.str("");
                    if(count > 5)
                        ss << "You already have the object";
                    else
                        ss << tokens[3] << " already has the object";
                    std::cout << ss.str() << std::endl;
                    JustinaHRI::waitAfterSay(ss.str(), 1000);
                }
                else if(tokens[0] == "droped") {
                    int count = 0;
                    for(int i = 0; i < 10 && ros::ok(); i++){
                        vision_msgs::VisionFaceObjects lastRecognizedFaces = tokens[3].compare("") == 0 ? JustinaVision::getFaces(): JustinaVision::getFaceRecognition(tokens[3]);
                        if(lastRecognizedFaces.recog_faces.size() > 0)
                            count++;
                    }
                    ss.str("");
                    if(count > 5)
                        ss << "You already have the object";
                    else
                        ss << tokens[3] << " already has the object";
                    std::cout << ss.str() << std::endl;
                    JustinaHRI::waitAfterSay(ss.str(), 1000);
                }
		else if(tokens[1] == "pyramid"){
			ss.str("");
    			split(tokens1, tokens[3], is_any_of("_"));
			ss << "I am sorry, I can not put any block on top of the " << tokens1[0] << " " << tokens1[1] << ", because is a pyramid";
                	JustinaHRI::waitAfterSay(ss.str(), 1000);
		}
        else if (tokens[1] == "no_grab"){
            ss.str("");
            ss << "I can not grab the cube";
            JustinaHRI::waitAfterSay(ss.str(), 1000);
        }
                else
                    JustinaHRI::waitAfterSay("the object is not on the table",1000);

                JustinaHRI::waitAfterSay("Would you like something else", 1000);
                responseMsg.successful = 0;
            } else {
                std::cout << testPrompt
                    << "Failed to call service of disponible"
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

void callbackCmdHappen(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
    std::cout << testPrompt << "--------- Command Happen ---------"
        << std::endl;
    std::cout << "name:" << msg->name << std::endl;
    std::cout << "params:" << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;

    std::vector<std::string> tokens;
    std::string str = msg->params;
    split(tokens, str, is_any_of(" "));

    responseMsg.successful = 1;

    if (tokens[1] == "nil" && tokens[2] != "nobody") {
        JustinaHRI::waitAfterSay("Some one else take the object", 1000);
        JustinaHRI::waitAfterSay("Would you like something else", 1000);
        responseMsg.successful = 0;
        responseMsg.params = "obj prs fuente";
    }

    if (tokens[1] == "nil" && tokens[2] == "nobody") {
        JustinaHRI::waitAfterSay("Some one else take the obkect", 1000);
        JustinaHRI::waitAfterSay("Would you like something else", 1000);
        responseMsg.successful = 0;
        responseMsg.params = "obj prs fuente";
    }

    else if (tokens[1] == "table") {
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
        const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
    std::cout << testPrompt << "--------- Command confirmation ---------"
        << std::endl;
    std::cout << "name:" << msg->name << std::endl;
    std::cout << "params:" << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;
    
////////only for cubes configuration
    std::vector<std::string> tokens;
    std::string str = responseMsg.params;
    split(tokens, str, is_any_of("_"));
	bool equal_cubes = false;
	if(tokens.size() > 6){
		std::cout << "token2: " << tokens[2] << " token6: " << tokens[6] << std::endl;
		if(tokens[2] == tokens[6])
			equal_cubes = true;
	}
////

    bool success = ros::service::waitForService("spg_say", 5000);
    success = success
        & ros::service::waitForService(
                "/planning_clips/confirmation", 5000);

    if (success) {

        std::string to_spech = responseMsg.params;
        boost::replace_all(to_spech, "_", " ");
        std::stringstream ss;

        ss << "Do you want me " << to_spech;
        std::cout << "------------- to_spech: ------------------ " << ss.str()
            << std::endl;
        JustinaHRI::waitAfterSay(ss.str(), 2000);

        knowledge_msgs::planning_cmd srv;
        srv.request.name = "test_confirmation";
        srv.request.params = responseMsg.params;
        if (srvCltWaitConfirmation.call(srv) && !equal_cubes) {
            std::cout << "Response of confirmation:" << std::endl;
            std::cout << "Success:" << (long int) srv.response.success
                << std::endl;
            std::cout << "Args:" << srv.response.args << std::endl;
            /*if (srv.response.success)
                JustinaHRI::waitAfterSay("Do you want me explain the plan", 1500);
            else
                JustinaHRI::waitAfterSay("Repeate the command please", 1000);*/

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

void callbackCmdTaskConfirmation( const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
    std::cout << testPrompt << "--------- Command confirm task -------" << std::endl;
    std::cout << "name: " << msg->name << std::endl;
    std::cout << "params: " << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;
    
    bool success = ros::service::waitForService("spg_say", 5000);
    success = success & ros::service::waitForService("/planning_clips/confirmation", 5000);

    if (success) {

        std::string to_spech = responseMsg.params;
        boost::replace_all(to_spech, "_", " ");
        std::stringstream ss;

        ss << "Do you want me " << to_spech;
        std::cout << "------------- to_spech: ------------------ " << ss.str()
            << std::endl;
        JustinaHRI::waitAfterSay(ss.str(), 2000);

        knowledge_msgs::planning_cmd srv;
        srv.request.name = "test_confirmation";
        srv.request.params = responseMsg.params;
        if (srvCltWaitConfirmation.call(srv)) {
            std::cout << "Response of confirmation:" << std::endl;
            std::cout << "Success:" << (long int) srv.response.success
                << std::endl;
            std::cout << "Args:" << srv.response.args << std::endl;

            responseMsg.params = "conf";
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
    //validateAttempsResponse(responseMsg);

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

    bool success = ros::service::waitForService(
            "/planning_clips/get_task", 5000);
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

void callbackCmdExplainThePlan(
        const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
    std::cout << testPrompt << "--------- Explain the plan ---------"
        << std::endl;
    std::cout << "name:" << msg->name << std::endl;
    std::cout << "params:" << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;
    bool explain;

    bool success2 = ros::service::waitForService(
            "/planning_clips/what_see", 5000);
    if (success2) {

        knowledge_msgs::planning_cmd srv2;
        srv2.request.name = "test_what_see";
        srv2.request.params = responseMsg.params;

        if (srvCltWhatSee.call(srv2)) {

            if (srv2.response.args == "explain") {
                JustinaHRI::waitAfterSay("I am going to explain the plan",
                        2000);
                explain = true;
            } else {
                //JustinaHRI::waitAfterSay("I start to execute the plan", 1000);
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
            "/planning_clips/plan_explain", 5000);
    if (success) {
        bool finish = false;
        do {
            knowledge_msgs::planning_cmd srv;
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
                    std::string param3 = tokens[4];

                    if (param1.compare("update_object_location") == 0
                            && explain) {
                        ss.str("");
                        ss << "I have to locate the " << param2
                            << " on the table";
                        //tasks.syncSpeech(ss.str(), 30000, 2000);
                    } else if (param1.compare("get_object") == 0 && explain) {
                        ss.str("");
                        ss << "First, I need to align myself to the table";
                        JustinaHRI::waitAfterSay(ss.str(), 10000);
                        ss.str("");
                        ss << "After that, I need to find the " << param2 << " on the table";
                        JustinaHRI::waitAfterSay(ss.str(), 10000);
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                        ss.str("");
                        ss << "Then, I need to grasp it";
                        JustinaHRI::waitAfterSay(ss.str(), 10000);
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                    } else if (param1.compare("find_person_in_room") == 0
                            && explain) {
                        ss.str("");
                        ss << "After that, I need to find " << param2;
                        JustinaHRI::waitAfterSay(ss.str(), 10000);
                        ss.str("");
                        ss << "And, I will approach myself to him";
                        JustinaHRI::waitAfterSay(ss.str(), 10000);
                    } else if (param1.compare("handover_object") == 0
                            && explain) {
                        ss.str("");
                        ss << "Then, I need to verify that the recipient, is in front of me";
                        JustinaHRI::waitAfterSay(ss.str(), 10000);
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                        ss.str("");
                        ss << "Finally, I will deliver the " << param2 << " to " << param3;
                        JustinaHRI::waitAfterSay(ss.str(), 10000);
                    }
                }
            } else {
                std::cout << testPrompt << "Failed to call get tasks"
                    << std::endl;
                responseMsg.successful = 0;
            }
            responseMsg.successful = 1;
        } while (ros::ok() && !finish);
    } else {/// end explain service
        std::cout << testPrompt << "Needed services are not available :'("
            << std::endl;
        responseMsg.successful = 0;
    }
    responseMsg.name = "cmd_explain";
    responseMsg.params = "open 3";
    responseMsg.id = msg->id;
    responseMsg.successful = 1;
    bool no_execute = false;

    JustinaHRI::waitAfterSay("Do you want me start to execute the plan", 1500);
    success2 = ros::service::waitForService("/planning_clips/what_see", 5000);
    if (success2) {

        knowledge_msgs::planning_cmd srv3;
        srv3.request.name = "test_what_see";
        srv3.request.params = responseMsg.params;

        if (srvCltWhatSee.call(srv3)) {

            if (srv3.response.args == "execute") {
                JustinaHRI::waitAfterSay("I will start to execute the plan", 1500);
                no_execute = false;
            } else {
                responseMsg.successful = 0;
                no_execute = true;
            }
        }

        else {
            std::cout << testPrompt << "Failed to call service execute plan"
                << std::endl;
            responseMsg.successful = 0;
        }
    } else {
        std::cout << testPrompt << "Needed services are not available :'("
            << std::endl;
        responseMsg.successful = 0;
    }

    bool success3;
    success3 = ros::service::waitForService("/planning_clips/disponible", 5000);
    if (success && no_execute) {

        knowledge_msgs::planning_cmd srv4;
        srv4.request.name = "test_disponible";
        srv4.request.params = responseMsg.params;
        if (srvCltDisponible.call(srv4))
            std::cout << testPrompt << "No se ejecutara el plan " << std::endl;
        JustinaHRI::waitAfterSay("Ok would you like something else", 1500);
    }
    else{
        std::cout << testPrompt << "Needed services are not available :'("
            << std::endl;
    }

    validateAttempsResponse(responseMsg);
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

    bool success = ros::service::waitForService("spg_say", 5000);
    if (success) {
        std::string param1 = tokens[1];
        if (param1.compare("a_question") == 0) {
            success = ros::service::waitForService("/planning_clips/answer",
                    5000);
            if (success) {
                success = JustinaHRI::waitAfterSay(
                        "I am waiting for the user question", 1000);
                knowledge_msgs::planning_cmd srv;
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

void callbackCmdWorld(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
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

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;

    knowledge_msgs::PlanningCmdClips responseModify;
    responseModify.name = "cmd_modify";
    responseModify.params = "modify";
    responseModify.id = msg->id;

    knowledge_msgs::PlanningCmdClips responseObject;
    responseObject.name = "cmd_wobj";
    responseObject.params = "wobj";
    responseObject.id = msg->id;

    std::stringstream ss;

    //responseMsg.successful = 1;
    //command_response_pub.publish(responseMsg);

    //bool success = ros::service::waitForService("spg_say", 5000);
    bool success = ros::service::waitForService(
            "/planning_clips/what_see", 5000);
    if (success) {

        knowledge_msgs::planning_cmd srv;
        srv.request.name = "test_what_see";
        srv.request.params = responseMsg.params;

        if (srvCltWhatSee.call(srv)) {

            if (srv.response.args == "what_see_person") {
                JustinaHRI::waitAfterSay(
                        "I am looking for people in the scene", 1500);

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
                JustinaVision::startFaceRecognition(true);
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

                JustinaManip::hdGoTo(0.52, 0.0, 5000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

                JustinaManip::hdGoTo(-0.52, 0.0, 5000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

                JustinaManip::hdGoTo(0, -0.4, 5000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                JustinaManip::hdGoTo(0, 0.0, 5000);

                prev = boost::posix_time::second_clock::local_time();

                do {
                    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                    JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);

                    ///El robot se mueve a una nueva posicion
                    //JustinaNavigation::moveLateral(-0.3, 4000);
                    JustinaManip::hdGoTo(0.0, 0, 5000);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
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
                }while (ros::ok() && (curr - prev).total_milliseconds() < timeOut && srv.response.args == "what_see_person");

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
        if (srv.response.args == "what_see_person") {
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
        JustinaVision::startFaceRecognition(false);
        responseMsg.params = srv.response.args;
        responseMsg.successful = srv.response.success;

        if (srv.response.args == "what_see_obj") {

            //if(objectsids.size()>0)
            //	objectsids.erase(objectsids.begin());
            //boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
            JustinaTasks::sayAndSyncNavigateToLoc("table", 12000);

            JustinaHRI::waitAfterSay(
                    "I am looking for stacks on the table", 1500);
            JustinaManip::hdGoTo(0, -0.9, 5000);
            boost::this_thread::sleep(
                    boost::posix_time::milliseconds(1000));
            JustinaTasks::alignWithTable(0.42);
            boost::this_thread::sleep(
                    boost::posix_time::milliseconds(1000));
            std::stringstream sss;
            std::stringstream speech;
            std::string block = "table";

            vision_msgs::CubesSegmented cubes;
            vision_msgs::Cube cube_aux;
            cube_aux.color = "red";
            cubes.recog_cubes.push_back(cube_aux);
            cube_aux.color = "blue";
            cubes.recog_cubes.push_back(cube_aux);
            cube_aux.color = "green";
            cubes.recog_cubes.push_back(cube_aux);
            cube_aux.color = "yellow";
            cubes.recog_cubes.push_back(cube_aux);
            std::vector<vision_msgs::CubesSegmented> Stacks;
            tf::StampedTransform transform;
            tf::TransformListener* tf_listener = new tf::TransformListener();

            bool fcubes;
            fcubes = JustinaVision::getCubesSeg(cubes);
            std::cout << "GET CUBES: " << fcubes << std::endl;
            Stacks.resize(4);
            int num_piles = 0;
            //if(fcubes) fcubes = JustinaTasks::sortCubes(cubes,Stacks);
            if(fcubes) fcubes = JustinaTasks::getStacks(cubes,Stacks,num_piles);
            std::cout << "SORT CUBES: " << fcubes << std::endl;
            for(int j=0; j < Stacks.size(); j++){
                std_msgs::String res1;
                sss.str("");
                speech.str("");
                block = "table";
                sss << "(assert (stack_dynamic";
                for(int k = Stacks.at(j).recog_cubes.size(); k > 0 ;k--){
                    ss.str("");
                    std::cout << "CUBE: " << Stacks.at(j).recog_cubes.at(k-1).color << std::endl;
                   
                    JustinaKnowledge::addUpdateObjectViz(Stacks.at(j).recog_cubes.at(k-1).color, Stacks.at(j).recog_cubes.at(k-1).minPoint.x, Stacks.at(j).recog_cubes.at(k-1).minPoint.y, Stacks.at(j).recog_cubes.at(k-1).minPoint.z, Stacks.at(j).recog_cubes.at(k-1).maxPoint.x, Stacks.at(j).recog_cubes.at(k-1).maxPoint.y, Stacks.at(j).recog_cubes.at(k-1).maxPoint.z, Stacks.at(j).recog_cubes.at(k-1).cube_centroid.x, Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y, Stacks.at(j).recog_cubes.at(k-1).cube_centroid.z, Stacks.at(j).recog_cubes.at(k-1).colorRGB.x, Stacks.at(j).recog_cubes.at(k-1).colorRGB.y, Stacks.at(j).recog_cubes.at(k-1).colorRGB.z, "base_link", "map");

                    tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
                    tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
                    tf::Vector3 pos(Stacks.at(j).recog_cubes.at(k-1).cube_centroid.x,
                                    Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y,
                                    Stacks.at(j).recog_cubes.at(k-1).cube_centroid.z);

                    pos = transform * pos;
                    ss << "(assert (cmd_insert cube " << Stacks.at(j).recog_cubes.at(k-1).color << "_block "
                       << pos.getX() << " " << pos.getY() << " " << pos.getZ();


                    if(Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y > 0)
                           ss << " left 1))";
                    else
                            ss << " right 1))";

                    sss << " " << Stacks.at(j).recog_cubes.at(k-1).color << "_block";
                    speech << "the " << block << " block is on top of the " << Stacks.at(j).recog_cubes.at(k-1).color << " block";
                    
                    if( k < Stacks.at(j).recog_cubes.size())    {
                        JustinaHRI::waitAfterSay(speech.str(), 1500);
                    }
                    speech.str("");
                    block = Stacks.at(j).recog_cubes.at(k-1).color;
                    
                    res1.data = ss.str();
                    sendAndRunClips_pub.publish(res1);
			        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                }
                if(block != "table"){
                    speech << "the " << block << " block is on top of the table";
                    JustinaHRI::waitAfterSay(speech.str(), 1500);
                }
                sss << "))";
                res1.data = sss.str();
                sendAndRunClips_pub.publish(res1);
			    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            }

        }				///termina recog objects

        if (srv.response.args == "what_see_person" || srv.response.args == "what_see_obj" ) {
            JustinaTasks::sayAndSyncNavigateToLoc("dining_room", 12000, false);
            JustinaHRI::waitAfterSay("I am ready for another petition",
                    1500);
        }

    } else {
        std::cout << testPrompt << "Failed to call service what do you see"
            << std::endl;
        responseMsg.successful = 0;
        JustinaHRI::waitAfterSay("Repeate the question please", 1500);
    }
} else {
    std::cout << testPrompt << "Needed services are not available :'("
        << std::endl;
    responseMsg.successful = 0;
}
command_response_pub.publish(responseMsg);
}

void callbackCmdDescribe(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
    std::cout << testPrompt << "--------- Command Describe ---------"
        << std::endl;
    std::cout << "name:" << msg->name << std::endl;
    std::cout << "params:" << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseDescribe;
    responseDescribe.name = "cmd_world";
    responseDescribe.params = "what_see_yes";
    responseDescribe.id = msg->id;
    responseDescribe.successful = 1;

    std::vector<std::string> tokens;
    std::string str = msg->params;
    split(tokens, str, is_any_of(" "));

    if (tokens[3] == "nil" && tokens[1] == "nil") {
        JustinaHRI::waitAfterSay("There are no persons in the room", 2000);
        std::cout << "There are no persons in the room" << std::endl;
    } else {
        if (tokens[3] != "nil" && tokens[3] != "solo") {
            if (tokens[1] == "derecha") {
                JustinaHRI::waitAfterSay("peter is in the left of john", 2000);
                std::cout << "peter is in the left of john" << std::endl;
            }
            if (tokens[1] == "izquerda") {
                JustinaHRI::waitAfterSay("peter is in the right of john", 2000);
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
                JustinaHRI::waitAfterSay("john is in the left of peter", 2000);
                std::cout << "john is in the left of peter" << std::endl;
            }
            if (tokens[3] == "izquerda") {
                JustinaHRI::waitAfterSay("john is in the right of peter", 2000);
                std::cout << "john is in the right of peter" << std::endl;
            }
        }
        if (tokens[1] == "nil" && tokens[3] != "nil") {
            JustinaHRI::waitAfterSay("john is the only person in the room",
                    2000);
            std::cout << "john is the only person in the room" << std::endl;
        }
    }
    std::stringstream ss;

    for (int k = 0; k < objectsids.size(); k++) {
        ss.str("");
        ss << "There are a " << objectsids[k] << " in the table";
        std::cout << ss.str() << std::endl;
        JustinaHRI::waitAfterSay(ss.str(), 2000);

    }
    if (objectsids.size() == 0) {
        std::cout << "There are not objects in the room" << std::endl;
        JustinaHRI::waitAfterSay("there are not objects in the room", 2000);
    }

    command_response_pub.publish(responseDescribe);
}

void callbackCmdWhere(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
    std::cout << testPrompt << "--------- Command Where ---------" << std::endl;
    std::cout << "name:" << msg->name << std::endl;
    std::cout << "params:" << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseDescribe;
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
        ss << "I dont know, where the " << tokens[0] << " is";
        std::cout << ss.str() << std::endl;
        JustinaHRI::waitAfterSay(ss.str(), 1500);
    } else if (tokens[1] == "table") {
        ss.str("");
        ss << "The " << tokens[0] << " is on the " << tokens[1];
        std::cout << ss.str() << std::endl;
        JustinaHRI::waitAfterSay(ss.str(), 1500);
    } else if (tokens[1] == "nil" && tokens[2] != "nobody") {
        int count = 0;
        for(int i = 0; i < 10 && ros::ok(); i++){
            vision_msgs::VisionFaceObjects lastRecognizedFaces = tokens[2].compare("") == 0 ? JustinaVision::getFaces(): JustinaVision::getFaceRecognition(tokens[2]);
            if(lastRecognizedFaces.recog_faces.size() > 0)
                count++;
        }
        ss.str("");
        if(count > 5)
            ss << "You already have the " << tokens[0];
        else
            ss << tokens[2] << " already has the " << tokens[0];
        std::cout << ss.str() << std::endl;
        JustinaHRI::waitAfterSay(ss.str(), 1000);
    } else if (tokens[1] == "droped") {
        int count = 0;
        for(int i = 0; i < 10 && ros::ok(); i++){
            vision_msgs::VisionFaceObjects lastRecognizedFaces = tokens[2].compare("") == 0 ? JustinaVision::getFaces() : JustinaVision::getFaceRecognition(tokens[2]);
            if(lastRecognizedFaces.recog_faces.size() > 0)
                count++;
        }
        ss.str("");
        if(count > 5)
            ss << "You already have the " << tokens[0];
        else
            ss << tokens[2] << " already has the " << tokens[0];
        std::cout << ss.str() << std::endl;
        JustinaHRI::waitAfterSay(ss.str(), 1000);
    }

    command_response_pub.publish(responseDescribe);
}

void callbackCmdTakeOrder(
        const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
    std::cout << testPrompt << "--------- Take order ---------" << std::endl;
    std::cout << "name:" << msg->name << std::endl;
    std::cout << "params:" << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;
    bool success = true;
    startSignalSM = true;

    //if(!runSMCLIPS)
    //	success = false;
    success = success
        & ros::service::waitForService(
                "/planning_clips/wait_command", 50000);
    if (success) {
        JustinaHRI::waitAfterSay("Yes what is your order", 1500);
        knowledge_msgs::planning_cmd srv;
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
    std::vector<std::string> blocks;
    std::string str = responseMsg.params;
    split(tokens, str, is_any_of(" "));
    split(blocks, tokens[0], is_any_of("_"));
    std::stringstream ss;

    bool success = ros::service::waitForService("spg_say", 5000);
    if (success) {
        std::cout << testPrompt << "find: " << tokens[0] << std::endl;

        ss.str("");
        if (tokens[0] == "person") {
            success = JustinaTasks::findPerson("", -1, JustinaTasks::STANDING, false);
            ss << responseMsg.params << " " << 1 << " " << 1 << " " << 1;
        } else if (tokens[0] == "man") {
            success = JustinaTasks::findAndFollowPersonToLoc(tokens[1]);
            ss << responseMsg.params;
        } else if (tokens[0] == "specific") {
            success = JustinaTasks::findPerson(tokens[1], -1, JustinaTasks::STANDING, true);
            ss << responseMsg.params;
        } else if (blocks.size() > 1){
            bool fcubes;
            bool withLeftOrRightArm; 
            if(blocks[1] == "block"){
                vision_msgs::CubesSegmented cubes;
                vision_msgs::Cube cube_aux;
                cube_aux.color = blocks[0];
                cubes.recog_cubes.push_back(cube_aux);
                JustinaManip::hdGoTo(0, -0.9, 5000);
                 boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                fcubes = JustinaVision::getCubesSeg(cubes);
                std::cout << "GET CUBES: " << fcubes << std::endl;
                if(fcubes && cubes.recog_cubes.at(0).detected_cube)
                    if(cubes.recog_cubes.at(0).cube_centroid.y > 0){
                    ss << responseMsg.params << " " << cubes.recog_cubes.at(0).cube_centroid.x << " "
                        << cubes.recog_cubes.at(0).cube_centroid.y << " "
                        << cubes.recog_cubes.at(0).cube_centroid.z << " left";
                    }
                    else{
                    ss << responseMsg.params << " " << cubes.recog_cubes.at(0).cube_centroid.x << " "
                        << cubes.recog_cubes.at(0).cube_centroid.y << " "
                        << cubes.recog_cubes.at(0).cube_centroid.z << " right";
                    }
            }
        }
        
        else {
            geometry_msgs::Pose pose;
            bool withLeftOrRightArm;
            success = JustinaTasks::findObject(tokens[0], pose, withLeftOrRightArm);
			//if(tokens[0] != "milk"){
			//	pose.position.x = 1;
			//	pose.position.y = 1;
			//	pose.position.z = 1;}
			if(withLeftOrRightArm)
				ss << responseMsg.params << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " left";
			else
				ss << responseMsg.params << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " right";
        }
        responseMsg.params = ss.str();
    }
    if (success)
        responseMsg.successful = 1;
    else
        responseMsg.successful = 0;
   // responseMsg.successful = 1;
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

    bool success = true;
    success = JustinaTasks::alignWithTable(0.42);
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
    std::vector<std::string> blocks;
    std::string str = responseMsg.params;
    split(tokens, str, is_any_of(" "));
    split(blocks, tokens[0], is_any_of("_"));
    bool armFlag = true;
    std::stringstream ss;

    bool success = ros::service::waitForService("spg_say", 5000);
	if(tokens[4] == "false")
			armFlag = false;
    if(blocks.size() == 2){
        success = success & JustinaTasks::graspBlockFeedback(atof(tokens[1].c_str()),
                atof(tokens[2].c_str()), atof(tokens[3].c_str()), armFlag,
                blocks[0], true);
    }
    else if(blocks.size() == 3){
        JustinaTasks::graspObject(atof(tokens[1].c_str()),
               atof(tokens[2].c_str()), atof(tokens[3].c_str()), armFlag, blocks[0].c_str(), true, true);
        success = true;
    }
    else{
        success = success & JustinaTasks::moveActuatorToGrasp(atof(tokens[1].c_str()),
                atof(tokens[2].c_str()), atof(tokens[3].c_str()), armFlag,
                tokens[0], true);
    }
    if (success)
        responseMsg.successful = 1;
	else{
		ss.str("");
		ss << "I did not grasp the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(), 100000);
		responseMsg.successful = 0;
	}

    ss.str("");
    if(blocks.size() > 1){
    ss << blocks[0] << "_" 
        << blocks[1] << " " 
        << tokens[1] << " "
        << tokens[2] << " "
        << tokens[3] << " " << tokens[4];
        //responseMsg.successful = 1;
        responseMsg.params = ss.str();}

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
    std::vector<std::string> block1;
    std::vector<std::string> block2;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
    if(tokens[0] == "block" || tokens[0] == "simul")
        split(block1, tokens[1], is_any_of("_"));
    if(tokens.size() > 3)
        split(block2, tokens[3], is_any_of("_"));
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
		(armFlag) ? JustinaManip::laGoTo("home", 6000) : JustinaManip::raGoTo("home", 6000);
	}
    else if(tokens[0] == "block"){
        ss.str("");
        ss << "I am going to place the " << block1[0] << " " << block1[1]
            << " on the " << block2[0] << " " << block2[1];
        JustinaHRI::waitAfterSay(ss.str(), 2000);
        //succes = JustinaTasks::placeBlockOnBlock(atof(tokens[4].c_str()), atof(tokens[5].c_str()), atof(tokens[6].c_str()), armFlag, block2[0]);
        succes = JustinaTasks::placeBlockOnBlock(atof(tokens[4].c_str()), armFlag, block2[0]);
        (armFlag) ? JustinaManip::laGoTo("home", 6000) : JustinaManip::raGoTo("home", 6000);
    }
    else if (tokens[0] == "simul"){
        ss.str("");
        if(tokens[1] == tokens[3]){
            tf::StampedTransform transform;
            tf::TransformListener* tf_listener = new tf::TransformListener();
            tf::Vector3 p(atof(tokens[5].c_str()), atof(tokens[6].c_str()), atof(tokens[7].c_str()));

            tf_listener->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(10.0));
            tf_listener->lookupTransform("base_link", "map", ros::Time(0), transform);
        
            p = transform * p;
            if(armFlag)
                p.setY(p.getY() + 0.15);
            else
                p.setY(p.getY() - 0.15);
            //p.setZ(p.getZ() - 0.138);
            
            tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
            tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);

            p = transform * p;

            JustinaTasks::placeBlockOnBlock(atof(tokens[4].c_str()), armFlag, block1[0], true,
                    p.getX(), p.getY(), 0.71, true);
                    //atof(tokens[5].c_str()), atof(tokens[6].c_str()),atof(tokens[7].c_str()));
                    
            (armFlag) ? JustinaManip::laGoTo("home", 6000) : JustinaManip::raGoTo("home", 6000);

            succes = true;
            
            ss.str("");
            ss << tokens[0] << " " << tokens[1] << " " << tokens[2] << " " << tokens[3] << " " << tokens[4] << " "
                << p.getX() << " " << p.getY() << " " << 0.802;

            responseMsg.params = ss.str();

        }
        else{
            JustinaTasks::placeBlockOnBlock(atof(tokens[4].c_str()), armFlag, block1[0], true,
                        atof(tokens[5].c_str()), atof(tokens[6].c_str()), atof(tokens[7].c_str()), true);
            (armFlag) ? JustinaManip::laGoTo("home", 6000) : JustinaManip::raGoTo("home", 6000);
            succes = true;
            float z_simul = atof(tokens[7].c_str()) + atof(tokens[4].c_str());
            ss.str("");
            ss << tokens[0] << " " << tokens[1] << " " << tokens[2] << " " << tokens[3] << " " << tokens[4] << " "
                << tokens[5] << " " << tokens[6] << " " << z_simul;
        }
    }

	
	if (succes)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
    
    //responseMsg.successful = 1;

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

void callbackReviewStack(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
    std::cout << testPrompt << "---------- Command Review Stacks --------- " << std::endl;
    std::cout << "name: " << msg->name << std::endl;
    std::cout << "params: " << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;


    //JustinaHRI::waitAfterSay(
    //        "I am looking for stacks on the table", 1500);
    JustinaManip::hdGoTo(0, -0.9, 5000);
    boost::this_thread::sleep(
            boost::posix_time::milliseconds(1000));
    JustinaTasks::alignWithTable(0.42);
    boost::this_thread::sleep(
            boost::posix_time::milliseconds(1000));
    std::stringstream sss;
    std::stringstream ss;

    vision_msgs::CubesSegmented cubes;
    vision_msgs::Cube cube_aux;
    cube_aux.color = "red";
    cubes.recog_cubes.push_back(cube_aux);
    cube_aux.color = "blue";
    cubes.recog_cubes.push_back(cube_aux);
    cube_aux.color = "green";
    cubes.recog_cubes.push_back(cube_aux);
    cube_aux.color = "yellow";
    cubes.recog_cubes.push_back(cube_aux);
    std::vector<vision_msgs::CubesSegmented> Stacks;
    tf::StampedTransform transform;
    tf::TransformListener* tf_listener = new tf::TransformListener();
    bool fcubes;
    int num_piles = 0;
    fcubes = JustinaVision::getCubesSeg(cubes);
    //if(fcubes) fcubes = JustinaTasks::getStacks(cubes,Stacks,num_piles);
    std::cout << "GET CUBES: " << fcubes << std::endl;
    Stacks.resize(4);
    //if(fcubes) fcubes = JustinaTasks::sortCubes(cubes,Stacks);
    if(fcubes) fcubes = JustinaTasks::getStacks(cubes,Stacks,num_piles);
    std::cout << "SORT CUBES: " << fcubes << std::endl;
    for(int j=0; j < Stacks.size(); j++){
        std_msgs::String res1;
        sss.str("");
        sss << "(assert (stack_review";
        for(int k = Stacks.at(j).recog_cubes.size(); k > 0 ;k--){
            ss.str("");
            std::cout << "CUBE: " << Stacks.at(j).recog_cubes.at(k-1).color << std::endl;
            sss << " " << Stacks.at(j).recog_cubes.at(k-1).color << "_block";

                
                /*JustinaKnowledge::addUpdateObjectViz(Stacks.at(j).recog_cubes.at(k-1).color, Stacks.at(j).recog_cubes.at(k-1).minPoint.x, Stacks.at(j).recog_cubes.at(k-1).minPoint.y, Stacks.at(j).recog_cubes.at(k-1).minPoint.z, Stacks.at(j).recog_cubes.at(k-1).maxPoint.x, Stacks.at(j).recog_cubes.at(k-1).maxPoint.y, Stacks.at(j).recog_cubes.at(k-1).maxPoint.z, Stacks.at(j).recog_cubes.at(k-1).cube_centroid.x, Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y, Stacks.at(j).recog_cubes.at(k-1).cube_centroid.z, Stacks.at(j).recog_cubes.at(k-1).colorRGB.x, Stacks.at(j).recog_cubes.at(k-1).colorRGB.y, Stacks.at(j).recog_cubes.at(k-1).colorRGB.z, "base_link", "map");
                tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
                tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
                tf::Vector3 pos(Stacks.at(j).recog_cubes.at(k-1).cube_centroid.x,
                                Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y,
                                Stacks.at(j).recog_cubes.at(k-1).cube_centroid.z);

                pos = transform * pos;
                ss << "(assert (cmd_insert cube " << Stacks.at(j).recog_cubes.at(k-1).color << "_block "
                   << pos.getX() << " " << pos.getY() << " " << pos.getZ();


                if(Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y > 0)
                       ss << " left 1))";
                else
                        ss << " right 1))";

                res1.data = ss.str();
                sendAndRunClips_pub.publish(res1);
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));*/
            /*if(Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y >0){
                ss << "(assert (cmd_insert cube " << Stacks.at(j).recog_cubes.at(k-1).color << "_block " 
                    << Stacks.at(j).recog_cubes.at(k-1).cube_centroid.x << " " 
                    << Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y << " "
                    << Stacks.at(j).recog_cubes.at(k-1).cube_centroid.z << " left 1))";
            }
            else{
                ss << "(assert (cmd_insert cube " << Stacks.at(j).recog_cubes.at(k-1).color << "_block " 
                    << Stacks.at(j).recog_cubes.at(k-1).cube_centroid.x << " " 
                    << Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y << " "
                    << Stacks.at(j).recog_cubes.at(k-1).cube_centroid.z << " right 1))";
            }

            res1.data = ss.str();
            sendAndRunClips_pub.publish(res1);
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));*/

        }
        sss << "))";
        res1.data = sss.str();
        sendAndRunClips_pub.publish(res1);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }

    responseMsg.successful = 1;
    command_response_pub.publish(responseMsg);

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
	
	ss << tokens[0];
	for(int i=1 ; i<tokens.size(); i++)
		ss << " "<< tokens[i];
	
	JustinaHRI::waitAfterSay(ss.str(), 10000);

	responseMsg.params = "sayed";
	responseMsg.successful = 1;

	command_response_pub.publish(responseMsg);
	

}

void callbackCmdalignWithPoint(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
    std::cout << testPrompt << "-------- Command Align with Point ----" << std::endl;
    std::cout << "name: " << msg->name << std::endl;
    std::cout << "params: " << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;

    std::stringstream ss;
    std::vector<std::string> tokens;
    std::string str = responseMsg.params;
    split(tokens, str, is_any_of(" "));

    JustinaTasks::alignWithPoint(atof(tokens[0].c_str()), atof(tokens[1].c_str()), atof(tokens[2].c_str()), tokens[3], tokens[4]);

    responseMsg.successful = 1;

    command_response_pub.publish(responseMsg);

}

void callbackCmdMakeBacktraking(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Backtracking-----" << std::endl;
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
	
	ss << tokens[0];
	for(int i=1 ; i<tokens.size(); i++)
		ss << " "<< tokens[i];
	
	JustinaHRI::waitAfterSay(ss.str(), 10000);

	responseMsg.params = "backtracking";
	responseMsg.successful = 1;

	command_response_pub.publish(responseMsg);
	

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
        std::cout << "person" << std::endl;
    } else {
        bool goToTable = false;
        if(tokens[1].compare("table") == 0)
            goToTable = true;
        success = JustinaTasks::sayAndSyncNavigateToLoc(tokens[1], 12000, goToTable);
        std::cout << "inspection" << std::endl;
    }
    if (success)
        responseMsg.successful = 1;
    else
        responseMsg.successful = 0;
    validateAttempsResponse(responseMsg);
    //command_response_pub.publish(responseMsg);
}

void callbackEnableSimul(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
    std::cout << testPrompt << "-------- Command Enable Simul" << std::endl;
    std::cout << "name: " << msg->name << std::endl;
    std::cout << "params: " << msg->params << std::endl;
    
    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;

    std_msgs::Bool flag;
    flag.data = true;

    if(flag.data)
        JustinaKnowledge::addUpdateKnownLoc("before_simul");

    std::vector<std::string> tokens;
    std::string str = responseMsg.params;
    split(tokens, str, is_any_of(" "));

    responseMsg.successful = 1;

    if(tokens[1] == "False"){
        flag.data = false;
        JustinaNavigation::getClose("before_simul", 10000);
    }

    command_response_pub.publish(responseMsg);
    simulated_pub.publish(flag);

}

void callbackSetCubePosition(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
    std::cout << testPrompt << "-------- Command Set Cube Simul Posotion ----------" << std::endl;
    std::cout << "name: " << msg->name << std::endl;
    std::cout << "params: " << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;
    
    std::vector<std::string> tokens;
    std::string str = responseMsg.params;
    split(tokens, str, is_any_of(" "));

    std::vector<std::string> block;
    split(block, tokens[0], is_any_of("_"));
    

    JustinaKnowledge::addUpdateObjectViz(block[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                     atof(tokens[1].c_str()),atof(tokens[2].c_str()), atof(tokens[3].c_str()),
                     0.0, 0.0, 0.0, "map", "map");

    responseMsg.successful = 1;
    command_response_pub.publish(responseMsg);

}

void callbackUpdateStack(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
    std::cout << testPrompt << "------------ Command Update Stacks " << std::endl;
    std::cout << "name: " << msg->name << std::endl;
    std::cout << "params: " << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;

        //JustinaTasks::sayAndSyncNavigateToLoc("table", 120000);

        JustinaManip::hdGoTo(0, -0.9, 5000);
        boost::this_thread::sleep(
                boost::posix_time::milliseconds(1000));
        JustinaTasks::alignWithTable(0.42);
        boost::this_thread::sleep(
                boost::posix_time::milliseconds(1000));
        std::stringstream sss;
        std::stringstream ss;

        vision_msgs::CubesSegmented cubes;
        vision_msgs::Cube cube_aux;
        cube_aux.color = "red";
        cubes.recog_cubes.push_back(cube_aux);
        cube_aux.color = "blue";
        cubes.recog_cubes.push_back(cube_aux);
        cube_aux.color = "green";
        cubes.recog_cubes.push_back(cube_aux);
        cube_aux.color = "yellow";
        cubes.recog_cubes.push_back(cube_aux);
        std::vector<vision_msgs::CubesSegmented> Stacks;
        tf::StampedTransform transform;
        tf::TransformListener* tf_listener = new tf::TransformListener();

        bool fcubes;
        fcubes = JustinaVision::getCubesSeg(cubes);
        std::cout << "GET CUBES: " << fcubes << std::endl;
        Stacks.resize(4);
        if(fcubes) fcubes = JustinaTasks::sortCubes(cubes,Stacks);
        std::cout << "SORT CUBES: " << fcubes << std::endl;
        for(int j=0; j < Stacks.size(); j++){
            std_msgs::String res1;
            sss.str("");
            sss << "(assert (stack_origin";
            for(int k = Stacks.at(j).recog_cubes.size(); k > 0 ;k--){
                ss.str("");
                std::cout << "CUBE: " << Stacks.at(j).recog_cubes.at(k-1).color << std::endl;
                
                JustinaKnowledge::addUpdateObjectViz(Stacks.at(j).recog_cubes.at(k-1).color, Stacks.at(j).recog_cubes.at(k-1).minPoint.x, Stacks.at(j).recog_cubes.at(k-1).minPoint.y, Stacks.at(j).recog_cubes.at(k-1).minPoint.z, Stacks.at(j).recog_cubes.at(k-1).maxPoint.x, Stacks.at(j).recog_cubes.at(k-1).maxPoint.y, Stacks.at(j).recog_cubes.at(k-1).maxPoint.z, Stacks.at(j).recog_cubes.at(k-1).cube_centroid.x, Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y, Stacks.at(j).recog_cubes.at(k-1).cube_centroid.z, Stacks.at(j).recog_cubes.at(k-1).colorRGB.x, Stacks.at(j).recog_cubes.at(k-1).colorRGB.y, Stacks.at(j).recog_cubes.at(k-1).colorRGB.z, "base_link", "map");
                tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
                tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
                tf::Vector3 pos(Stacks.at(j).recog_cubes.at(k-1).cube_centroid.x,
                                Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y,
                                Stacks.at(j).recog_cubes.at(k-1).cube_centroid.z);

                pos = transform * pos;
                ss << "(assert (cmd_insert cube " << Stacks.at(j).recog_cubes.at(k-1).color << "_block "
                   << pos.getX() << " " << pos.getY() << " " << pos.getZ();


                if(Stacks.at(j).recog_cubes.at(k-1).cube_centroid.y > 0)
                       ss << " left 1))";
                else
                        ss << " right 1))";

                sss << " " << Stacks.at(j).recog_cubes.at(k-1).color << "_block";
                
                res1.data = ss.str();
                sendAndRunClips_pub.publish(res1);
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            }
            sss << "))";
            res1.data = sss.str();
            sendAndRunClips_pub.publish(res1);
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }

    
    responseMsg.successful = 1;
    command_response_pub.publish(responseMsg);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "cubes_challenge_test");
    ros::NodeHandle n;

    srvCltWhatSee = n.serviceClient<knowledge_msgs::planning_cmd>(
            "/planning_clips/what_see");
    ros::Subscriber subCmdWorld = n.subscribe(
            "/planning_clips/cmd_world", 1, callbackCmdWorld);
    ros::Subscriber subCmdDescribe = n.subscribe(
            "/planning_clips/cmd_describe", 1, callbackCmdDescribe);
    ros::Subscriber subCmdTakeOrder = n.subscribe(
            "/planning_clips/cmd_order", 1, callbackCmdTakeOrder);
    ros::Subscriber subCmdSpeech = n.subscribe(
            "/planning_clips/cmd_speech", 1, callbackCmdSpeech);
    ros::Subscriber subCmdInterpret = n.subscribe(
            "/planning_clips/cmd_int", 1, callbackCmdInterpret);
    ros::Subscriber subCmdConfirmation = n.subscribe(
            "/planning_clips/cmd_conf", 1, callbackCmdConfirmation);
    ros::Subscriber subCmdGetTasks = n.subscribe(
            "/planning_clips/cmd_task", 1, callbackCmdGetTasks);
    ros::Subscriber subCmdExplain = n.subscribe(
            "/planning_clips/cmd_explain", 1,
            callbackCmdExplainThePlan);
    ros::Subscriber subCmdWhere = n.subscribe(
            "/planning_clips/cmd_where", 1, callbackCmdWhere);
    ros::Subscriber subCmdDisponible = n.subscribe(
            "/planning_clips/cmd_disp", 1, callbackCmdDisponible);
    ros::Subscriber subCmdHappen = n.subscribe(
            "/planning_clips/cmd_happen", 1, callbackCmdHappen);

    ros::Subscriber subCmdNavigation = n.subscribe(
            "/planning_clips/cmd_goto", 1, callbackCmdNavigation);
    ros::Subscriber subCmdAnswer = n.subscribe(
            "/planning_clips/cmd_answer", 1, callbackCmdAnswer);
    ros::Subscriber subCmdFindObject = n.subscribe(
            "/planning_clips/cmd_find_object", 1,
            callbackCmdFindObject);
    ros::Subscriber subCmdAskFor = n.subscribe(
            "/planning_clips/cmd_ask_for", 1, callbackAskFor);
    ros::Subscriber subCmdStatusObject = n.subscribe(
            "/planning_clips/cmd_status_object", 1,
            callbackStatusObject);
    ros::Subscriber subCmdMoveActuator = n.subscribe(
            "/planning_clips/cmd_move_actuator", 1,
            callbackMoveActuator);
    ros::Subscriber subCmdDrop = n.subscribe(
            "/planning_clips/cmd_drop", 1, callbackDrop);
    ros::Subscriber subCmdUnknown = n.subscribe(
            "/planning_clips/cmd_unknown", 1, callbackUnknown);
    
    ros::Subscriber subCmdReviewStack = n.subscribe(
            "/planning_clips/cmd_rstack", 1, callbackReviewStack);
	
    ros::Subscriber subSpeechGenerator = n.subscribe(
            "/planning_clips/cmd_speech_generator", 1, callbackCmdSpeechGenerator);
    ros::Subscriber subCmdMakeBacktraking = n.subscribe(
            "/planning_clips/cmd_mbt", 1, callbackCmdMakeBacktraking);
    ros::Subscriber subCmdEnableSimul = n.subscribe(
            "/planning_clips/cmd_enable_simul", 1, callbackEnableSimul);
    ros::Subscriber subCmdUpdateStack = n.subscribe(
            "/planning_clips/cmd_up_stack", 1, callbackUpdateStack);
    ros::Subscriber subCmdResetCubePos = n.subscribe(
            "/planning_clips/cmd_reset_cube_pos", 1, callbackSetCubePosition);
    ros::Subscriber subCmdTaskConfirmation = n.subscribe(
            "/planning_clips/cmd_task_conf", 1, callbackCmdTaskConfirmation);
    ros::Subscriber subCmdAlignWithPoint = n.subscribe(
            "/planning_clips/cmd_align_point", 1, callbackCmdalignWithPoint);

    srvCltGetTasks = n.serviceClient<knowledge_msgs::planning_cmd>(
            "/planning_clips/get_task");
    srvCltInterpreter = n.serviceClient<knowledge_msgs::planning_cmd>(
            "/planning_clips/interpreter_open");
    srvCltWaitConfirmation = n.serviceClient<knowledge_msgs::planning_cmd>(
            "/planning_clips/confirmation");
    srvCltWaitForCommand = n.serviceClient<knowledge_msgs::planning_cmd>(
            "/planning_clips/wait_command");
    srvCltAnswer = n.serviceClient<knowledge_msgs::planning_cmd>(
            "/planning_clips/answer");
    srvCltExplain = n.serviceClient<knowledge_msgs::planning_cmd>(
            "/planning_clips/plan_explain");
    srvCltDisponible = n.serviceClient<knowledge_msgs::planning_cmd>(
            "/planning_clips/disponible");

    command_response_pub = n.advertise<knowledge_msgs::PlanningCmdClips>(
            "/planning_clips/command_response", 1);

    sendAndRunClips_pub = n.advertise<std_msgs::String>(
            "/planning_clips/command_sendAndRunCLIPS", 1);

    simulated_pub = n.advertise<std_msgs::Bool>(
            "/simulated", 1);

    std::string locationsFilePath = "";
    for (int i = 0; i < argc; i++) {
        std::string strParam(argv[i]);
        if (strParam.compare("-f") == 0)
            locationsFilePath = argv[++i];
    }

    validCommandsStop.push_back("stop follow me");

    JustinaHRI::setNodeHandle(&n);
    JustinaHardware::setNodeHandle(&n);
    JustinaKnowledge::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaRepresentation::setNodeHandle(&n);

    JustinaHRI::setInputDevice(JustinaHRI::KINECT);
    //JustinaHRI::setVolumenInputDevice(JustinaHRI::KINECT, 100000);
    //JustinaHRI::setVolumenOutputDevice(JustinaHRI::DEFUALT, 50000);

    JustinaRepresentation::initKDB("/cubes_challenge/challenge.dat", false, 2000);
    ros::Rate rate(10);
    nextState = SM_INIT;
    
    JustinaHRI::setInputDevice(JustinaHRI::KINECT);
    
    validCommandsTake.push_back("place this block on top of the red block in the dining table");
    validCommandsTake.push_back("place this block on top of the green block in the dining table");
    validCommandsTake.push_back("place this block on top of the blue block in the dining table");
    
    validCommandsStop.push_back("here is the car");
    validCommandsStop.push_back("stop follow me");

    while (ros::ok()) {
        switch (nextState) {

            case SM_INIT:
                std::cout << "GPSRTest.->INIT" << std::endl;
                if (startSignalSM) {
                    nextState = SM_SAY_WAIT_FOR_DOOR;
                }
                break;
            case SM_SAY_WAIT_FOR_DOOR:
                std::cout << "GPSRTest.->WAIT DOOR" << std::endl;
                nextState = SM_WAIT_FOR_DOOR;
                break;
            case SM_WAIT_FOR_DOOR:
                std::cout << "GPSRTest.->WAIT FOR DOOR" << std::endl;
                if (!JustinaNavigation::obstacleInFront())
                    nextState = SM_NAVIGATE_TO_THE_INIT;
                break;
            case SM_NAVIGATE_TO_THE_INIT:
                std::cout << "GPSRTest.->NAVIGATE TO THE INIT" << std::endl;
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                JustinaHRI::waitAfterSay("Now I can see that the door is open",4000);
                JustinaNavigation::moveDist(1.0, 4000);
                if (!JustinaTasks::sayAndSyncNavigateToLoc("arena", 120000)) {
                    std::cout << "GPSRTest.->Second try to move" << std::endl;
                    if (!JustinaTasks::sayAndSyncNavigateToLoc("arena", 120000)) {
                        std::cout << "GPSRTest.->Third try to move" << std::endl;
                        if (JustinaTasks::sayAndSyncNavigateToLoc("arena", 120000)) {
                            JustinaHRI::waitAfterSay("Tell me follow me, for start following you", 12000, maxDelayAfterSay);
                            nextState = SM_WAIT_FOR_OPERATOR;
                        }
                    } else {
                        JustinaHRI::waitAfterSay("Tell me follow me, for start following you", 12000, maxDelayAfterSay);
                        nextState = SM_WAIT_FOR_OPERATOR;
                    }
                } else {
                    JustinaHRI::waitAfterSay("Tell me follow me, for start following you", 12000, maxDelayAfterSay);
                    nextState = SM_WAIT_FOR_OPERATOR;
                }
                JustinaHRI::loadGrammarSpeechRecognized("HelpMeCarry.xml");//load the grammar
                JustinaHRI::enableSpeechRecognized(true);//disable recognized speech
                cont_z=0;
                break;
            
            case SM_WAIT_FOR_OPERATOR:
                std::cout << "State machine: SM_WAIT_FOR_OPERATOR" << std::endl;
                if(JustinaHRI::waitForSpecificSentence("follow me" , 15000)){
                    nextState = SM_MEMORIZING_OPERATOR;
                }
                else                    
                    cont_z++;    		

                if(cont_z > 3){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("Please repeat the command", 5000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    cont_z = 0;
                }
                break;


            case SM_MEMORIZING_OPERATOR:
                std::cout << "State machine: SM_MEMORIZING_OPERATOR" << std::endl;
                if(!follow_start){
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    // JustinaHRI::waitAfterSay("Human, please put in front of me", 3000, minDelayAfterSay);
                    JustinaHRI::enableLegFinder(true);
                }
                else
                    JustinaHRI::enableLegFinder(true);    

                nextState=SM_WAIT_FOR_LEGS_FOUND;
                break;

            case SM_WAIT_FOR_LEGS_FOUND:
                std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
                if(JustinaHRI::frontalLegsFound()){
                    if(follow_start){
                        std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("I found you, please walk", 4000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        JustinaHRI::startFollowHuman();
                        ros::spinOnce();
                        rate.sleep();
                        JustinaHRI::startFollowHuman();
                        nextState = SM_FOLLOWING_PHASE;
                    }
                    else{
                        std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("I found you, i will start to follow you human, please walk", 10000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//disable recognized speech
                        JustinaHRI::startFollowHuman();
                        follow_start=true;
                        nextState = SM_FOLLOWING_PHASE;
                    }
                }
                break;

            case SM_FOLLOWING_PHASE:
                std::cout << "State machine: SM_FOLLOWING_PHASE" << std::endl;
                if(JustinaHRI::waitForSpecificSentence(validCommandsStop, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find("here is the car") != std::string::npos || lastRecoSpeech.find("stop follow me") != std::string::npos){
                        /*JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("is the goal location, please tell me robot yes, or robot no", 10000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        JustinaHRI::waitForUserConfirmation(userConfirmation, 5000);
                        if(userConfirmation){
                            JustinaHRI::stopFollowHuman();
                            JustinaHRI::enableLegFinder(false);
                            JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                            JustinaHRI::waitAfterSay("I stopped", 2000, minDelayAfterSay);
                            JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                            nextState = SM_BRING_THE_PYRAMID;
                            cont_z=8;
                            break;
                        }
                        else{
                            JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                            JustinaHRI::waitAfterSay("Ok, please walk", 3000, maxDelayAfterSay);
                            JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                        }*/
                        JustinaHRI::stopFollowHuman();
                        JustinaHRI::enableLegFinder(false);
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay("I stopped", 2000, minDelayAfterSay);
                        nextState = SM_BRING_THE_PYRAMID;
                        cont_z=8;
                        JustinaHRI::loadGrammarSpeechRecognized("order_cubes_challenge.xml");//load the grammar
                        break;
                    }
                }
                if(!JustinaHRI::frontalLegsFound()){
                    std::cout << "State machine: SM_FOLLOWING_PHASE -> Lost human!" << std::endl;
                    JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                    JustinaHRI::waitAfterSay("I lost you, please put in front of me again", 5500, maxDelayAfterSay);
                    JustinaHRI::stopFollowHuman();
                    JustinaHRI::enableLegFinder(false);
                    nextState=SM_MEMORIZING_OPERATOR;
                }        
                break;
            
            case SM_BRING_THE_PYRAMID:
                std::cout << "State machine: SM_BRING_THE_PYRAMID" << std::endl; 
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                if(cont_z > 3){
                    // TODO change the command to what dialog is correct
                    ss.str("");
                    ss << "where do you want, i place the " << block << " block";
                    JustinaHRI::waitAfterSay("", 7000, maxDelayAfterSay);
                    JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                    cont_z=0;
                }
                JustinaHRI::enableSpeechRecognized(true);//enable recognized speech
                cont_z++;
                if(JustinaHRI::waitForSpecificSentence(validCommandsTake, lastRecoSpeech, 7000)){
                    attemptsRecogLoc++;
 
                    if(lastRecoSpeech.find("blue") != std::string::npos){
                        block = "blue";
                        nextState=SM_BRING_THE_PYRAMID_CONF;
                    }
                    
                    if(lastRecoSpeech.find("green") != std::string::npos){
                        block = "green";
                        nextState=SM_BRING_THE_PYRAMID_CONF;
                    }
                    
                    if(lastRecoSpeech.find("red") != std::string::npos){
                        block = "red";
                        nextState=SM_BRING_THE_PYRAMID_CONF;
                    }

                    if(block.compare("") != 0 && nextState == SM_BRING_THE_PYRAMID_CONF){
                        ss.str("");
                        ss << "Do you want me, take this block and place on top of the " << block << " block in the dining table";
                        JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                        JustinaHRI::waitAfterSay(ss.str(), 5000, maxDelayAfterSay);
                        JustinaHRI::enableSpeechRecognized(true);//disable recognized speech
                    }

                }
                break;
            
            case SM_BRING_THE_PYRAMID_CONF:
                std::cout << "State machine: SM_BRING_THE_PYRAMID" << std::endl;
                JustinaHRI::loadGrammarSpeechRecognized("HelpMeCarry.xml");//load the grammar
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                JustinaHRI::waitForUserConfirmation(userConfirmation, 7000);
                attemptsConfLoc++;
                if(userConfirmation)
                    nextState = SM_BRING_THE_PYRAMID_TAKE;
                else if(attemptsConfLoc < MAX_ATTEMPTS_CONF){
                    nextState = SM_BRING_THE_PYRAMID_CONF;
                    cont_z = 8;
                }
                else
                    nextState = SM_BRING_THE_PYRAMID_TAKE;
                break;
            
            case SM_BRING_THE_PYRAMID_TAKE:    
                std::cout << "State machine: SM_BRING_THE_PYRAMID_TAKE" << std::endl;
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                JustinaManip::startHdGoTo(0.0, 0.0);
                JustinaHRI::waitAfterSay("Please put in front of me to see your face", 3000);
                ros::Duration(1.0).sleep();
                while(!recog && contChances < 3)
                {
                    faces = recognizeFaces (10000, recog);
                    JustinaVision::startFaceRecognition(false);
                    contChances++;
                }

                if(faces.recog_faces.size()==0)
                {
                    JustinaHRI::say("Sorry, I cannot see anybody in front of me");
                    ros::Duration(1.5).sleep();
                    JustinaHRI::say("i can not take the pyramid from your hand but i will take it if you put in my gripper");
                    ros::Duration(1.0).sleep();
                    JustinaTasks::detectObjectInGripper("block", true, 20000);
                    withLeftArm = true;
                    ros::Duration(1.0).sleep();
                }
                else{
                    JustinaManip::startHdGoTo(0.0, -0.4);
                    JustinaHRI::say("Ready, now wait for the next instruction");
                    ros::Duration(2.0).sleep();
                     if(JustinaTasks::graspBagHand(faces.recog_faces[0].face_centroid, withLeftArm))
                        std::cout << "test succesfully" << std::endl;
                    else
                    {
                        JustinaHRI::say("sorry i can not see your hand");
                        ros::Duration(1.0).sleep();
                        JustinaHRI::say("i can not take the pyramid from your hand but i will take it if you put in my gripper");
                        ros::Duration(1.0).sleep();
                        JustinaTasks::detectObjectInGripper("block", true, 7000);
                        withLeftArm = true;
                        ros::Duration(1.0).sleep();
                    }
                }

                //JustinaTasks::detectBagInFront(true, 20000);

                ss.str("");
                ss << "Ok human, I will go to the dining table"; ;
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                //JustinaManip::startTorsoGoTo(0.3, 0.0, 0.0);
                nextState=SM_NAVIGATE_TO_THE_LOCATION; 
                break;

            case SM_NAVIGATE_TO_THE_LOCATION:
                std::cout << "GPSRTest.->NAVIGATE DINING TABLE" << std::endl;
                JustinaNavigation::getClose("dining_table", 120000);
                JustinaHRI::enableSpeechRecognized(false);//disable recognized speech
                JustinaHRI::loadGrammarSpeechRecognized("final_cubes_challenge.xml");//load the grammar
                JustinaHRI::enableSpeechRecognized(true);//disable recognized speech
                nextState = SM_PLACE_BLOCK;
                break;
            case SM_PLACE_BLOCK:
                std::cout << "GPSRTest.->PLACE BLOCK" << std::endl;
                JustinaKnowledge::addUpdateObjectViz(block, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0.0, 0.0, "base_link", "map");
                JustinaTasks::placeBlockOnBlock(0.09, withLeftArm, block);
                nextState = SM_SEND_INIT_CLIPS;
                break;
            case SM_SEND_INIT_CLIPS:
                JustinaVision::startQRReader();
                initMsg.successful = 0;
                runSMCLIPS = true;
                command_response_pub.publish(initMsg);
                nextState = SM_LOOK_CUBES_CONFIG;
                break;
            case SM_LOOK_CUBES_CONFIG:
                //JustinaHRI::fakeSpeechRecognized("look at the cubes configuration");
                JustinaHRI::enableSpeechRecognized(true);//disable recognized speech
                nextState = SM_RUN_SM_CLIPS;
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
