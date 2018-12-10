#include "ros/ros.h"

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
#include "justina_tools/JustinaRepresentation.h"

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
knowledge_msgs::PlanningCmdClips initMsg;

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
    std::string str = msg->params;
    split(tokens, str, is_any_of(" "));

    responseMsg.successful = 1;

    if (tokens[0] == "nil" || tokens[0] == "droped") {

        bool success;
        success = ros::service::waitForService(
                "/planning_clips/disponible", 5000);
        if (success) {
            std::cout << "------------- No Disponible: ------------------ "
                << std::endl;

            if (tokens[2] == "found")
                JustinaTasks::sayAndSyncNavigateToLoc("dining_room", 120000, false);

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
                        vision_msgs::VisionFaceObjects lastRecognizedFaces = JustinaVision::getFaceRecognition(tokens[3]);
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
                        vision_msgs::VisionFaceObjects lastRecognizedFaces;
                        if(tokens[3].compare("") == 0)
                            lastRecognizedFaces = JustinaVision::getFaces();
                        else
                            lastRecognizedFaces = JustinaVision::getFaceRecognition(tokens[3]);
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
        if (srvCltWaitConfirmation.call(srv)) {
            std::cout << "Response of confirmation:" << std::endl;
            std::cout << "Success:" << (long int) srv.response.success
                << std::endl;
            std::cout << "Args:" << srv.response.args << std::endl;
            if (srv.response.success)
                JustinaHRI::waitAfterSay("Do you want me explain the plan", 1500);
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
        JustinaVision::startFaceRecognition(true);
        responseMsg.params = srv.response.args;
        responseMsg.successful = srv.response.success;

        if (srv.response.args == "what_see_obj") {

            //if(objectsids.size()>0)
            //	objectsids.erase(objectsids.begin());
            //boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
            JustinaTasks::sayAndSyncNavigateToLoc("table", 120000);

            JustinaHRI::waitAfterSay(
                    "I am looking for objects on the table", 1500);
            JustinaManip::hdGoTo(0, -0.9, 5000);
            boost::this_thread::sleep(
                    boost::posix_time::milliseconds(1000));
            JustinaTasks::alignWithTable(0.42);
            boost::this_thread::sleep(
                    boost::posix_time::milliseconds(1000));

            objectsids.clear();

            std::map<std::string, int> countObj;
            countObj["soup"] = 0;
            countObj["sugar"] = 0;
            countObj["milk"] = 0;
            countObj["juice"] = 0;

            for(float headPanTurn = -0.3; ros::ok() && headPanTurn <= 0.3; headPanTurn+=0.3){
                JustinaManip::startHdGoTo(headPanTurn, -0.9);
                JustinaManip::waitForHdGoalReached(3000);
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
                            if (it != countObj.end())
                                it->second = it->second + 1;
                        }
                    }
                }
            }
            JustinaManip::hdGoTo(0, 0.0, 5000);
            responseObject.successful = 1;

            int objRecog = 0;
            for (std::map<std::string, int>::iterator it = countObj.begin();
                    it != countObj.end(); ++it) {
                if (it->second > 10)
                    objRecog++;
            }

            for (std::map<std::string, int>::iterator it = countObj.begin();
                    it != countObj.end(); ++it) {
                std::stringstream ssr;
                if (it->second > 10) {
                    ssr << it->first << " table";
                    responseObject.params = ssr.str();
                    command_response_pub.publish(responseObject);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                    ros::spinOnce();
                    objectsids.push_back(it->first);						
                } else {
                    ssr << it->first << " nil";
                    responseObject.params = ssr.str();
                    command_response_pub.publish(responseObject);
                }
            }

            std::stringstream ss;	
            if(objectsids.size() > 0){
                ss << "I have found ";
                for(int i = 0; i < objectsids.size(); i++){
                    if( i < objectsids.size() - 1 || objectsids.size() == 1)
                        ss << "the " << objectsids[i] << ", ";
                    else
                        ss << "and the " << objectsids[i];
                }	
            }
            else
                ss << "I have not found objects ";

            ss << ", on the table";
            JustinaHRI::waitAfterSay(ss.str(), 1500);
            JustinaNavigation::moveDistAngle(-0.2, 0.0, 3000);
            JustinaTasks::sayAndSyncNavigateToLoc("dining_room", 120000, false);
        }				///termina recog objects

        if (srv.response.args == "what_see_person" || srv.response.args == "what_see_obj" ) {
            JustinaHRI::waitAfterSay("I am ready for another question",
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
    std::string str = responseMsg.params;
    split(tokens, str, is_any_of(" "));
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
        } else {
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
    std::string str = responseMsg.params;
    split(tokens, str, is_any_of(" "));
    bool armFlag = true;
    std::stringstream ss;

    bool success = ros::service::waitForService("spg_say", 5000);
	if(tokens[4] == "false")
			armFlag = false;
    success = success
        & JustinaTasks::moveActuatorToGrasp(atof(tokens[1].c_str()),
                atof(tokens[2].c_str()), atof(tokens[3].c_str()), armFlag,
                tokens[0], true);
    if (success)
        responseMsg.successful = 1;
	else{
		ss.str("");
		ss << "I did not grasp the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(), 100000);
		responseMsg.successful = 0;
	}
        //responseMsg.successful = 1;

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

    bool success = JustinaTasks::dropObject(tokens[1], armFlag, 30000);

    if (success)
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
        success = JustinaTasks::sayAndSyncNavigateToLoc(tokens[1], 120000, goToTable);
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
    JustinaRepresentation::setNodeHandle(&n);

    JustinaHRI::setInputDevice(JustinaHRI::KINECT);
    JustinaHRI::setVolumenInputDevice(JustinaHRI::KINECT, 100000);
    JustinaHRI::setVolumenOutputDevice(JustinaHRI::DEFUALT, 50000);

    JustinaRepresentation::initKDB("/open_challenge/challenge.dat", false, 2000);
    ros::Rate rate(10);
    state = SM_INIT;

    while (ros::ok()) {
        switch (state) {
            case SM_INIT:
                if (startSignalSM) {
                    JustinaHRI::waitAfterSay(
                            "Hellow my name is Justina, I'm ready for the open challange test",
                            2000);
                    float currx, curry, currentTheta;
                    JustinaNavigation::getRobotPose(currx, curry, currentTheta);
                    JustinaKnowledge::addUpdateKnownLoc("dining_room", currx, curry, currentTheta);
                    JustinaKnowledge::addUpdateKnownLoc("table", currx, curry, currentTheta + 1.5708);
                    JustinaHRI::loadGrammarSpeechRecognized("open_chalenge.xml");//load the grammar
                    JustinaHRI::enableSpeechRecognized(true);//Enable recognized speech
                    state = SM_NAVIGATE_TO_THE_LOCATION;
                }
                std::cout << "state:" << state << std::endl;
                break;
            case SM_NAVIGATE_TO_THE_LOCATION:
                std::cout << "GPSRTest.->First try to move" << std::endl;
                JustinaHRI::waitAfterSay("I'am ready for user questions.", 1500);
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
