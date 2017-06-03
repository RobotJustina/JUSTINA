#include "justina_tools/JustinaRepresentation.h"

ros::Publisher * JustinaRepresentation::command_runCLIPS;
ros::Publisher * JustinaRepresentation::command_resetCLIPS;
ros::Publisher * JustinaRepresentation::command_factCLIPS;
ros::Publisher * JustinaRepresentation::command_ruleCLIPS;
ros::Publisher * JustinaRepresentation::command_agendaCLIPS;
ros::Publisher * JustinaRepresentation::command_sendCLIPS;
ros::Publisher * JustinaRepresentation::command_loadCLIPS;
ros::ServiceClient * JustinaRepresentation::cliSpechInterpretation;
ros::ServiceClient * JustinaRepresentation::cliStringInterpretation;

JustinaRepresentation::~JustinaRepresentation(){
    delete command_runCLIPS;
    delete command_resetCLIPS;
    delete command_factCLIPS;
    delete command_ruleCLIPS;
    delete command_agendaCLIPS;
    delete command_sendCLIPS;
    delete command_loadCLIPS;
    delete cliSpechInterpretation;
    delete cliStringInterpretation;
}

void JustinaRepresentation::setNodeHandle(ros::NodeHandle * nh) {
    command_runCLIPS = new ros::Publisher(nh->advertise<std_msgs::Bool>("/planning_clips/command_runCLIPS", 1));
    command_resetCLIPS = new ros::Publisher(nh->advertise<std_msgs::Bool>("/planning_clips/command_resetCLIPS", 1));
    command_factCLIPS = new ros::Publisher(nh->advertise<std_msgs::Bool>("/planning_clips/command_factCLIPS", 1));
    command_ruleCLIPS = new ros::Publisher(nh->advertise<std_msgs::Bool>("/planning_clips/command_ruleCLIPS", 1));
    command_agendaCLIPS = new ros::Publisher(nh->advertise<std_msgs::Bool>("/planning_clips/command_agendaCLIPS", 1));
    command_sendCLIPS = new ros::Publisher(nh->advertise<std_msgs::String>("/planning_clips/command_sendCLIPS", 1));
    command_loadCLIPS = new ros::Publisher(nh->advertise<std_msgs::String>("/planning_clips/command_loadCLIPS", 1));
    cliSpechInterpretation = new ros::ServiceClient(nh->serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/interpreter"));
    cliStringInterpretation = new ros::ServiceClient(nh->serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/interpreter"));
}

void JustinaRepresentation::runCLIPS(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    command_runCLIPS->publish(msg);
}

void JustinaRepresentation::resetCLIPS(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    command_resetCLIPS->publish(msg);
}

void JustinaRepresentation::factCLIPS(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    command_factCLIPS->publish(msg);
}

void JustinaRepresentation::ruleCLIPS(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    command_ruleCLIPS->publish(msg);
}

void JustinaRepresentation::agendaCLIPS(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    command_agendaCLIPS->publish(msg);
}

void JustinaRepresentation::sendCLIPS(std::string command){
    std_msgs::String msg;
    msg.data = command;
    command_sendCLIPS->publish(msg);
}

void JustinaRepresentation::loadCLIPS(std::string file)
{
    std_msgs::String msg;
    msg.data = file;
    command_loadCLIPS->publish(msg);
}

void JustinaRepresentation::getLocations(std::string path, std::map<std::string, std::vector<std::string> > &locations)
{
    std::cout << "Ltm.->Loading known locations from " << path << std::endl;
    std::vector<std::string> lines;
    std::ifstream file(path.c_str());
    std::string tempStr;
    while (std::getline(file, tempStr))
        lines.push_back(tempStr);

    //Extraction of lines without comments
    for (size_t i = 0; i < lines.size(); i++) {
        size_t idx = lines[i].find("//");
        if (idx != std::string::npos)
            lines[i] = lines[i].substr(0, idx);
    }

    locations.clear();
    float locX, locY, locAngle;
    bool parseSuccess;
    for (size_t i = 0; i < lines.size(); i++) {
        //std::cout << "Ltm.->Parsing line: " << lines[i] << std::endl;
        std::vector<std::string> parts;
        std::vector<std::string> loc;
        boost::split(parts, lines[i], boost::is_any_of(" ,\t"),
                boost::token_compress_on);
        if (parts.size() < 3)
            continue;
        //std::cout << "Ltm.->Parsing splitted line: " << lines[i] << std::endl;
        parseSuccess = true;
        loc.push_back(parts[0]);
        loc.push_back(parts[2]);
        loc.push_back(parts[3]);

        if (parseSuccess) {
            //std::cout << "LOCATIONS" << parts[0] << std::endl;
            locations[parts[1]] = loc;
        }
    }
    std::cout << "Ltm.->Total number of known locations: " << locations.size() << std::endl;
    for (std::map<std::string, std::vector<std::string> >::iterator it = locations.begin(); it != locations.end(); it++) {
        std::cout << "Ltm.->Location " << it->first << " " << it->second[0] << " " << it->second[1];
        if (it->second.size() > 2)
            std::cout << " " << it->second[2];
        std::cout << std::endl;
    }
    if (locations.size() < 1)
        std::cout << "Ltm.->WARNING: Cannot load known locations from file: "
            << path << ". There are no known locations." << std::endl;

}

void JustinaRepresentation::getObjects(std::string path, std::map<std::string, std::vector<std::string> > &objects)
{
    std::cout << "Ltm.->Loading known locations from " << path << std::endl;
    std::vector<std::string> lines;
    std::ifstream file(path.c_str());
    std::string tempStr;
    while (std::getline(file, tempStr))
        lines.push_back(tempStr);

    //Extraction of lines without comments
    for (size_t i = 0; i < lines.size(); i++) {
        size_t idx = lines[i].find("//");
        if (idx != std::string::npos)
            lines[i] = lines[i].substr(0, idx);
    }

    objects.clear();
    bool parseSuccess;
    for (size_t i = 0; i < lines.size(); i++) {
        //std::cout << "Ltm.->Parsing line: " << lines[i] << std::endl;
        std::vector<std::string> parts;
        std::vector<std::string> loc;
        boost::split(parts, lines[i], boost::is_any_of(" ,\t"),
                boost::token_compress_on);
        if (parts.size() < 3)
            continue;
        //std::cout << "Ltm.->Parsing splitted line: " << lines[i] << std::endl;
        parseSuccess = true;
        loc.push_back(parts[1]);
        loc.push_back(parts[2]);
        loc.push_back(parts[3]);
        loc.push_back(parts[4]);
        loc.push_back(parts[5]);
        loc.push_back(parts[6]);
        loc.push_back(parts[7]);

        if (parseSuccess) {
            //std::cout << "LOCATIONS" << parts[0] << std::endl;
            objects[parts[0]] = loc;
        }
    }
    std::cout << "Ltm.->Total number of known locations: " << objects.size() << std::endl;
    for (std::map<std::string, std::vector<std::string> >::iterator it = objects.begin(); it != objects.end(); it++) {
        std::cout << "Ltm.->Location " << it->first << " " << it->second[0] << " " << it->second[1];
        if (it->second.size() > 2)
            std::cout << " " << it->second[2];
        std::cout << std::endl;
    }
    if (objects.size() < 1)
        std::cout << "Ltm.->WARNING: Cannot load known locations from file: "
            << path << ". There are no known locations." << std::endl;

}

void JustinaRepresentation::addLocations(std::map<std::string, std::vector<std::string> >& locations, std::string name, std::vector<std::string> values)
{
    std::map<std::string, std::vector<std::string> >::iterator it;

    it = locations.find(name);
    if (it != locations.end()) {
        it->second[0] = values.at(0);
        it->second[1] = values.at(1);
        it->second[2] = values.at(2);
    }
    else
        locations[name] = values;

}

void JustinaRepresentation::addObjects(std::map<std::string, std::vector<std::string> >& objects, std::string name, std::vector<std::string> values)
{
    std::map<std::string, std::vector<std::string> >::iterator it;

    it = objects.find(name);
    if (it != objects.end()) {
        it->second[0] = values.at(0);
        it->second[1] = values.at(1);
        it->second[2] = values.at(2);
        it->second[3] = values.at(3);
        it->second[4] = values.at(4);
        it->second[5] = values.at(5);
        it->second[6] = values.at(6);
    }
    else
        objects[name] = values;

}

bool JustinaRepresentation::speachInterpretation(){
    bool success = ros::service::waitForService("/planning_clips/interpreter",
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
}

bool JustinaRepresentation::stringInterpretation(){
}

