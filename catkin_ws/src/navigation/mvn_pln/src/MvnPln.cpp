#include "MvnPln.h"

MvnPln::MvnPln()
{
    this->newTask = false;
}

MvnPln::~MvnPln()
{
}

void MvnPln::initROSConnection(ros::NodeHandle* nh)
{
    this->nh = nh;
    //Publishers and subscribers for the commands executed by this node
    this->subGetCloseLoc = nh->subscribe("/navigation/mvn_pln/get_close_loc", 1, &MvnPln::callbackGetCloseLoc, this);
    this->subGetCloseXYA = nh->subscribe("/navigation/mvn_pln/get_close_xya", 1, &MvnPln::callbackGetCloseXYA, this);
    this->pubGoalReached = nh->advertise<std_msgs::Bool>("/navigation/goal_reached", 1);
}

bool MvnPln::loadKnownLocations(std::string path)
{
    std::cout << "MvnPln.->Loading known locations from " << path << std::endl;
    std::vector<std::string> lines;
    std::ifstream file(path.c_str());
    std::string tempStr;
    while(std::getline(file, tempStr))
        lines.push_back(tempStr);

    //Extraction of lines without comments
    for(size_t i=0; i< lines.size(); i++)
    {
        size_t idx = lines[i].find("//");
        if(idx!= std::string::npos)
            lines[i] = lines[i].substr(0, idx);
    }

    this->locations.clear();
    float locX, locY, locAngle;
    bool parseSuccess;
    for(size_t i=0; i<lines.size(); i++)
    {
        //std::cout << "MvnPln.->Parsing line: " << lines[i] << std::endl;
        std::vector<std::string> parts;
        std::vector<float> loc;
        boost::split(parts, lines[i], boost::is_any_of(" ,\t"), boost::token_compress_on);
        if(parts.size() < 3)
            continue;
        //std::cout << "MvnPln.->Parsing splitted line: " << lines[i] << std::endl;
        parseSuccess = true;
        std::stringstream ssX(parts[1]);
        if(!(ssX >> locX)) parseSuccess = false;
        std::stringstream ssY(parts[2]);
        if(!(ssY >> locY)) parseSuccess = false;
        loc.push_back(locX);
        loc.push_back(locY);
        if(parts.size() >= 4)
        {
            std::stringstream ssAngle(parts[3]);
            if(!(ssAngle >> locAngle)) parseSuccess = false;
            loc.push_back(locAngle);
        }

        if(parseSuccess)
        {
            this->locations[parts[0]] = loc;
        }
    }
    std::cout << "MvnPln.->Total number of known locations: " << this-locations.size() << std::endl;
    return this->locations.size() > 0;
}

void MvnPln::spin()
{
    ros::Rate loop(10);
    int currentState = SM_INIT;

    while(ros::ok())
    {
        switch(currentState)
        {
        case SM_INIT:
            std::cout << "MvnPln.->Current state: " << currentState << "Waiting for new task..." << std::endl;
            currentState = SM_WAITING_FOR_NEW_TASK;
            break;
        case SM_WAITING_FOR_NEW_TASK:
            if(this->newTask)
            {
                std::cout << "MvnPln.->New task received..." << std::endl;
                currentState = SM_CALCULATE_PATH;
            }
            break;
        case SM_CALCULATE_PATH:
            break;
        }
        ros::spinOnce();
        loop.sleep();
    }
}

void MvnPln::callbackGetCloseLoc(const std_msgs::String::ConstPtr& msg)
{
}

void MvnPln::callbackGetCloseXYA(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //If msg has two values, the robot will try to reach the goal point without correcting the final angle
    //If it has three values, the third one will be the final desired angle.
}
