#include "MvnPln.h"

MvnPln::MvnPln()
{
}

MvnPln::~MvnPln()
{
}

void initROSConnection(ros::NodeHandle* nh)
{
    this->nh = nh;
    this->subGetCloseLoc = nh->subscribe("/navigation/mvn_pln/get_close_loc", 1, &MvnPln::callbackGetCloseLoc, this);
    this->subGetCloseXYA = nh->subscribe("/navigation/mvn_pln/get_close_xya", 1, &MvnPln::callbackGetCloseXYA, this);
    this->pubGoalReached = nh->advertise<std_msgs::Bool>("/navigation/goal_reached", 1);
}

void spin()
{
}

bool MvnPln::GetClose(std::string location)
{
}

bool MvnPln::GetClose(float goalX, float goalY)
{
    
}

bool MvnPln::GetClose(float goalX, float goalY, float goalTheta)
{
}

void MvnPln::callbackGetCloseLoc(const std_msgs::String::ConstPtr& msg)
{
}

void MvnPln::callbackGetCloseXYA(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
}
