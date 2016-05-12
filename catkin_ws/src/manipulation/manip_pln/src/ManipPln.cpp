#include "ManipPln.h"

ManipPln::ManipPln()
{
}

ManipPln::~ManipPln()
{
}

void ManipPln::setNodeHandle(ros::NodeHandle* n)
{
    this->nh = n;
    this->pubLaGoalReached = nh->advertise<std_msgs::Bool>("/manipulation/la_goal_reached", 1);
    this->pubRaGoalReached = nh->advertise<std_msgs::Bool>("/manipulation/ra_goal_reached", 1);
    this->pubHdGoalReached = nh->advertise<std_msgs::Bool>("/manipulation/head_goal_reached", 1);
    this->subLaGoalPose = nh->subscribe("/manipulation/la_control/goal_pose", 1, &ManipPln::callbackLaGoalPose, this);
    this->subRaGoalPose = nh->subscribe("/manipulation/ra_control/goal_pose", 1, &ManipPln::callbackRaGoalPose, this);
    this->subHdGoalPose = nh->subscribe("/manipulation/hd_control/goal_pose", 1, &ManipPln::callbackHdGoalPose, this);
    this->subLaCurrentPose = nh->subscribe("/hardware/left_arm/current_pose", 1, &ManipPln::callbackLaCurrentPose, this);
    this->subRaCurrentPose = nh->subscribe("/hardware/right_arm/current_pose", 1, &ManipPln::callbackRaCurrentPose, this);
    this->subHdCurrentPose = nh->subscribe("/hardware/head/current_pose", 1, &ManipPln::callbackHdCurrentPose, this);
    this->pubLaGoalPose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1);
    this->pubRaGoalPose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 1);
    this->pubHdGoalPose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    this->pubLaGoalTorque = nh->advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_torque", 1);
    this->pubRaGoalTorque = nh->advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_torque", 1);
    this->pubHdGoalTorque = nh->advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_torque", 1);
}

bool ManipPln::loadKnownPosesAndMovs(std::string directory)
{
}

void ManipPln::spin()
{
    ros::Rate loop(10);
    std_msgs::Bool msgLaGoalReached;
    std_msgs::Bool msgRaGoalReached;
    std_msgs::Bool msgHdGoalReached;
    std_msgs::Float32MultiArray msgLaGoalPose;
    std_msgs::Float32MultiArray msgRaGoalPose;
    std_msgs::Float32MultiArray msgHdGoalPose;

    while(ros::ok())
    {
        if(this->laNewGoal)
        {
            float error = this->calculateError(this->laCurrentPose, this->laGoalPose);
            if(error < 0.05)
            {
                msgLaGoalReached.data = true;
                pubLaGoalReached.publish(msgLaGoalReached);
                this->laNewGoal = false;
            }
            else
            {
                msgLaGoalPose.data = this->laGoalPose;
                pubLaGoalPose.publish(msgLaGoalPose);
            }
        }
        if(this->raNewGoal)
        {
            float error = this->calculateError(this->raCurrentPose, this->raGoalPose);
            if(error < 0.05)
            {
                msgRaGoalReached.data = true;
                pubRaGoalReached.publish(msgRaGoalReached);
                this->raNewGoal = false;
            }
            else
            {
                msgRaGoalPose.data = this->raGoalPose;
                pubRaGoalPose.publish(msgRaGoalPose);
            }
        }
        if(this->hdNewGoal)
        {
            float error = this->calculateError(this->hdCurrentPose, this->hdGoalPose);
            if(error < 0.05)
            {
                msgHdGoalReached.data = true;
                pubHdGoalReached.publish(msgHdGoalReached);
                this->hdNewGoal = false;
            }
            else
            {
                msgHdGoalPose.data = this->hdGoalPose;
                pubHdGoalPose.publish(msgHdGoalPose);
            }
        }
        ros::spinOnce();
        loop.sleep();
    }
}

float ManipPln::calculateError(std::vector<float>& v1, std::vector<float>& v2)
{
    float max = 0;
    for(int i=0; i < 7; i++)
    {
        float temp = fabs(v1[i] - v2[i]);
        if(temp > max)
            max = temp;
    }
    return max;
}

void ManipPln::callbackLaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "La pose received" << std::endl;
    this->laCurrentPose = msg->data;
}

void ManipPln::callbackRaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "Ra pose received" << std::endl;
    this->raCurrentPose = msg->data;
}

void ManipPln::callbackHdCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "Hd pose received" << std::endl;
    this->hdCurrentPose = msg->data;
}

void ManipPln::callbackLaGoalPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 7)
    {
        std::cout << "ManipPln.->LaGoalPose must have 7 values. " << std::endl;
        return;
    }
    std::cout << "ManipPln.->Left Arm goal pose: ";
    for(int i=0; i< 7; i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;

    this->laGoalPose = msg->data;
    this->laNewGoal = true;
}

void ManipPln::callbackRaGoalPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 7)
    {
        std::cout << "ManipPln.->RaGoalPose must have 7 values. " << std::endl;
        return;
    }
    std::cout << "ManipPln.->Right Arm goal pose: ";
    for(int i=0; i< 7; i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;

    this->raGoalPose = msg->data;
    this->raNewGoal = true;
}

void ManipPln::callbackHdGoalPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 2)
    {
        std::cout << "ManipPln.->HeadGoalPose must have 7 values. " << std::endl;
        return;
    }
    std::cout << "ManipPln.->Head goal pose: ";
    for(int i=0; i< 2; i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;

    this->hdGoalPose = msg->data;
    this->hdNewGoal = true;
}
