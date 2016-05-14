#include "ManipPln.h"

ManipPln::ManipPln()
{
    this->laNewGoal = false;
    this->raNewGoal = false;
    this->hdNewGoal = false;
}

ManipPln::~ManipPln()
{
}

void ManipPln::setNodeHandle(ros::NodeHandle* n)
{
    std::cout << "ManipPln.->Setting ros node..." << std::endl;
    this->nh = n;
    //Publishers for indicating that a goal pose has been reached
    this->pubLaGoalReached = nh->advertise<std_msgs::Bool>("/manipulation/la_goal_reached", 1);
    this->pubRaGoalReached = nh->advertise<std_msgs::Bool>("/manipulation/ra_goal_reached", 1);
    this->pubHdGoalReached = nh->advertise<std_msgs::Bool>("/manipulation/head_goal_reached", 1);
    //Subscribers for the commands executed by this node
    this->subLaGoToAngles = nh->subscribe("/manipulation/manip_pln/la_goto_angles", 1, &ManipPln::callbackLaGoToAngles, this);
    this->subRaGoToAngles = nh->subscribe("/manipulation/manip_pln/ra_goto_angles", 1, &ManipPln::callbackRaGoToAngles, this);
    this->subHdGoToAngles = nh->subscribe("/manipulation/manip_pln/hd_goto_angles", 1, &ManipPln::callbackHdGoToAngles, this);
    this->subLaGoToPoseWrtArm = nh->subscribe("/manipulation/manip_pln/la_pose_wrt_arm", 1, &ManipPln::callbackLaGoToPoseWrtArm, this);
    this->subRaGoToPoseWrtArm = nh->subscribe("/manipulation/manip_pln/ra_pose_wrt_arm", 1, &ManipPln::callbackRaGoToPoseWrtArm, this);
    this->subLaGoToPoseWrtRobot = nh->subscribe("/manipulation/manip_pln/la_pose_wrt_robot", 1, &ManipPln::callbackLaGoToPoseWrtRobot, this);
    this->subRaGoToPoseWrtRobot = nh->subscribe("/manipulation/manip_pln/ra_pose_wrt_robot", 1, &ManipPln::callbackRaGoToPoseWrtRobot, this);
    this->subLaGoToLoc = nh->subscribe("/manipulation/manip_pln/la_goto_loc", 1, &ManipPln::callbackLaGoToLoc, this);
    this->subRaGoToLoc = nh->subscribe("/manipulation/manip_pln/ra_goto_loc", 1, &ManipPln::callbackRaGoToLoc, this);
    this->subHdGoToLoc = nh->subscribe("/manipulation/manip_pln/hd_goto_loc", 1, &ManipPln::callbackHdGoToLoc, this);
    this->subLaMove = nh->subscribe("/manipulation/manip_pln/la_move", 1, &ManipPln::callbackLaMove, this);
    this->subRaMove = nh->subscribe("/manipulation/manip_pln/ra_move", 1, &ManipPln::callbackRaMove, this);
    this->subHdMove = nh->subscribe("/manipulation/manip_pln/hd_move", 1, &ManipPln::callbackHdMove, this);
    //Publishers and subscribers for operating the hardware nodes
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

//
//Callback for subscribers for the commands executed by this node
//

void ManipPln::callbackLaGoToAngles(const std_msgs::Float32MultiArray::ConstPtr& msg)
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

void ManipPln::callbackRaGoToAngles(const std_msgs::Float32MultiArray::ConstPtr& msg)
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

void ManipPln::callbackHdGoToAngles(const std_msgs::Float32MultiArray::ConstPtr& msg)
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

void ManipPln::callbackLaGoToPoseWrtArm(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    
}

void ManipPln::callbackRaGoToPoseWrtArm(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
}

void ManipPln::callbackLaGoToPoseWrtRobot(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
}

void ManipPln::callbackRaGoToPoseWrtRobot(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
}

void ManipPln::callbackLaGoToLoc(const std_msgs::String::ConstPtr& msg)
{
}

void ManipPln::callbackRaGoToLoc(const std_msgs::String::ConstPtr& msg)
{
}

void ManipPln::callbackHdGoToLoc(const std_msgs::String::ConstPtr& msg)
{
}

void ManipPln::callbackLaMove(const std_msgs::String::ConstPtr& msg)
{
}

void ManipPln::callbackRaMove(const std_msgs::String::ConstPtr& msg)
{
}

void ManipPln::callbackHdMove(const std_msgs::String::ConstPtr& msg)
{
}

//
//Callback for subscribers for operating the hardware nodes
//
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

