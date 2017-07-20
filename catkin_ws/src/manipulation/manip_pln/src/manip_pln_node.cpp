#include <iostream>
#include <fstream>
#include <map>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/path.hpp>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"
#include "manip_msgs/InverseKinematicsPose.h"
#include "manip_msgs/InverseKinematicsPath.h"
#include "manip_msgs/DirectKinematics.h"

#define SM_LA_INIT 0
#define SM_LA_GOAL_ACCEL 1
#define SM_LA_GOAL_CRUISE 2
#define SM_LA_GOAL_DECCEL 3
#define SM_LA_GOAL_FINISH 4
#define SM_RA_INIT 10
#define SM_RA_GOAL_ACCEL 11
#define SM_RA_GOAL_CRUISE 12
#define SM_RA_GOAL_DECCEL 13
#define SM_RA_GOAL_FINISH 14
#define SM_HD_INIT 20
#define SM_HD_GOAL_ACCEL 21
#define SM_HD_GOAL_CRUISE 22
#define SM_HD_GOAL_DECCEL 23
#define SM_HD_GOAL_FINISH 24

std::vector<float> la_goal_angles;
std::vector<float> ra_goal_angles;
std::vector<float> hd_goal_angles;
std::vector<float> la_current_angles;
std::vector<float> ra_current_angles;
std::vector<float> hd_current_angles;

std::map<std::string, std::vector<float> > laPredefPoses;
std::map<std::string, std::vector<float> > raPredefPoses;
std::map<std::string, std::vector<float> > hdPredefPoses;

ros::ServiceClient cltIkFloatArray;
ros::ServiceClient cltIkPath;
ros::ServiceClient cltIkPose;

bool la_new_pose = false;
bool ra_new_pose = false;
bool hd_new_pose = false;
bool stop = false;

void callbackLaGoToAngles(std_msgs::Float32MultiArray::Ptr msg)
{
    std::cout << "ManipPln.->Received Left Arm goal pose: ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Invalid data: LaGoalPose must have 3, 6 or 7 values. " << std::endl;
        return;
    }
    
    while(msg->data.size() < 7) //All non-specified angles are considered to be zero
        msg->data.push_back(0);

    la_goal_angles = msg->data;
    la_new_pose    = true;
    stop = false;
}

void callbackRaGoToAngles(std_msgs::Float32MultiArray::Ptr msg)
{
    std::cout << "ManipPln.->Received Right Arm goal pose: ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;

    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Invalid data: RaGoalPose must have 3, 6 or 7 values. " << std::endl;
        return;
    }

    while(msg->data.size() < 7) //All non-specified angles are considered to be zero
        msg->data.push_back(0);

    ra_goal_angles = msg->data;
    ra_new_pose    = true;
    stop = false;
}

void callbackHdGoToAngles(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "ManipPln.->Received Head goal pose: ";
    //for(int i=0; i< 2; i++)
    //    std::cout << msg->data[i] << " ";
    //std::cout << std::endl;
    
    if(msg->data.size() != 2)
    {
        std::cout << "ManipPln.->Invalid data: HeadGoalPose must have 2 values. " << std::endl;
        return;
    }

    hd_goal_angles = msg->data;
    hd_new_pose    = true;
    stop = false;
}

void callbackLaGoToPoseWrtArm(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "ManipPln.->Received Left arm goal pose (wrt arm): ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }
    
    std::cout << "ManipPln.->Calling service for inverse kinematics..." << std::endl;
    manip_msgs::InverseKinematicsFloatArray srv;
    srv.request.cartesian_pose.data = msg->data;
    if(!cltIkFloatArray.call(srv))
    {
        std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
        return;
    }

    la_goal_angles = srv.response.articular_pose.data;
    la_new_pose    = true;
    stop = false;
}

void callbackRaGoToPoseWrtArm(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "ManipPln.->Received Right arm goal pose (wrt arm): ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }
    
    std::cout << "ManipPln.->Calling service for inverse kinematics..." << std::endl;
    manip_msgs::InverseKinematicsFloatArray srv;
    srv.request.cartesian_pose.data = msg->data;
    if(!cltIkFloatArray.call(srv))
    {
        std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
        return;
    }

    ra_goal_angles = srv.response.articular_pose.data;
    ra_new_pose    = true;
    stop = false;
}

void callbackLaGoToPoseWrtRobot(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "ManipPln.->Received Left arm Goal Pose (wrt robot): " << std::endl;
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    tf::TransformListener tf_listener;
    tf::StampedTransform ht;
    tf_listener.waitForTransform("left_arm_link0",  "base_link", ros::Time(0), ros::Duration(0.5));
    tf_listener.lookupTransform("left_arm_link0", "base_link", ros::Time(0), ht);
    
    tf::Vector3 p(msg->data[0], msg->data[1], msg->data[2]);
    tf::Quaternion q;
    if(msg->data.size() > 3)
        q.setRPY(msg->data[3], msg->data[4], msg->data[5]);
    else
        q.setRPY(0,0,0);
    
    p = ht * p; //These two lines make the transform
    q = ht * q;
    
    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(q).getRPY(dRoll, dPitch, dYaw);
    
    std_msgs::Float32MultiArray msg_;
    msg_.data.push_back(p.x());
    msg_.data.push_back(p.y());
    msg_.data.push_back(p.z());
    if(msg->data.size() > 3)
    {
        msg_.data.push_back((float)dRoll);
        msg_.data.push_back((float)dPitch);
        msg_.data.push_back((float)dYaw);
    }
    if(msg->data.size() == 7)
        msg_.data.push_back(msg->data[6]);
    
    const std_msgs::Float32MultiArray::ConstPtr msgptr(new std_msgs::Float32MultiArray(msg_));
    callbackLaGoToPoseWrtArm(msgptr);
}

void callbackRaGoToPoseWrtRobot(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "ManipPln.->Received Right arm Goal Pose (wrt robot): " << std::endl;
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    tf::TransformListener tf_listener;
    tf::StampedTransform ht;
    tf_listener.waitForTransform("right_arm_link0",  "base_link", ros::Time(0), ros::Duration(0.5));
    tf_listener.lookupTransform("right_arm_link0", "base_link", ros::Time(0), ht);
    
    tf::Vector3 p(msg->data[0], msg->data[1], msg->data[2]);
    tf::Quaternion q;
    if(msg->data.size() > 3)
        q.setRPY(msg->data[3], msg->data[4], msg->data[5]);
    else
        q.setRPY(0,0,0);
    
    p = ht * p; //These two lines make the transform
    q = ht * q;
    
    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(q).getRPY(dRoll, dPitch, dYaw);
    
    std_msgs::Float32MultiArray msg_;
    msg_.data.push_back(p.x());
    msg_.data.push_back(p.y());
    msg_.data.push_back(p.z());
    if(msg->data.size() > 3)
    {
        msg_.data.push_back((float)dRoll);
        msg_.data.push_back((float)dPitch);
        msg_.data.push_back((float)dYaw);
    }
    if(msg->data.size() == 7)
        msg_.data.push_back(msg->data[6]);
    
    const std_msgs::Float32MultiArray::ConstPtr msgptr(new std_msgs::Float32MultiArray(msg_));
    callbackRaGoToPoseWrtArm(msgptr);
}

void callbackLaGoToLoc(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "ManipPln.->Received left arm predefined position: " << msg->data << std::endl;
    if(laPredefPoses.find(msg->data) == laPredefPoses.end())
    {
        std::cout << "ManipPln.->Cannot find left arm predefined position: " << msg->data << std::endl;
        return;
    }
    
    std::cout << "ManipPln.->Left Arm goal pose: " << msg->data << " = ";
    for(int i=0; i< laPredefPoses[msg->data].size(); i++)
        std::cout << laPredefPoses[msg->data][i] << " ";
    std::cout << std::endl;

    la_goal_angles = laPredefPoses[msg->data];
    la_new_pose    = true;
    stop = false;
}

void callbackRaGoToLoc(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "ManipPln.->Received right arm predefined position: " << msg->data << std::endl;
    if(raPredefPoses.find(msg->data) == raPredefPoses.end())
    {
        std::cout << "ManipPln.->Cannot find right arm predefined position: " << msg->data << std::endl;
        return;
    }
    
    std::cout << "ManipPln.->Right Arm goal pose: " << msg->data << " = ";
    for(int i=0; i< raPredefPoses[msg->data].size(); i++)
        std::cout << raPredefPoses[msg->data][i] << " ";
    std::cout << std::endl;

    ra_goal_angles = raPredefPoses[msg->data];
    ra_new_pose    = true;
    stop = false;
}

void callbackLaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    la_current_angles = msg->data;
}

void callbackRaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ra_current_angles = msg->data;
}

void callbackHdCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    hd_current_angles = msg->data;
}

std::map<std::string, std::vector<float> > loadArrayOfFloats(std::string path)
{
    std::cout << "ManipPln.->Extracting array of floats from file: " << path << std::endl;
    std::vector<std::string> lines;
    std::ifstream file(path.c_str());
    std::string tempStr;
    while(std::getline(file, tempStr))
        lines.push_back(tempStr);

    //Extraction of lines without comments
    for(size_t i=0; i < lines.size(); i++)
    {
        size_t idx = lines[i].find("//");
        if(idx != std::string::npos)
            lines[i] = lines[i].substr(0, idx);
    }

    std::map<std::string, std::vector<float> > data;

    float fValue;
    bool parseSuccess;
    for(size_t i=0; i<lines.size(); i++)
    {
        //std::cout << "ManipPln.->Parsing line: " << lines[i] << std::endl;
        std::vector<std::string> parts;
        boost::split(parts, lines[i], boost::is_any_of(" ,\t"), boost::token_compress_on);
        if(parts.size() < 2)
            continue;
        //First part should be the label and the next ones, the values
        if(!boost::filesystem::portable_posix_name(parts[0]))
            continue;
        parseSuccess = true;
        for(size_t j=1; j<parts.size() && parseSuccess; j++)
        {
            std::stringstream ssValue(parts[j]);
            if(!(ssValue >> fValue)) parseSuccess = false;
            else data[parts[0]].push_back(fValue);
        }
    }
    return data;
}

bool loadPredefinedPosesAndMovements(std::string folder)
{
    //Load predefined positions for left arm
    //
    std::string leftArmPosesFile = folder + "left_arm_poses.txt";
    std::map<std::string, std::vector<float> > data = loadArrayOfFloats(leftArmPosesFile);
    for(std::map<std::string, std::vector<float> >::iterator i = data.begin(); i != data.end(); i++)
    {
        if(i->second.size() != 7)
        {
            std::cout << "ManipPln.->Invalid number of angles in left arm predef position " << i->first << std::endl;
            continue;
        }
        laPredefPoses[i->first] = i->second;
    }
    std::cout << "ManipPln.->Left arm predefined positions: " <<std::endl;
    for(std::map<std::string, std::vector<float> >::iterator i = laPredefPoses.begin(); i != laPredefPoses.end(); i++)
    {
        std::cout << i->first << " ";
        for(int j=0; j < i->second.size(); j++)
            std::cout << i->second[j] << " ";
        std::cout << std::endl;
    }

    //
    //Load predefined positions for right arm
    //
    std::string rightArmPosesFile = folder + "right_arm_poses.txt";
    data = loadArrayOfFloats(rightArmPosesFile);
    for(std::map<std::string, std::vector<float> >::iterator i = data.begin(); i != data.end(); i++)
    {
        if(i->second.size() != 7)
        {
            std::cout << "ManipPln.->Invalid number of angles in right arm predef position " << i->first << std::endl;
            continue;
        }
        raPredefPoses[i->first] = i->second;
    }
    std::cout << "ManipPln.->Right arm predefined positions: " <<std::endl;
    for(std::map<std::string, std::vector<float> >::iterator i = laPredefPoses.begin(); i != laPredefPoses.end(); i++)
    {
        std::cout << i->first << " ";
        for(int j=0; j < i->second.size(); j++)
            std::cout << i->second[j] << " ";
        std::cout << std::endl;
    }
    return true;
}

float calculateError(std::vector<float>& v1, std::vector<float>& v2)
{
    float max = 0;
    for(int i=0; i < v1.size(); i++)
    {
        float temp = fabs(v1[i] - v2[i]);
        if(temp > max)
            max = temp;
    }
    return max;
}

int main(int argc, char** argv)
{
    std::string folder = "";
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
            folder = argv[++i];
    }
    
    std::cout << "INITIALIZING MANIPULATION PLANNER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "manip_pln");
    ros::NodeHandle n;
    //Publishers for indicating that a goal pose has been reached
    ros::Publisher  pubLaGoalReached = n.advertise<std_msgs::Bool>("/manipulation/la_goal_reached", 1);
    ros::Publisher  pubRaGoalReached = n.advertise<std_msgs::Bool>("/manipulation/ra_goal_reached", 1);
    ros::Publisher  pubHdGoalReached = n.advertise<std_msgs::Bool>("/manipulation/hd_goal_reached", 1);
    //Subscribers for the commands executed by this node
    ros::Subscriber subLaGoToAngles       = n.subscribe("/manipulation/manip_pln/la_goto_angles",  1, callbackLaGoToAngles);     
    ros::Subscriber subRaGoToAngles       = n.subscribe("/manipulation/manip_pln/ra_goto_angles",  1, callbackRaGoToAngles);     
    ros::Subscriber subHdGoToAngles       = n.subscribe("/manipulation/manip_pln/hd_goto_angles",  1, callbackHdGoToAngles);     
    ros::Subscriber subLaGoToPoseWrtArm   = n.subscribe("/manipulation/manip_pln/la_pose_wrt_arm", 1, callbackLaGoToPoseWrtArm);
    ros::Subscriber subRaGoToPoseWrtArm   = n.subscribe("/manipulation/manip_pln/ra_pose_wrt_arm", 1, callbackRaGoToPoseWrtArm);
    ros::Subscriber subLaGoToPoseWrtRobot = n.subscribe("/manipulation/manip_pln/la_pose_wrt_robot", 1, callbackLaGoToPoseWrtRobot);
    ros::Subscriber subRaGoToPoseWrtRobot = n.subscribe("/manipulation/manip_pln/ra_pose_wrt_robot", 1, callbackRaGoToPoseWrtRobot);
    ros::Subscriber subLaGoToLoc          = n.subscribe("/manipulation/manip_pln/la_goto_loc", 1, callbackLaGoToLoc);          
    ros::Subscriber subRaGoToLoc          = n.subscribe("/manipulation/manip_pln/ra_goto_loc", 1, callbackRaGoToLoc);
    //Publishers and subscribers for operating the hardware nodes
    ros::Subscriber subLaCurrentPose = n.subscribe("/hardware/left_arm/current_pose",  1, callbackLaCurrentPose); 
    ros::Subscriber subRaCurrentPose = n.subscribe("/hardware/right_arm/current_pose", 1, callbackRaCurrentPose);
    ros::Subscriber subHdCurrentPose = n.subscribe("/hardware/head/current_pose",      1, callbackHdCurrentPose); 
    ros::Publisher pubLaGoalPose     = n.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1);               
    ros::Publisher pubRaGoalPose     = n.advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 1);              
    ros::Publisher pubHdGoalPose     = n.advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    //Stuff for tranformations and inverse kinematics
    cltIkFloatArray = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");
    tf::TransformListener tf_listener;
    ros::Rate loop(20);

    std::cout << "ManipPln.->Trying to load predefined positions from folder: " << folder << std::endl;
    loadPredefinedPosesAndMovements(folder);

    tf::StampedTransform transform;
    tf::Quaternion q;
    try
    {
        tf_listener.waitForTransform("left_arm_link0",  "base_link", ros::Time(0), ros::Duration(10.0));
        tf_listener.waitForTransform("right_arm_link0", "base_link", ros::Time(0), ros::Duration(10.0));
        tf_listener.lookupTransform("left_arm_link0",  "base_link", ros::Time(0), transform);
        tf_listener.lookupTransform("right_arm_link0", "base_link", ros::Time(0), transform);
    }
    catch(...)
    {
        std::cout << "ManipPln.->Cannot get tranforms from arms to base_link... :'(" << std::endl;
        return -1;
    }

    int la_state = SM_LA_INIT;
    int ra_state = SM_RA_INIT;
    int hd_state = SM_HD_INIT;
    float la_error = 0;
    float ra_error = 0;
    float hd_error = 0;
    float la_speed = 0;
    float ra_speed = 0;
    float hd_speed = 0;
    float la_min_speed = 0.01;
    float ra_min_speed = 0.01;
    float hd_min_speed = 0.01;
    float la_max_speed = 0.15;
    float ra_max_speed = 0.15;
    float hd_max_speed = 0.15;
    float la_tolerance = 0.05;
    float ra_tolerance = 0.05;
    float hd_tolerance = 0.05;
    int la_attempts = 0;
    int ra_attempts = 0;
    int hd_attempts = 0;
    float delta_increment  = 0.0033;
    float delta_decrement  = 0.0033;
    std::vector<float> la_speed_factors;
    std::vector<float> ra_speed_factors;
    std::vector<float> hd_speed_factors;
    std_msgs::Bool msg_goal_reached;
    std_msgs::Float32MultiArray msg_la_goal_pose;
    std_msgs::Float32MultiArray msg_ra_goal_pose;
    std_msgs::Float32MultiArray msg_hd_goal_pose;
    la_speed_factors.resize(7);
    ra_speed_factors.resize(7);
    hd_speed_factors.resize(7);
    msg_la_goal_pose.data.resize(14);
    msg_ra_goal_pose.data.resize(14);
    msg_hd_goal_pose.data.resize(14);

    while(ros::ok())
    {
        //
        //STATE MACHIME FOR THE LEFT ARM CONTROL
        //
        switch(la_state)
        {
        case SM_LA_INIT:
            la_speed = 0;
            if(la_new_pose)
            {
                la_state = SM_LA_GOAL_ACCEL;
                la_new_pose = false;
                la_error = calculateError(la_goal_angles, la_current_angles);
                la_attempts = (la_error + 0.2)*60;
                for(int i=0; i < la_speed_factors.size(); i++)
                {
                    la_speed_factors[i] = fabs(la_goal_angles[i] - la_current_angles[i])/la_error;
                    msg_la_goal_pose.data[i] = la_goal_angles[i];
                }
            }
            break;

            
        case SM_LA_GOAL_ACCEL:
            la_speed += delta_increment;
            la_error = calculateError(la_goal_angles, la_current_angles);
            std::cout << "Speed: " << la_speed << "\tattempts = " << la_attempts << "\terror = " << la_error << std::endl;
            if(la_error < la_tolerance)
                la_state = SM_LA_GOAL_FINISH;
            else
            {
                if(la_error < la_speed*10)
                    la_state = SM_LA_GOAL_DECCEL;
                else if(la_speed >= la_max_speed)
                    la_state = SM_LA_GOAL_CRUISE;
                else
                    la_state = SM_LA_GOAL_ACCEL;

                for(int i=7; i < 14; i++) msg_la_goal_pose.data[i] = la_speed * la_speed_factors[i-7];
                pubLaGoalPose.publish(msg_la_goal_pose);
            }
            if(--la_attempts <= 0)
                la_state = SM_LA_GOAL_FINISH;
            break;

            
        case SM_LA_GOAL_CRUISE:
            la_error = calculateError(la_goal_angles, la_current_angles);
            std::cout << "Speed: " << la_speed << "\tattempts = " << la_attempts << "\terror = " << la_error << std::endl;
            if(la_error < la_speed*10)
                la_state = SM_LA_GOAL_DECCEL;

            //It is not necessary to reasign speeds since it will keep the speeds calculated in the ACCEL state
            pubLaGoalPose.publish(msg_la_goal_pose);
            if(--la_attempts <= 0)
                la_state = SM_LA_GOAL_FINISH;
            break;

            
        case SM_LA_GOAL_DECCEL:
            la_speed -= delta_decrement;
            la_error = calculateError(la_goal_angles, la_current_angles);
            std::cout << "Speed: " << la_speed << "\tattempts = " << la_attempts << "\terror = " << la_error << std::endl;
            if(la_speed < la_min_speed) la_speed = la_min_speed;
            if(la_error < la_tolerance)
                la_state = SM_LA_GOAL_FINISH;

            for(int i=7; i < 14; i++) msg_la_goal_pose.data[i] = la_speed * la_speed_factors[i-7];
                pubLaGoalPose.publish(msg_la_goal_pose);
            if(--la_attempts <= 0)
                la_state = SM_LA_GOAL_FINISH;
            break;

            
        case SM_LA_GOAL_FINISH:
            std::cout << "ManipPln.->Succesful left arm movement (Y)" << std::endl;
            la_state = SM_LA_INIT;
            msg_goal_reached.data = true;
            pubLaGoalReached.publish(msg_goal_reached);
            break;
            
        default:
            std::cout << "ManipPln.->A VERY STUPID PERSON PROGRAMMED THIS SHIT. SORRY :'(" << std::endl;
            return -1;
        }


        //
        //STATE MACHIME FOR THE RIGHT ARM CONTROL
        //
        switch(ra_state)
        {
        case SM_RA_INIT:
            break;

            
        case SM_RA_GOAL_ACCEL:
            break;

            
        case SM_RA_GOAL_CRUISE:
            break;

            
        case SM_RA_GOAL_DECCEL:
            break;

            
        case SM_RA_GOAL_FINISH:
            break;
            
        default:
            std::cout << "ManipPln.->A VERY STUPID PERSON PROGRAMMED THIS SHIT. SORRY :'(" << std::endl;
            return -1;
        }


        //
        //STATE MACHIME FOR THE HEAD CONTROL
        //
        switch(hd_state)
        {
        case SM_HD_INIT:
            break;

            
        case SM_HD_GOAL_ACCEL:
            break;

            
        case SM_HD_GOAL_CRUISE:
            break;

            
        case SM_HD_GOAL_DECCEL:
            break;

            
        case SM_HD_GOAL_FINISH:
            break;
            
        default:
            std::cout << "ManipPln.->A VERY STUPID PERSON PROGRAMMED THIS SHIT. SORRY :'(" << std::endl;
            return -1;
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
