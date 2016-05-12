#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"

class ManipPln
{
public:
    ManipPln();
    ~ManipPln();

private:
    ros::NodeHandle* nh;
    ros::Publisher pubLaGoalReached;
    ros::Publisher pubRaGoalReached;
    ros::Publisher pubHdGoalReached;
    ros::Subscriber subLaGoalPose;
    ros::Subscriber subRaGoalPose;
    ros::Subscriber subHdGoalPose;
    ros::Subscriber subLaCurrentPose;
    ros::Subscriber subRaCurrentPose;
    ros::Subscriber subHdCurrentPose;
    ros::Publisher pubLaGoalPose;
    ros::Publisher pubRaGoalPose;
    ros::Publisher pubHdGoalPose;
    ros::Publisher pubLaGoalTorque;
    ros::Publisher pubRaGoalTorque;
    ros::Publisher pubHdGoalTorque;

    bool laNewGoal;
    bool raNewGoal;
    bool hdNewGoal;
    std::vector<float> laCurrentPose;
    std::vector<float> raCurrentPose;
    std::vector<float> hdCurrentPose;
    std::vector<float> laGoalPose;
    std::vector<float> raGoalPose;
    std::vector<float> hdGoalPose;
    std::map<std::string, std::vector<float> > laPredefPoses;
    std::map<std::string, std::vector<float> > raPredefPoses;
    std::map<std::string, std::vector<float> > hdPredefPoses;
    std::map<std::string, std::vector<std::vector<float> > > laPredefMoves;
    std::map<std::string, std::vector<std::vector<float> > > raPredefMoves;
    std::map<std::string, std::vector<std::vector<float> > > hdPredefMoves;

public:
    void setNodeHandle(ros::NodeHandle* n);
    bool loadKnownPosesAndMovs(std::string directory);
    void spin();

private:
    float calculateError(std::vector<float>& v1, std::vector<float>& v2);
    void callbackLaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackRaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackHdCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackLaGoalPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackRaGoalPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackHdGoalPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
};
