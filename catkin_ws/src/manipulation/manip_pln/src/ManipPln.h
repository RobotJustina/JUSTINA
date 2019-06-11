#pragma once
#include <iostream>
#include <fstream>
#include <vector>
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
#include "visualization_msgs/MarkerArray.h"

#define THR_MIN 0.03
#define THR_MAX 0.15

class ManipPln
{
public:
    ManipPln();
    ~ManipPln();

private:
    ros::NodeHandle* nh;
    //Publishers for indicating that a goal pose has been reached
    ros::Publisher pubLaGoalReached;
    ros::Publisher pubRaGoalReached;
    ros::Publisher pubHdGoalReached;
    ros::Publisher pubStartGetGripperPosition;
    //Subscribers for the commands executed by this node
    ros::Subscriber subLaGoToAngles;
    ros::Subscriber subRaGoToAngles;
    ros::Subscriber subHdGoToAngles;
    ros::Subscriber subLaGoToPoseWrtArm;
    ros::Subscriber subRaGoToPoseWrtArm;
    ros::Subscriber subLaGoToPoseWrtRobot;
    ros::Subscriber subRaGoToPoseWrtRobot;
    ros::Subscriber subLaGoToPoseWrtArmFeedback;
    ros::Subscriber subRaGoToPoseWrtArmFeedback;
    ros::Subscriber subLaGoToPoseWrtRobotFeedback;
    ros::Subscriber subRaGoToPoseWrtRobotFeedback;
    ros::Subscriber subLaStopGoTo;
    ros::Subscriber subRaStopGoTo;
    ros::Subscriber subLaGoToPoseWrtArmTraj;
    ros::Subscriber subRaGoToPoseWrtArmTraj;
    ros::Subscriber subLaGoToPoseWrtRobotTraj;
    ros::Subscriber subRaGoToPoseWrtRobotTraj;
    ros::Subscriber subLaGoToLoc;
    ros::Subscriber subRaGoToLoc;
    ros::Subscriber subHdGoToLoc;
    ros::Subscriber subLaMove;
    ros::Subscriber subRaMove;
    ros::Subscriber subHdMove;
    ros::Subscriber subGripperPosition;
    //Publishers and subscribers for operating the hardware nodes
    ros::Subscriber subLaCurrentPose;
    ros::Subscriber subRaCurrentPose;
    ros::Subscriber subHdCurrentPose;
    ros::Publisher pubLaGoalPose;
    ros::Publisher pubRaGoalPose;
    ros::Publisher pubHdGoalPose;
    ros::Publisher pubLaGoalTorque;
    ros::Publisher pubRaGoalTorque;
    ros::Publisher pubHdGoalTorque;
    //Publishers of the marker manipulation
    ros::Publisher pubMarkerManipulation; 
    //Stuff for tranformations and inverse kinematics
    ros::ServiceClient cltIkFloatArray;
    ros::ServiceClient cltIkFloatArrayWithoutOpt;
    ros::ServiceClient cltIkPath;
    ros::ServiceClient cltIkPose;
    ros::ServiceClient cltDK;

    tf::TransformListener* tf_listener;
    

    bool laNewGoal;
    bool raNewGoal;
    bool hdNewGoal;
    bool laFeedbackNewGoal;
    bool raFeedbackNewGoal;
    bool laNewGoalTraj;
    bool raNewGoalTraj;
    std::vector<float> lCarGoalPose;
    std::vector<float> rCarGoalPose;
    std::vector<float> laCurrentPose;
    std::vector<float> raCurrentPose;
    std::vector<float> hdCurrentPose;
    std::vector<float> laGoalPose;
    std::vector<float> raGoalPose;
    std::vector<float> hdGoalPose;
    std::vector<float> laGoalSpeeds;
    std::vector<float> raGoalSpeeds;
    std::vector<float> hdGoalSpeeds;
    std::map<std::string, std::vector<float> > laPredefPoses;
    std::map<std::string, std::vector<float> > raPredefPoses;
    std::map<std::string, std::vector<float> > hdPredefPoses;
    std::map<std::string, std::vector<std::vector<float> > > laPredefMoves;
    std::map<std::string, std::vector<std::vector<float> > > raPredefMoves;
    std::map<std::string, std::vector<std::vector<float> > > hdPredefMoves;
    geometry_msgs::Point gPos;
    visualization_msgs::MarkerArray manipMarker;
    std::vector<std::vector<float> > lGoalArticularTraj;
    std::vector<std::vector<float> > lGoalCartesianTraj;
    std::vector<std::vector<float> > rGoalArticularTraj;
    std::vector<std::vector<float> > rGoalCartesianTraj;
    int idMarker;

public:
    void setNodeHandle(ros::NodeHandle* n);
    bool loadPredefinedPosesAndMovements(std::string folder);
    void spin();

private:
    float calculateError(std::vector<float>& v1, std::vector<float>& v2);
    void calculateOptimalSpeeds(std::vector<float>& currentPose, std::vector<float>& goalPose, std::vector<float>& speeds);
    void calculateOptimalSpeeds(float currx, float curry, float currz, float goalx, float goaly, float goalz, std::vector<float>& speeds);
    void calculateOptimalSpeeds(float currx, float curry, float currz, float goalx, float goaly, float goalz, std::vector<float> currentArtPose, std::vector<float> goalArtPose , std::vector<float>& speeds);
    std::map<std::string, std::vector<float> > loadArrayOfFloats(std::string path);
    std::map<std::string, std::vector<std::vector<float> > > loadArrayOfArrayOfFloats(std::string path);
    //Callback for subscribers for the commands executed by this node
    void callbackLaGoToAngles(std_msgs::Float32MultiArray::Ptr msg);
    void callbackRaGoToAngles(std_msgs::Float32MultiArray::Ptr msg);
    void callbackHdGoToAngles(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackLaGoToPoseWrtArm(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackRaGoToPoseWrtArm(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackLaGoToPoseWrtRobot(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackRaGoToPoseWrtRobot(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackLaGoToPoseWrtArmFeedback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackRaGoToPoseWrtArmFeedback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackLaGoToPoseWrtRobotFeedback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackRaGoToPoseWrtRobotFeedback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackLaStopGoTo(const std_msgs::Empty::ConstPtr &msg);
    void callbackRaStopGoTo(const std_msgs::Empty::ConstPtr &msg);
    void callbackLaGoToPoseWrtArmTraj(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackRaGoToPoseWrtArmTraj(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackLaGoToPoseWrtRobotTraj(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackRaGoToPoseWrtRobotTraj(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackLaGoToLoc(const std_msgs::String::ConstPtr& msg);
    void callbackRaGoToLoc(const std_msgs::String::ConstPtr& msg);
    void callbackHdGoToLoc(const std_msgs::String::ConstPtr& msg);
    void callbackLaMove(const std_msgs::String::ConstPtr& msg);
    void callbackRaMove(const std_msgs::String::ConstPtr& msg);
    void callbackHdMove(const std_msgs::String::ConstPtr& msg);
    //Callback for subscribers for operating the hardware nodes
    void callbackLaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackRaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackHdCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackgPos(const geometry_msgs::Point::ConstPtr& msg);
};
