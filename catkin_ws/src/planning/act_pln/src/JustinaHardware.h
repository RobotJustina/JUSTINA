#pragma once
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"

class JustinaHardware
{
public:
    static ros::Publisher pub_Head_GoalPose;
    static ros::Publisher pub_La_GoalPose;
    static ros::Publisher pub_Ra_GoalPose;
    static ros::Publisher pub_Spg_Say;
    static ros::Subscriber sub_Spr_Recognized;
    
    static bool SetNodeHandle(ros::NodeHandle* nh);
    //Methods for operating the mobile base
    static bool MoveBase(float dist);
    static bool MoveBase(float dist, float angle);
    static bool MoveToPose(float x, float y);
    static bool MoveToPose(float x, float y, float theta);
    static bool MoveToPoseRel(float x, float y);
    static bool StartMoveBase(float dist);
    static bool StartMoveBase(float dist, float angle);
    static bool StartMoveToPose(float x, float y);
    static bool StartMoveToPose(float x, float y, float theta);
    static bool StartMoveToPoseRel(float x, float y);
    static bool GetCurrentRobotPos(float& robotX, float& robotY, float& robotTheta);
    //Methods for speech synthesis and recognition
    static bool Say(std::string strToSay);
    static bool WaitForRecogSpeech(std::string& recognized, int time_out_ms);
    //Methods for operating arms
    static bool LeftArmGoTo(float x, float y, float z);
    static bool LeftArmGoTo(float x, float y, float z, float roll, float pitch, float yaw);
    static bool LeftArmGoTo(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static bool LeftArmArticular(std::vector<float> angles);
    static bool LeftArmGoTo(std::string location);
    static bool LeftArmMove(std::string movement);
    static bool StartLeftArmGoTo(float x, float y, float z);
    static bool StartLeftArmGoTo(float x, float y, float z, float roll, float pitch, float yaw);
    static bool StartLeftArmGoTo(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static bool StartLeftArmGoTo(std::string location);
    static bool StartLeftArmMove(std::string movement);
    static bool RightArmGoTo(float x, float y, float z);
    static bool RightArmGoTo(float x, float y, float z, float roll, float pitch, float yaw);
    static bool RightArmGoTo(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static bool RightArmArticular(std::vector<float> angles);
    static bool RightArmGoTo(std::string location);
    static bool RightArmMove(std::string movement);
    static bool StartRightArmGoTo(float x, float y, float z);
    static bool StartRightArmGoTo(float x, float y, float z, float roll, float pitch, float yaw);
    static bool StartRightArmGoTo(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static bool StartRightArmGoTo(std::string location);
    static bool StartRightArmMove(std::string movement);
    //Methods for operating head
    static bool HeadGoTo(float pan, float tilt);
    static bool HeadGoTo(std::string position);
    static bool HeadMove(std::string movement);
    static bool StartHeadGoTo(float pan, float tilt);
    static bool StartHeadGoTo(std::string position);
    static bool StartHeadMove(std::string movement);

    //callbacks
    static void callbackRecognized(const std_msgs::String::ConstPtr& msg);
};
