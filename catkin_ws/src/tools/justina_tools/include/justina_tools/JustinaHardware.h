#pragma once
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "bbros_bridge/RecognizedSpeech.h"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/PointCloud2.h"
#include "point_cloud_manager/GetRgbd.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/thread/thread.hpp"

class JustinaHardware
{
private:
    static bool is_node_set;
    //Publishers and subscribers for operating the head
    static ros::Subscriber subHeadCurrentPose;
    static ros::Publisher pubHeadGoalPose;
    //Publishers and subscribers for operating left arm
    static ros::Subscriber subLeftArmCurrentGripper;
    static ros::Subscriber subLeftArmCurrentPose;
    static ros::Publisher pubLeftArmGoalGripper;
    static ros::Publisher pubLeftArmGoalPose;
    static ros::Publisher pubLeftArmGoalTorqueGrip;
    static ros::Publisher pubLeftArmGoalTorque;
    //Publishers and subscribers for operating right arm
    static ros::Subscriber subRightArmCurrentGripper;
    static ros::Subscriber subRightArmCurrentPose;
    static ros::Publisher pubRightArmGoalGripper;
    static ros::Publisher pubRightArmGoalPose;
    static ros::Publisher pubRightArmGoalTorqueGrip;
    static ros::Publisher pubRightArmGoalTorque;
    //Subscribers for operating torso
    static ros::Subscriber subTorsoCurrentPose;
    //Publishers and subscribers for operating mobile base
    static ros::Publisher pubBaseSpeeds;
    static ros::Publisher pubBaseCmdVel;
    //Publishers and subscribers for checking robot state
    static ros::Publisher pubRobotStop;
    static ros::Subscriber subBaseBattery;
    static ros::Subscriber subLeftArmBattery;
    static ros::Subscriber subRightArmBattery;
    static ros::Subscriber subHeadBattery;
    //Topics and services for operating point_cloud_manager
    static ros::ServiceClient cltRgbdKinect;
    static ros::ServiceClient cltRgbdRobot;
    static ros::Publisher pubSaveCloud;
    static ros::Publisher pubStopSavingCloud;

    //Variables for head position
    static float headPan;
    static float headTilt;
    //Variables for arms
    static float leftArmCurrentGripper;
    static float rightArmCurrentGripper;
    static std::vector<float> leftArmCurrentPose;
    static std::vector<float> rightArmCurrentPose;
    //Variables for torso
    static float torsoCurrentSpine;
    static float torsoCurrentWaist;
    static float torsoCurrentShoulders;
    //Variables for robot state;
    static float _baseBattery;
    static float _leftArmBattery;
    static float _rightArmBattery;
    static float _headBattery;
    static int _baseBatteryPerc;
    static int _leftArmBatteryPerc;
    static int _rightArmBatteryPerc;
    static int _headBatteryPerc;

public:
    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methods for operating head
    static void getHeadCurrentPose(float& pan, float& tilt);
    static float getHeadCurrentPan();
    static float getHeadCurrentTilt();
    static void setHeadGoalPose(float pan, float tilt);
    //Methods for operating the left arm
    static float getLeftArmCurrentGripper();
    static void getLeftArmCurrentPose(std::vector<float>& currentPose);
    static void setLeftArmGoalGripper(float goalGripper);
    static void setLeftArmGoalPose(std::vector<float>& goalAngles);
    static void setLeftArmGoalPose(float theta0, float theta1, float theta2, float theta3, float theta4, float theta5, float theta6);
    static void setLeftArmGoalTorqueGrip(float torqueGripper);
    static void setLeftArmGoalTorque(std::vector<float>& goalTorques);
    static void setLeftArmGoalTorque(float t0, float t1, float t2, float t3, float t4, float t5, float t6);
    //Methods for operating right arm
    static float getRightArmCurrentGripper();
    static void getRightArmCurrentPose(std::vector<float>& currentPose);
    static void setRightArmGoalGripper(float goalGripper);
    static void setRightArmGoalPose(std::vector<float>& goalAngles);
    static void setRightArmGoalPose(float theta0, float theta1, float theta2, float theta3, float theta4, float theta5, float theta6);
    static void setRightArmGoalTorqueGrip(float torqueGripper);
    static void setRightArmGoalTorque(std::vector<float>& goalTorques);
    static void setRightArmGoalTorque(float t0, float t1, float t2, float t3, float t4, float t5, float t6);
    //Methods for operating torso
    static void getTorsoCurrentPose(float& spine, float& waist, float& shoulders);
    //Methods for operating the mobile base
    static void setBaseSpeeds(float leftSpeed, float rightSpeed);
    static void setBaseCmdVel(float linearX, float linearY, float angular);
    //Methods for operating robot state
    static void stopRobot();
    static bool isStopRobot();
    static float baseBattery();
    static float leftArmBattery();
    static float rightArmBattery();
    static float headBattery();
    static int baseBatteryPerc();
    static int leftArmBatteryPerc();
    static int rightArmBatteryPerc();
    static int headBatteryPerc();
    //Methods for operating point_cloud_manager
    static bool getRgbdWrtKinect(sensor_msgs::PointCloud2& cloud);
    static bool getRgbdWrtRobot(sensor_msgs::PointCloud2& cloud);
    static void startSavingCloud(std::string fileName);
    static void stopSavingCloud();
     
    //callbacks for head operation
    static void callbackHeadCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    //callbacks for left arm operation
    static void callbackLeftArmCurrentGripper(const std_msgs::Float32::ConstPtr& msg);
    static void callbackLeftArmCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    //callbacks for right arm operation
    static void callbackRightArmCurrentGripper(const std_msgs::Float32::ConstPtr& msg);
    static void callbackRightArmCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    //callbacks for torso
    static void callbackTorsoCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    //callbacks for robot state
    static void callbackBaseBattery(const std_msgs::Float32::ConstPtr& msg);
    static void callbackLeftArmBattery(const std_msgs::Float32::ConstPtr& msg);
    static void callbackRightArmBattery(const std_msgs::Float32::ConstPtr& msg);
    static void callbackHeadBattery(const std_msgs::Float32::ConstPtr& msg);
};
