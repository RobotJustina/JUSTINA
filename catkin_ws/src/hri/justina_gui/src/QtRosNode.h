#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "navig_msgs/PathFromMap.h"
#include "nav_msgs/GetMap.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::Publisher pub_SimpleMove_GoalDist;
    ros::Publisher pub_Head_GoalPose;
    ros::Publisher pub_La_GoalPose;
    ros::Publisher pub_Ra_GoalPose;
    bool gui_closed;
    
    void run();
    void call_PathCalculator_WaveFront(float currentX, float currentY, float currentAng, float goalX, float goalY, float goalAng);
    void call_PathCalculator_AStar(float currentX, float currentY, float currentAng, float goalX, float goalY, float goalAngl);
    void publish_SimpleMove_GoalDist(float goalDist);
    void publish_Head_GoalPose(float pan, float tilt);
    void publish_La_GoalPose(std::vector<float> angles);
    void publish_Ra_GoalPose(std::vector<float> angles);

signals:
    void onRosNodeFinished();
    void onCurrentRobotPoseReceived(float currentX, float currentY, float currentTheta);
    void onCurrentHeadPoseReceived(float pan, float tilt);
    void onCurrentLaPoseReceived(std::vector<float> angles);
    void onCurrentRaPoseReceived(std::vector<float> angles);

private:
    void callbackRobotCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void callbackHeadCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackLaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackRaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
};
