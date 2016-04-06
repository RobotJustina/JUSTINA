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
    bool gui_closed;
    
    void run();
    void publish_SimpleMove_GoalDist(float goalDist);
    void publish_PathCalculator_WaveFront(float currentX, float currentY, float currentTheta, float goalX, float goalY, float goalTheta);
    void publish_Head_GoalPose(float pan, float tilt);

signals:
    void onRosNodeFinished();
    void onCurrentPoseReceived(float currentX, float currentY, float currentTheta);

private:
    void callbackCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

};
