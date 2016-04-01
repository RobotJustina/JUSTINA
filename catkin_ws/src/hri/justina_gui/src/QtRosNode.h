#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

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

signals:
    void onRosNodeFinished();
    void onCurrentPoseReceived(float currentX, float currentY, float currentTheta);

private:
    void callbackCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

};
