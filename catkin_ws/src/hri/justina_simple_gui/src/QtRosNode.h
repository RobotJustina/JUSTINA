#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "hri_msgs/RecognizedSpeech.h"
#include "navig_msgs/PathFromMap.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::Publisher pubEqualizer;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void sendEqualizer(float b1, float b2, float b3, float b4, float b5, float b6, float b7, float b8, float b9, float b10);

signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
