#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include <ros/package.h>

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

signals:
    void updateGraphics();
    void onRosNodeFinished();
};
