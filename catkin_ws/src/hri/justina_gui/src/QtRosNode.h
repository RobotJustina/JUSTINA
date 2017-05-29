#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "hri_msgs/RecognizedSpeech.h"
#include "navig_msgs/PathFromMap.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaRepresentation.h"

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
