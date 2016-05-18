#pragma once
#include <iostream>
#include <vector>
#include <map>
#include "geometry_msgs/Point.h"

class MvnPln
{
public:
    MvnPln();
    ~MvnPln();

    ros::NodeHandle* nh;
    ros::Subscriber subGetCloseLoc;
    ros::Subscriber subGetCloseXYA;
    ros::Publisher pubGoalReached;

    bool newGoal;
    int currentState;
    std::map<string, std::vector<float> > locations;

    void initROSConnection(ros::NodeHandle* nh);
    bool loadKnownLocations(std::string path);
    void spin();

    bool GetClose(std::string location);
    bool GetClose(float goalX, float goalY);
    bool GetClose(float goalX, float goalY, float goalTheta);

    void callbackGetCloseLoc(const std_msgs::String::ConstPtr& msg);
    void callbackGetCloseXYA(const std_msgs::Float32MultiArray::ConstPtr& msg);
};
