#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"

#define SM_INIT 0
#define SM_WAITING_FOR_NEW_TASK 1
#define SM_CALCULATE_PATH 2

class MvnPln
{
public:
    MvnPln();
    ~MvnPln();

private:
    ros::NodeHandle* nh;
    //Publishers and subscribers for the commands executed by this node
    ros::Subscriber subGetCloseLoc;
    ros::Subscriber subGetCloseXYA;
    ros::Publisher pubGoalReached;

    bool newTask;
    std::map<std::string, std::vector<float> > locations;

public:
    void initROSConnection(ros::NodeHandle* nh);
    bool loadKnownLocations(std::string path);
    void spin();

private:
    void callbackGetCloseLoc(const std_msgs::String::ConstPtr& msg);
    void callbackGetCloseXYA(const std_msgs::Float32MultiArray::ConstPtr& msg);
};
