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

class JustinaVision
{
private:
    static bool is_node_set;
    //Members for operating skeleton finder
    static ros::Publisher pubSktStartRecog;
    static ros::Publisher pubSktStopRecog;
    //Members for operating face recognizer
    static ros::Publisher pubFacStartRecog;
    static ros::Publisher pubFacStopRecog;

public:
    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methods for operating skeleton finder
    static void startSkeletonFinding();
    static void stopSkeletonFinding();
    //Methods for operating face recognizer
    static void startFaceRecognition();
    static void stopFaceRecognition();
};
