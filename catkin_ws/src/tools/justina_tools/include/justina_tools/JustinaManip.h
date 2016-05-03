#pragma once
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"
#include "manip_msgs/InverseKinematicsPath.h"
#include "manip_msgs/InverseKinematicsPose.h"
#include "manip_msgs/DirectKinematics.h"

class JustinaManip
{
public:
    static bool is_node_set;
    static ros::ServiceClient cltIKFloatArray;
    static ros::ServiceClient cltIKPath;
    static ros::ServiceClient cltIKPose;
    static ros::ServiceClient cltDK;

    static bool setNodeHandle(ros::NodeHandle* nh);

    static bool inverseKinematics(std::vector<float>& cartesian, std::vector<float>& articular);
    static bool inverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::vector<float>& articular);
    static bool inverseKinematics(float x, float y, float z, std::vector<float>& articular);
    static bool inverseKinematics(std::vector<float>& cartesian, std::string frame_id, std::vector<float>& articular);
    static bool inverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::string frame_id, std::vector<float>& articular);
    static bool inverseKinematics(float x, float y, float z, std::string frame_id, std::vector<float>& articular);
    //static bool inverseKinematics(geometry_msgs::Pose& cartesian, std::vector<float>& articular);
    //static bool inverseKinematics(nav_msgs::Path& cartesianPath, std::vector<std::vector<float> >& articularPath);
    //static bool inverseKinematics(nav_msgs::Path& cartesianPath, std::vector<Float32MultiArray>& articularPath);
    static bool directKinematics(std::vector<float>& cartesian, std::vector<float>& articular);
};
