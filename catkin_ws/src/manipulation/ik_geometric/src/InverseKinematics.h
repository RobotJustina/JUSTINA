#include <iostream>
#include <vector>
#include <cmath>
#include "nav_msgs/Path.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf/transform_broadcaster.h"

class InverseKinematics
{
public:
    static bool GetInverseKinematics(std::vector<float>& cartesian, std::vector<float>& articular);
    static bool GetInverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, float elbow, std::vector<float>& articular);
    static bool GetInverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::vector<float>& articular);
    static bool GetInverseKinematics(float x, float y, float z, std::vector<float>& articular);
    static bool GetInverseKinematicsWithoutOptimization(float x, float y, float z, std::vector<float>& articular);
    static bool GetInverseKinematics(geometry_msgs::Pose& cartesian_pose, std::vector<float>& articular_pose);
    static bool GetInverseKinematics(nav_msgs::Path& cartesianPath, std::vector<std_msgs::Float32MultiArray> articularPath);
    static bool GetDirectKinematics(std::vector<float>& articular, std::vector<float>& cartesian);
};
