#include <iostream>
#include <vector>
#include "nav_msgs/Path.h"
#include "std_msgs/Float32MultiArray.h"

class InverseKinematics
{
public:
    static bool GetInverseKinematics(std::vector<float>& cartesian, std::vector<float>& articular);
    static bool GetInverseKinematics(geometry_msgs::Pose& cartesian, std::vector<float>& articular);
    static bool GetInverseKinematics(nav_msgs::Path& cartesianPath, std::vector<std_msgs::Float32MultiArray> articularPath);
};
