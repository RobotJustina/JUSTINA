#include <iostream>
#include <vector>
#include <cmath>
#include "std_msgs/Float32MultiArray.h"
#include "tf/transform_broadcaster.h"

class InverseKinematics
{
public:
    static bool GetInverseKinematics(std::vector<float>& cartesian, std::vector<float>& articular);
    static bool GetInverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::vector<float>& articular);
    static bool GetInverseKinematics(float x, float y, float z, std::vector<float>& articular);
    static bool GetDirectKinematics(std::vector<float>& articular, std::vector<float>& cartesian);
};

