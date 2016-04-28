#include "justina_tools/JustinaManip.h"

bool JustinaManip::is_node_set = false;
ros::ServiceClient JustinaManip::cltIKFloatArray;
ros::ServiceClient JustinaManip::cltIKPath;
ros::ServiceClient JustinaManip::cltIKPose;

bool JustinaManip::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaManip::is_node_set)
        return true;
    if(nh == 0)
        return false;

    JustinaManip::cltIKFloatArray = nh->serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");
    JustinaManip::cltIKPath = nh->serviceClient<manip_msgs::InverseKinematicsPath>("/manipulation/ik_geometric/ik_path");
    JustinaManip::cltIKPose = nh->serviceClient<manip_msgs::InverseKinematicsPose>("/manipulation/ik_geometric/ik_pose");
    JustinaManip::is_node_set = true;
    return true;
}

bool JustinaManip::inverseKinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
    std::cout << "JustinaManip.->Calling service for inverse kinematics..." << std::endl;
    manip_msgs::InverseKinematicsFloatArray srv;
    srv.request.cartesian_pose.data = cartesian;
    bool success = JustinaManip::cltIKFloatArray.call(srv);
    articular = srv.response.articular_pose.data;
    return success;
}

bool JustinaManip::inverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::vector<float>& articular)
{
}

bool JustinaManip::inverseKinematics(float x, float y, float z, std::vector<float>& articular)
{
}

bool JustinaManip::inverseKinematics(std::vector<float>& cartesian, std::string frame_id, std::vector<float>& articular)
{
}

bool JustinaManip::inverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::string frame_id, std::vector<float>& articular)
{
}

bool JustinaManip::inverseKinematics(float x, float y, float z, std::string frame_id, std::vector<float>& articular)
{
}

// bool JustinaManip::inverseKinematics(geometry_msgs::Pose& cartesian, std::vector<float>& articular);
// bool JustinaManip::inverseKinematics(nav_msgs::Path& cartesianPath, std::vector<std::vector<float> >& articularPath);
// bool JustinaManip::inverseKinematics(nav_msgs::Path& cartesianPath, std::vector<Float32MultiArray>& articularPath);
bool JustinaManip::directKinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
}

