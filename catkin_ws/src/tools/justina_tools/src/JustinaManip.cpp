#include "justina_tools/JustinaManip.h"

bool JustinaManip::is_node_set = false;
ros::ServiceClient JustinaManip::cltIKFloatArray;
ros::ServiceClient JustinaManip::cltIKPath;
ros::ServiceClient JustinaManip::cltIKPose;
ros::ServiceClient JustinaManip::cltDK;
//Subscribers for indicating that a goal pose has been reached
ros::Subscriber JustinaManip::subLaGoalReached;
ros::Subscriber JustinaManip::subRaGoalReached;
ros::Subscriber JustinaManip::subHdGoalReached;
ros::Subscriber JustinaManip::subStopRobot;
//Publishers for the commands executed by this node
ros::Publisher JustinaManip::pubLaGoToAngles;
ros::Publisher JustinaManip::pubRaGoToAngles;
ros::Publisher JustinaManip::pubHdGoToAngles;
ros::Publisher JustinaManip::pubLaGoToPoseWrtArm;
ros::Publisher JustinaManip::pubRaGoToPoseWrtArm;
ros::Publisher JustinaManip::pubLaGoToPoseWrtRobot;
ros::Publisher JustinaManip::pubRaGoToPoseWrtRobot;
ros::Publisher JustinaManip::pubLaGoToLoc;
ros::Publisher JustinaManip::pubRaGoToLoc;
ros::Publisher JustinaManip::pubHdGoToLoc;
ros::Publisher JustinaManip::pubLaMove;
ros::Publisher JustinaManip::pubRaMove;
ros::Publisher JustinaManip::pubHdMove;
//
bool JustinaManip::_isLaGoalReached = false;
bool JustinaManip::_isRaGoalReached = false;
bool JustinaManip::_isHdGoalReached = false;
bool JustinaManip::_stopReceived = false;

bool JustinaManip::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaManip::is_node_set)
        return true;
    if(nh == 0)
        return false;

    JustinaManip::cltIKFloatArray = nh->serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");
    JustinaManip::cltIKPath = nh->serviceClient<manip_msgs::InverseKinematicsPath>("/manipulation/ik_geometric/ik_path");
    JustinaManip::cltIKPose = nh->serviceClient<manip_msgs::InverseKinematicsPose>("/manipulation/ik_geometric/ik_pose");
    JustinaManip::cltDK = nh->serviceClient<manip_msgs::DirectKinematics>("/manipulation/ik_geometric/direct_kinematics");
    //Subscribers for indicating that a goal pose has been reached
    JustinaManip::subLaGoalReached = nh->subscribe("/manipulation/la_goal_reached", 1, &JustinaManip::callbackLaGoalReached);
    JustinaManip::subRaGoalReached = nh->subscribe("/manipulation/ra_goal_reached", 1, &JustinaManip::callbackRaGoalReached);
    JustinaManip::subHdGoalReached = nh->subscribe("/manipulation/hd_goal_reached", 1, &JustinaManip::callbackHdGoalReached);
    JustinaManip::subStopRobot = nh->subscribe("/hardware/robot_state/stop", 1, &JustinaManip::callbackRobotStop);
    //Publishers for the commands executed by this node
    JustinaManip::pubLaGoToAngles = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/la_goto_angles", 1);
    JustinaManip::pubRaGoToAngles = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/ra_goto_angles", 1);
    JustinaManip::pubHdGoToAngles = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/hd_goto_angles", 1);
    JustinaManip::pubLaGoToPoseWrtArm = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/la_pose_wrt_arm", 1);
    JustinaManip::pubRaGoToPoseWrtArm = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/ra_pose_wrt_arm", 1);
    JustinaManip::pubLaGoToPoseWrtRobot = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/la_pose_wrt_robot", 1);
    JustinaManip::pubRaGoToPoseWrtRobot = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/ra_pose_wrt_robot", 1);
    JustinaManip::pubLaGoToLoc = nh->advertise<std_msgs::String>("/manipulation/manip_pln/la_goto_loc", 1);
    JustinaManip::pubRaGoToLoc = nh->advertise<std_msgs::String>("/manipulation/manip_pln/ra_goto_loc", 1);
    JustinaManip::pubHdGoToLoc = nh->advertise<std_msgs::String>("/manipulation/manip_pln/hd_goto_loc", 1);
    JustinaManip::pubLaMove = nh->advertise<std_msgs::String>("/manipulation/manip_pln/la_move", 1);
    JustinaManip::pubRaMove = nh->advertise<std_msgs::String>("/manipulation/manip_pln/la_move", 1);
    JustinaManip::pubHdMove = nh->advertise<std_msgs::String>("/manipulation/manip_pln/la_move", 1);
    
    JustinaManip::is_node_set = true;
    return true;
}

bool JustinaManip::isLaGoalReached()
{
    return JustinaManip::_isLaGoalReached;
}

bool JustinaManip::isRaGoalReached()
{
    return JustinaManip::_isRaGoalReached;
}

bool JustinaManip::isHdGoalReached()
{
    //std::cout << "JustinaManip.-> isHdGoalReached: " << JustinaManip::_isHdGoalReached << std::endl;
    return JustinaManip::_isHdGoalReached;
}

bool JustinaManip::waitForLaGoalReached(int timeOut_ms)
{
    int attempts = timeOut_ms / 100;
    ros::Rate loop(10);
    JustinaManip::_stopReceived = false;
    JustinaManip::_isLaGoalReached = false;
    while(ros::ok() && !JustinaManip::_isLaGoalReached && !JustinaManip::_stopReceived && attempts-- >= 0)
    {
        ros::spinOnce();
        loop.sleep();
    }
    JustinaManip::_stopReceived = false; //This flag is set True in the subscriber callback
    return JustinaManip::_isLaGoalReached;
}

bool JustinaManip::waitForRaGoalReached(int timeOut_ms)
{
    int attempts = timeOut_ms / 100;
    ros::Rate loop(10);
    JustinaManip::_stopReceived = false;
    JustinaManip::_isRaGoalReached = false;
    while(ros::ok() && !JustinaManip::_isRaGoalReached && !JustinaManip::_stopReceived && attempts-- >= 0)
    {
        ros::spinOnce();
        loop.sleep();
    }
    JustinaManip::_stopReceived = false; //This flag is set True in the subscriber callback
    return JustinaManip::_isRaGoalReached;
}

bool JustinaManip::waitForHdGoalReached(int timeOut_ms)
{
    int attempts = timeOut_ms / 100;
    ros::Rate loop(10);
    JustinaManip::_stopReceived = false;
    JustinaManip::_isHdGoalReached = false;
    while(ros::ok() && !JustinaManip::_isHdGoalReached && !JustinaManip::_stopReceived && attempts-- >= 0)
    {
        ros::spinOnce();
        loop.sleep();
    }
    JustinaManip::_stopReceived = false; //This flag is set True in the subscriber callback
    return JustinaManip::_isHdGoalReached;
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
    std::cout << "JustinaManip.->Calling service for direct kinematics..." << std::endl;
    manip_msgs::DirectKinematics srv;
    srv.request.articular_pose.data = articular;
    bool success = JustinaManip::cltDK.call(srv);
    cartesian = srv.response.cartesian_pose.data;
    return success;
}

//
//Methods for operating arms and head
//
void JustinaManip::startLaGoToArticular(std::vector<float>& articular)
{
    std_msgs::Float32MultiArray msg;
    msg.data = articular;
    JustinaManip::pubLaGoToAngles.publish(msg);
}

void JustinaManip::startLaGoToCartesian(std::vector<float>& cartesian)
{
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubLaGoToPoseWrtArm.publish(msg);
}

void JustinaManip::startLaGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubLaGoToPoseWrtArm.publish(msg);
}

void JustinaManip::startLaGoToCartesianWrtRobot(std::vector<float>& cartesian)
{
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubLaGoToPoseWrtRobot.publish(msg);
}

void JustinaManip::startLaGoToCartesianWrtRobot(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubLaGoToPoseWrtRobot.publish(msg);
}

void JustinaManip::startLaGoTo(std::string location)
{
    std_msgs::String msg;
    msg.data = location;
    JustinaManip::pubLaGoToLoc.publish(msg);
}

void JustinaManip::startLaMove(std::string movement)
{
    std_msgs::String msg;
    msg.data = movement;
    JustinaManip::pubLaMove.publish(msg);
}

void JustinaManip::startRaGoToArticular(std::vector<float>& articular)
{
    std_msgs::Float32MultiArray msg;
    msg.data = articular;
    JustinaManip::pubRaGoToAngles.publish(msg);
}

void JustinaManip::startRaGoToCartesian(std::vector<float>& cartesian)
{
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubRaGoToPoseWrtArm.publish(msg);
}

void JustinaManip::startRaGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubRaGoToPoseWrtArm.publish(msg);
}

void JustinaManip::startRaGoToCartesianWrtRobot(std::vector<float>& cartesian)
{
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubRaGoToPoseWrtRobot.publish(msg);
}

void JustinaManip::startRaGoToCartesianWrtRobot(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubRaGoToPoseWrtRobot.publish(msg);
}

void JustinaManip::startRaGoTo(std::string location)
{
    std_msgs::String msg;
    msg.data = location;
    JustinaManip::pubRaGoToLoc.publish(msg);
}

void JustinaManip::startRaMove(std::string movement)
{
    std_msgs::String msg;
    msg.data = movement;
    JustinaManip::pubRaMove.publish(msg);
}

void JustinaManip::startHdGoTo(float pan, float tilt)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(pan);
    msg.data.push_back(tilt);
    JustinaManip::pubHdGoToAngles.publish(msg);
}

void JustinaManip::startHdGoTo(std::string location)
{
    std_msgs::String msg;
    msg.data = location;
    JustinaManip::pubHdGoToLoc.publish(msg);
}

void JustinaManip::startHdMove(std::string movement)
{
    std_msgs::String msg;
    msg.data = movement;
    JustinaManip::pubHdMove.publish(msg);
}

bool JustinaManip::laGoToArticular(std::vector<float>& articular, int timeOut_ms)
{
    JustinaManip::startLaGoToArticular(articular);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::laGoToCartesian(std::vector<float>& cartesian, int timeOut_ms)
{
    JustinaManip::startLaGoToCartesian(cartesian);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::laGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms)
{
    JustinaManip::startLaGoToCartesian(x, y, z, roll, pitch, yaw, elbow);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::laGoTo(std::string location, int timeOut_ms)
{
    JustinaManip::startLaGoTo(location);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::laMove(std::string movement, int timeOut_ms)
{
    JustinaManip::startLaMove(movement);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::raGoToArticular(std::vector<float>& articular, int timeOut_ms)
{
    JustinaManip::startRaGoToArticular(articular);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::raGoToCartesian(std::vector<float>& cartesian, int timeOut_ms)
{
    JustinaManip::startRaGoToCartesian(cartesian);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::raGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms)
{
    JustinaManip::startRaGoToCartesian(x, y, z, roll, pitch, yaw, elbow);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::raGoTo(std::string location, int timeOut_ms)
{
    JustinaManip::startRaGoTo(location);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::raMove(std::string movement, int timeOut_ms)
{
    JustinaManip::startRaMove(movement);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::hdGoTo(float pan, float tilt, int timeOut_ms)
{
    JustinaManip::startHdGoTo(pan, tilt);
    return JustinaManip::waitForHdGoalReached(timeOut_ms);
}

bool JustinaManip::hdGoTo(std::string location, int timeOut_ms)
{
    JustinaManip::startHdGoTo(location);
    return JustinaManip::waitForHdGoalReached(timeOut_ms);
}

bool JustinaManip::hdMove(std::string movement, int timeOut_ms)
{
    JustinaManip::startHdMove(movement);
    return JustinaManip::waitForHdGoalReached(timeOut_ms);
}

//
//Callbacks for catching goal-reached signals
//
void JustinaManip::callbackRobotStop(const std_msgs::Empty::ConstPtr& msg)
{
    JustinaManip::_stopReceived = true;
}

void JustinaManip::callbackLaGoalReached(const std_msgs::Bool::ConstPtr& msg)
{
    JustinaManip::_isLaGoalReached = msg->data;
}

void JustinaManip::callbackRaGoalReached(const std_msgs::Bool::ConstPtr& msg)
{
    JustinaManip::_isRaGoalReached = msg->data;
}

void JustinaManip::callbackHdGoalReached(const std_msgs::Bool::ConstPtr& msg)
{
    JustinaManip::_isHdGoalReached = msg->data;
}
