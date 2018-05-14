#include "justina_tools/JustinaManip.h"

bool JustinaManip::is_node_set = false;
tf::TransformListener * JustinaManip::tf_listener;
ros::ServiceClient JustinaManip::cltIKFloatArray;
ros::ServiceClient JustinaManip::cltIKPath;
ros::ServiceClient JustinaManip::cltIKPose;
ros::ServiceClient JustinaManip::cltDK;
//Subscribers for indicating that a goal pose has been reached
ros::Subscriber JustinaManip::subLaGoalReached;
ros::Subscriber JustinaManip::subRaGoalReached;
ros::Subscriber JustinaManip::subHdGoalReached;
ros::Subscriber JustinaManip::subTrGoalReached;
ros::Subscriber JustinaManip::subStopRobot;
ros::Subscriber JustinaManip::subObjOnRightHand;
ros::Subscriber JustinaManip::subObjOnLeftHand;
ros::Subscriber JustinaManip::subLaCurrentPos;
ros::Subscriber JustinaManip::subRaCurrentPos;
ros::Subscriber JustinaManip::subTorsoCurrentPos;
//Publishers for the commands executed by this node
ros::Publisher JustinaManip::pubLaGoToAngles;
ros::Publisher JustinaManip::pubRaGoToAngles;
ros::Publisher JustinaManip::pubHdGoToAngles;
ros::Publisher JustinaManip::pubLaGoToPoseWrtArm;
ros::Publisher JustinaManip::pubRaGoToPoseWrtArm;
ros::Publisher JustinaManip::pubLaGoToPoseWrtRobot;
ros::Publisher JustinaManip::pubRaGoToPoseWrtRobot;
ros::Publisher JustinaManip::pubLaGoToPoseWrtArmFeedback;
ros::Publisher JustinaManip::pubRaGoToPoseWrtArmFeedback;
ros::Publisher JustinaManip::pubLaGoToPoseWrtRobotFeedback;
ros::Publisher JustinaManip::pubRaGoToPoseWrtRobotFeedback;
ros::Publisher JustinaManip::pubLaStopGoTo;
ros::Publisher JustinaManip::pubRaStopGoTo;
ros::Publisher JustinaManip::pubLaGoToPoseWrtArmTraj;
ros::Publisher JustinaManip::pubRaGoToPoseWrtArmTraj;
ros::Publisher JustinaManip::pubLaGoToPoseWrtRobotTraj;
ros::Publisher JustinaManip::pubRaGoToPoseWrtRobotTraj;
ros::Publisher JustinaManip::pubLaGoToLoc;
ros::Publisher JustinaManip::pubRaGoToLoc;
ros::Publisher JustinaManip::pubHdGoToLoc;
ros::Publisher JustinaManip::pubLaMove;
ros::Publisher JustinaManip::pubRaMove;
ros::Publisher JustinaManip::pubHdMove;
ros::Publisher JustinaManip::pubLaCloseGripper;
ros::Publisher JustinaManip::pubRaCloseGripper;
ros::Publisher JustinaManip::pubLaOpenGripper;
ros::Publisher JustinaManip::pubRaOpenGripper;
ros::Publisher JustinaManip::pubTrGoToPose;
ros::Publisher JustinaManip::pubTrGoToRelPose;
//For moving up and down torso
ros::Publisher JustinaManip::pubTorsoUp;
ros::Publisher JustinaManip::pubTorsoDown;
//
bool JustinaManip::_isLaGoalReached = false;
bool JustinaManip::_isRaGoalReached = false;
bool JustinaManip::_isHdGoalReached = false;
bool JustinaManip::_isTrGoalReached = false;
bool JustinaManip::_isObjOnRightHand = false;
bool JustinaManip::_isObjOnLeftHand = false;
bool JustinaManip::_stopReceived = false;

std::vector<float> JustinaManip::_laCurrentPos;
std::vector<float> JustinaManip::_raCurrentPos;
std::vector<float> JustinaManip::_torsoCurrentPos;

bool JustinaManip::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaManip::is_node_set)
        return true;
    if(nh == 0)
        return false;
    std::cout << "JustinaManip.->Setting ros node..." << std::endl;
    JustinaManip::cltIKFloatArray = nh->serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");
    JustinaManip::cltIKPath = nh->serviceClient<manip_msgs::InverseKinematicsPath>("/manipulation/ik_geometric/ik_path");
    JustinaManip::cltIKPose = nh->serviceClient<manip_msgs::InverseKinematicsPose>("/manipulation/ik_geometric/ik_pose");
    JustinaManip::cltDK = nh->serviceClient<manip_msgs::DirectKinematics>("/manipulation/ik_geometric/direct_kinematics");
    //Subscribers for indicating that a goal pose has been reached
    JustinaManip::subLaGoalReached = nh->subscribe("/manipulation/la_goal_reached", 1, &JustinaManip::callbackLaGoalReached);
    JustinaManip::subRaGoalReached = nh->subscribe("/manipulation/ra_goal_reached", 1, &JustinaManip::callbackRaGoalReached);
    JustinaManip::subHdGoalReached = nh->subscribe("/manipulation/hd_goal_reached", 1, &JustinaManip::callbackHdGoalReached);
    JustinaManip::subTrGoalReached = nh->subscribe("/hardware/torso/goal_reached", 1, &JustinaManip::callbackTrGoalReached);
    JustinaManip::subObjOnRightHand = nh->subscribe("/hardware/right_arm/object_on_hand", 1, &JustinaManip::callbackObjOnRightHand);
    JustinaManip::subObjOnLeftHand = nh->subscribe("/hardware/left_arm/object_on_hand", 1, &JustinaManip::callbackObjOnLeftHand);
    JustinaManip::subLaCurrentPos = nh->subscribe("/hardware/left_arm/current_pose", 1, &JustinaManip::callbackLaCurrentPos);
    JustinaManip::subRaCurrentPos = nh->subscribe("/hardware/right_arm/current_pose", 1, &JustinaManip::callbackRaCurrentPos);
    JustinaManip::subTorsoCurrentPos = nh->subscribe("/hardware/torso/current_pose", 1, &JustinaManip::callbackTorsoCurrentPos);

    JustinaManip::subStopRobot = nh->subscribe("/hardware/robot_state/stop", 1, &JustinaManip::callbackRobotStop);
    //Publishers for the commands executed by this node
    JustinaManip::pubLaGoToAngles = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/la_goto_angles", 1);
    JustinaManip::pubRaGoToAngles = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/ra_goto_angles", 1);
    JustinaManip::pubHdGoToAngles = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/hd_goto_angles", 1);
    JustinaManip::pubLaGoToPoseWrtArm = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/la_pose_wrt_arm", 1);
    JustinaManip::pubRaGoToPoseWrtArm = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/ra_pose_wrt_arm", 1);
    JustinaManip::pubLaGoToPoseWrtRobot = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/la_pose_wrt_robot", 1);
    JustinaManip::pubRaGoToPoseWrtRobot = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/ra_pose_wrt_robot", 1);
    JustinaManip::pubLaGoToPoseWrtArmFeedback = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/la_pose_wrt_arm_feedback", 1);
    JustinaManip::pubRaGoToPoseWrtArmFeedback = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/ra_pose_wrt_arm_feedback", 1);
    JustinaManip::pubLaGoToPoseWrtRobotFeedback = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/la_pose_wrt_robot_feedback", 1);
    JustinaManip::pubRaGoToPoseWrtRobotFeedback = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/ra_pose_wrt_robot_feedback", 1);
    JustinaManip::pubLaGoToPoseWrtArmTraj = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/la_pose_wrt_arm_traj", 1);
    JustinaManip::pubRaGoToPoseWrtArmTraj = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/ra_pose_wrt_arm_traj", 1);
    JustinaManip::pubLaGoToPoseWrtRobotTraj = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/la_pose_wrt_robot_traj", 1);
    JustinaManip::pubRaGoToPoseWrtRobotTraj = nh->advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/ra_pose_wrt_robot_traj", 1);
    JustinaManip::pubLaStopGoTo = nh->advertise<std_msgs::Empty>("/manipulation/manip_pln/la_stop", 1);
    JustinaManip::pubRaStopGoTo = nh->advertise<std_msgs::Empty>("/manipulation/manip_pln/ra_stop", 1);
    JustinaManip::pubLaGoToLoc = nh->advertise<std_msgs::String>("/manipulation/manip_pln/la_goto_loc", 1);
    JustinaManip::pubRaGoToLoc = nh->advertise<std_msgs::String>("/manipulation/manip_pln/ra_goto_loc", 1);
    JustinaManip::pubHdGoToLoc = nh->advertise<std_msgs::String>("/manipulation/manip_pln/hd_goto_loc", 1);
    JustinaManip::pubLaMove = nh->advertise<std_msgs::String>("/manipulation/manip_pln/la_move", 1);
    JustinaManip::pubRaMove = nh->advertise<std_msgs::String>("/manipulation/manip_pln/la_move", 1);
    JustinaManip::pubHdMove = nh->advertise<std_msgs::String>("/manipulation/manip_pln/la_move", 1);
    JustinaManip::pubLaCloseGripper = nh->advertise<std_msgs::Float32>("/hardware/left_arm/torque_gripper", 1);
    JustinaManip::pubLaOpenGripper = nh->advertise<std_msgs::Float32>("/hardware/left_arm/goal_gripper", 1);
    JustinaManip::pubRaCloseGripper = nh->advertise<std_msgs::Float32>("/hardware/right_arm/torque_gripper", 1);
    JustinaManip::pubRaOpenGripper = nh->advertise<std_msgs::Float32>("/hardware/right_arm/goal_gripper", 1);
    JustinaManip::pubTrGoToPose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/torso/goal_pose", 1);
    JustinaManip::pubTrGoToRelPose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/torso/goal_rel_pose", 1);

    JustinaManip::is_node_set = true;
    JustinaManip::tf_listener = new tf::TransformListener();
    JustinaManip::tf_listener->waitForTransform("base_link", "right_arm_grip_center", ros::Time(0), ros::Duration(10.0));
    JustinaManip::tf_listener->waitForTransform("base_link", "left_arm_grip_center", ros::Time(0), ros::Duration(10.0));

    //For moving up and down torso
    JustinaManip::pubTorsoUp   = nh->advertise<std_msgs::String>("/hardware/torso/torso_up", 1);
    JustinaManip::pubTorsoDown = nh->advertise<std_msgs::String>("/hardware/torso/torso_down", 1);

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

bool JustinaManip::isTorsoGoalReached()
{
    return JustinaManip::_isTrGoalReached;
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


bool JustinaManip::waitForTorsoGoalReached(int timeOut_ms)
{
    int attempts = timeOut_ms / 100;
    ros::Rate loop(10);
    JustinaManip::_stopReceived = false;
    JustinaManip::_isTrGoalReached = false;
    while(ros::ok() && !JustinaManip::_isTrGoalReached && !JustinaManip::_stopReceived && attempts-- >= 0)
    {
        ros::spinOnce();
        loop.sleep();
    }
    JustinaManip::_stopReceived = false; //This flag is set True in the subscriber callback
    return JustinaManip::_isTrGoalReached;
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
    return false;
}

bool JustinaManip::inverseKinematics(float x, float y, float z, std::vector<float>& articular)
{
    return false;
}

bool JustinaManip::inverseKinematics(std::vector<float>& cartesian, std::string frame_id, std::vector<float>& articular)
{
    return false;
}

bool JustinaManip::inverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::string frame_id, std::vector<float>& articular)
{
    return false;
}

bool JustinaManip::inverseKinematics(float x, float y, float z, std::string frame_id, std::vector<float>& articular)
{
    return false;
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

void JustinaManip::startLaGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    JustinaManip::pubLaGoToPoseWrtArm.publish(msg);
}

void JustinaManip::startLaGoToCartesian(float x, float y, float z)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
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

void JustinaManip::startLaGoToCartesianFeedback(std::vector<float>& cartesian){
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubLaGoToPoseWrtArmFeedback.publish(msg);
}

void JustinaManip::startLaGoToCartesianFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow){
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubLaGoToPoseWrtArmFeedback.publish(msg);
}

void JustinaManip::startLaGoToCartesianFeedback(float x, float y, float z){
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    JustinaManip::pubLaGoToPoseWrtArmFeedback.publish(msg);
}

void JustinaManip::startLaGoToCartesianWrtRobotFeedback(std::vector<float>& cartesian){
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubLaGoToPoseWrtRobotFeedback.publish(msg);
}

void JustinaManip::startLaGoToCartesianWrtRobotFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow){
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubLaGoToPoseWrtRobotFeedback.publish(msg);
}

void JustinaManip::startLaGoToCartesianTraj(std::vector<float>& cartesian){
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubLaGoToPoseWrtArmTraj.publish(msg);
}

void JustinaManip::startLaGoToCartesianTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow){
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubLaGoToPoseWrtArmTraj.publish(msg);
}

void JustinaManip::startLaGoToCartesianTraj(float x, float y, float z){
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    JustinaManip::pubLaGoToPoseWrtArmTraj.publish(msg);
}

void JustinaManip::startLaGoToCartesianWrtRobotTraj(std::vector<float>& cartesian){
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubLaGoToPoseWrtRobotTraj.publish(msg);
}

void JustinaManip::startLaGoToCartesianWrtRobotTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow){
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubLaGoToPoseWrtRobotTraj.publish(msg);
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

void JustinaManip::startLaOpenGripper(float angle)
{
    std_msgs::Float32 msgAngle;
    msgAngle.data = angle;
    JustinaManip::pubLaOpenGripper.publish(msgAngle);
}


void JustinaManip::startLaCloseGripper(float torque)
{
   std_msgs::Float32 msgTorque;
   msgTorque.data = torque;
   JustinaManip::pubLaCloseGripper.publish(msgTorque);
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

void JustinaManip::startRaGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    JustinaManip::pubRaGoToPoseWrtArm.publish(msg);
}

void JustinaManip::startRaGoToCartesian(float x, float y, float z)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
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

void JustinaManip::startRaGoToCartesianFeedback(std::vector<float>& cartesian)
{
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubRaGoToPoseWrtArmFeedback.publish(msg);
}

void JustinaManip::startRaGoToCartesianFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubRaGoToPoseWrtArmFeedback.publish(msg);
}

void JustinaManip::startRaGoToCartesianFeedback(float x, float y, float z)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    JustinaManip::pubRaGoToPoseWrtArmFeedback.publish(msg);
}

void JustinaManip::startRaGoToCartesianWrtRobotFeedback(std::vector<float>& cartesian)
{
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubRaGoToPoseWrtRobotFeedback.publish(msg);
}

void JustinaManip::startRaGoToCartesianWrtRobotFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubRaGoToPoseWrtRobotFeedback.publish(msg);
}

void JustinaManip::startRaGoToCartesianTraj(std::vector<float>& cartesian)
{
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubRaGoToPoseWrtArmTraj.publish(msg);
}

void JustinaManip::startRaGoToCartesianTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubRaGoToPoseWrtArmTraj.publish(msg);
}

void JustinaManip::startRaGoToCartesianTraj(float x, float y, float z)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    JustinaManip::pubRaGoToPoseWrtArmTraj.publish(msg);
}

void JustinaManip::startRaGoToCartesianWrtRobotTraj(std::vector<float>& cartesian)
{
    std_msgs::Float32MultiArray msg;
    msg.data = cartesian;
    JustinaManip::pubRaGoToPoseWrtRobotTraj.publish(msg);
}

void JustinaManip::startRaGoToCartesianWrtRobotTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(z);
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);
    msg.data.push_back(elbow);
    JustinaManip::pubRaGoToPoseWrtRobotTraj.publish(msg);
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

void JustinaManip::startRaOpenGripper(float angle)
{
    std_msgs::Float32 msgAngle;
    msgAngle.data = angle;
    JustinaManip::pubRaOpenGripper.publish(msgAngle);
}


void JustinaManip::startRaCloseGripper(float torque)
{
   std_msgs::Float32 msgTorque;
   msgTorque.data = torque;
   JustinaManip::pubRaCloseGripper.publish(msgTorque);
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

void JustinaManip::startTorsoGoTo(float goalSpine, float goalWaist, float goalShoulders)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(goalSpine);
    msg.data.push_back(goalWaist);
    msg.data.push_back(goalShoulders);
    JustinaManip::pubTrGoToPose.publish(msg);
}

void JustinaManip::startTorsoGoToRel(float goalRelSpine, float goalRelWaist, float goalRelShoulders)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(goalRelSpine);
    msg.data.push_back(goalRelWaist);
    msg.data.push_back(goalRelShoulders);
    JustinaManip::pubTrGoToRelPose.publish(msg);
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

bool JustinaManip::laGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, int timeOut_ms)
{
    JustinaManip::startLaGoToCartesian(x, y, z, roll, pitch, yaw);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::laGoToCartesian(float x, float y, float z, int timeOut_ms)
{
    JustinaManip::startLaGoToCartesian(x, y, z);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::laGoToCartesianFeedback(std::vector<float>& cartesian, int timeOut_ms)
{
    JustinaManip::startLaGoToCartesianFeedback(cartesian);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::laGoToCartesianFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms)
{
    JustinaManip::startLaGoToCartesianFeedback(x, y, z, roll, pitch, yaw, elbow);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::laGoToCartesianFeedback(float x, float y, float z, int timeOut_ms)
{
    JustinaManip::startLaGoToCartesianFeedback(x, y, z);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

void JustinaManip::laStopGoToCartesian(){
    std_msgs::Empty msg;
    JustinaManip::pubLaStopGoTo.publish(msg);
}

bool JustinaManip::laGoToCartesianTraj(std::vector<float>& cartesian, int timeOut_ms)
{
    JustinaManip::startLaGoToCartesianTraj(cartesian);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::laGoToCartesianTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms)
{
    JustinaManip::startLaGoToCartesianTraj(x, y, z, roll, pitch, yaw, elbow);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::laGoToCartesianTraj(float x, float y, float z, int timeOut_ms)
{
    JustinaManip::startLaGoToCartesianTraj(x, y, z);
    return JustinaManip::waitForLaGoalReached(timeOut_ms);
}

bool JustinaManip::laGoTo(std::string location, int timeOut_ms)
{
    if(location == "navigation" && JustinaManip::isLaInPredefPos("home"))
    {
        JustinaManip::startLaGoTo("pre_nav");
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
    }
    else if (location == "home" && JustinaManip::isLaInPredefPos("navigation"))
    {
        JustinaManip::startLaGoTo("pre_nav");
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
    }
    
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

bool JustinaManip::raGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, int timeOut_ms)
{
    JustinaManip::startRaGoToCartesian(x, y, z, roll, pitch, yaw);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::raGoToCartesian(float x, float y, float z, int timeOut_ms)
{
    JustinaManip::startRaGoToCartesian(x, y, z);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::raGoToCartesianFeedback(std::vector<float>& cartesian, int timeOut_ms)
{
    JustinaManip::startRaGoToCartesianFeedback(cartesian);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::raGoToCartesianFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms)
{
    JustinaManip::startRaGoToCartesianFeedback(x, y, z, roll, pitch, yaw, elbow);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::raGoToCartesianFeedback(float x, float y, float z, int timeOut_ms)
{
    JustinaManip::startRaGoToCartesianFeedback(x, y, z);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

void JustinaManip::raStopGoToCartesian(){
    std_msgs::Empty msg;
    JustinaManip::pubRaStopGoTo.publish(msg);
}

bool JustinaManip::raGoToCartesianTraj(std::vector<float>& cartesian, int timeOut_ms)
{
    JustinaManip::startRaGoToCartesianTraj(cartesian);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::raGoToCartesianTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms)
{
    JustinaManip::startRaGoToCartesianTraj(x, y, z, roll, pitch, yaw, elbow);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::raGoToCartesianTraj(float x, float y, float z, int timeOut_ms)
{
    JustinaManip::startRaGoToCartesianTraj(x, y, z);
    return JustinaManip::waitForRaGoalReached(timeOut_ms);
}

bool JustinaManip::raGoTo(std::string location, int timeOut_ms)
{
    if(location == "navigation" && JustinaManip::isRaInPredefPos("home"))
    {
        JustinaManip::startRaGoTo("pre_nav");
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
    }
    else if (location == "home" && JustinaManip::isRaInPredefPos("navigation"))
    {
        JustinaManip::startRaGoTo("pre_nav");
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
    }

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

bool JustinaManip::torsoGoTo(float goalSpine, float goalWaist, float goalShoulders, int timeOut_ms)
{
    JustinaManip::startTorsoGoTo(goalSpine, goalWaist, goalShoulders);
    return JustinaManip::waitForTorsoGoalReached(timeOut_ms);
}

bool JustinaManip::torsoGoToRel(float goalRelSpine, float goalRelWaist, float goalRelShoulders, int timeOut_ms)
{
    JustinaManip::startTorsoGoToRel(goalRelSpine, goalRelWaist, goalRelShoulders);
    return JustinaManip::waitForTorsoGoalReached(timeOut_ms);
}

bool JustinaManip::objOnRightHand(){
    return _isObjOnRightHand;
}

bool JustinaManip::objOnLeftHand(){
    return _isObjOnLeftHand;
}


void JustinaManip::getRightHandPosition(float &x, float &y, float &z){
	tf::StampedTransform transform;
	tf_listener->waitForTransform("base_link", "right_arm_grip_center", ros::Time(0), ros::Duration(10.0));
	tf_listener->lookupTransform("base_link", "right_arm_grip_center", ros::Time(0), transform);
	x = transform.getOrigin().getX();
	y = transform.getOrigin().getY();
	z = transform.getOrigin().getZ();
	std::cout << "JustinaManip.->right_arm_griper_center.x:" << x << std::endl;
	std::cout << "JustinaManip.->right_arm_griper_center.y:" << y << std::endl;
	std::cout << "JustinaManip.->right_arm_griper_center.z:" << z << std::endl;
}

void JustinaManip::getLeftHandPosition(float &x, float &y, float &z){
	tf::StampedTransform transform;
	tf_listener->waitForTransform("base_link", "left_arm_grip_center", ros::Time(0), ros::Duration(10.0));
	tf_listener->lookupTransform("base_link", "left_arm_grip_center", ros::Time(0), transform);
	x = transform.getOrigin().getX();
	y = transform.getOrigin().getY();
	z = transform.getOrigin().getZ();
	std::cout << "JustinaManip.->left_arm_griper_center.x:" << x << std::endl;
	std::cout << "JustinaManip.->left_arm_griper_center.y:" << y << std::endl;
	std::cout << "JustinaManip.->left_arm_griper_center.z:" << z << std::endl;
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

void JustinaManip::callbackTrGoalReached(const std_msgs::Bool::ConstPtr& msg)
{
    JustinaManip::_isTrGoalReached = msg->data;
}

void JustinaManip::callbackObjOnRightHand(const std_msgs::Bool::ConstPtr& msg)
{
    JustinaManip::_isObjOnRightHand = msg->data;
}

void JustinaManip::callbackObjOnLeftHand(const std_msgs::Bool::ConstPtr& msg)
{
    JustinaManip::_isObjOnLeftHand = msg->data;
}


void JustinaManip::callbackLaCurrentPos(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "La pose received" << std::endl;
    JustinaManip::_laCurrentPos = msg->data;
    //std::cout << "LA current_pos: " << JustinaManip::_laCurrentPos.size() << std::endl;
}

void JustinaManip::callbackRaCurrentPos(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "Ra pose received" << std::endl;
    JustinaManip::_raCurrentPos = msg->data;
    //std::cout << "RA current_pos: " << JustinaManip::_raCurrentPos.size() << std::endl;
}

void JustinaManip::callbackTorsoCurrentPos(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "Torso pose received:  ";
    JustinaManip::_torsoCurrentPos = msg->data;
    //std::cout << JustinaManip::_torsoCurrentPos << std::endl;
}

void JustinaManip::getLaCurrentPos(std::vector<float>& pos)
{
    //std::cout << "LA current_pos: " << JustinaManip::_laCurrentPos.size() << std::endl;
    pos = JustinaManip::_laCurrentPos;
}

void JustinaManip::getRaCurrentPos(std::vector<float>& pos)
{
    //std::cout << "RA current_pos: " << JustinaManip::_raCurrentPos.size() << std::endl;
    pos = JustinaManip::_raCurrentPos;
}

void JustinaManip::getTorsoCurrentPos(std::vector<float>& pos)
{
    pos = JustinaManip::_torsoCurrentPos;
}

bool JustinaManip::isLaInPredefPos(std::string id)
{
    float threshold = 0.2;
    std::vector<float> poses;

    JustinaKnowledge::getPredLaArmPose(id, poses);

    if(poses.size() < 1)
        return false;

    //std::cout << "JustinaManip::isLaInPredefPos.->  pose_size:  " << JustinaManip::_laCurrentPos.size() << " - " << JustinaManip::_laCurrentPos[0] << std::endl;
    
    std::cout << "JustinaManip::isLaInPredefPos      current_pos" << std::endl;
    for(int i = 0; i < poses.size(); i++)
    {
        //std::cout << "I'm into the for...." << std::endl;
        //std::cout << "                     " << poses[i] << "             " << JustinaManip::_laCurrentPos[i] << std::endl;
        if(poses[i] > JustinaManip::_laCurrentPos[i]-threshold && poses[i] < JustinaManip::_laCurrentPos[i]+threshold )
        {   
            continue;
        }
        else
        {
            std::cout << "                     " << poses[i] << "             " << JustinaManip::_laCurrentPos[i] << std::endl;
            std::cout << "Left arm is NOT in  " << id << "  pose" << std::endl;
            return false;
        }
    }

    std::cout << "Left arm is Already in  " << id << "  pose" << std::endl;
    return true;
}

bool JustinaManip::isRaInPredefPos(std::string id)
{
    float threshold = 0.2;
    std::vector<float> poses;


    JustinaKnowledge::getPredRaArmPose(id, poses);
    if(poses.size() < 1)
        return false;

    //std::cout << "JustinaManip::isRaInPredefPos.->  pose_size:  " << poses.size() << " - " << poses[0] << std::endl;

    std::cout << "JustinaManip::isRaInPredefPos      current_pos" << std::endl;
    for(int i = 0; i < poses.size(); i++)
    {
        //std::cout << "I'm into the for...." << std::endl;
        //std::cout << "                    " << poses[i] << "              " << JustinaManip::_raCurrentPos[i] << std::endl;
        if(poses[i] > JustinaManip::_raCurrentPos[i]-threshold && poses[i] < JustinaManip::_raCurrentPos[i]+threshold )
        {
            continue;
        }
        else
        {
            std::cout << "                    " << poses[i] << "              " << JustinaManip::_raCurrentPos[i] << std::endl;
            std::cout << "Right arm is NOT in  " << id << "  pose" << std::endl;
            return false;
        }
    }

    std::cout << "Right arm is Already in  " << id << "  pose" << std::endl;
    return true;
}

//Methods for moving torso up or down
void JustinaManip::moveTorsoUp(std_msgs::String msg)
{
    JustinaManip::pubTorsoUp.publish(msg);
}

void JustinaManip::moveTorsoDown(std_msgs::String msg)
{
    JustinaManip::pubTorsoDown.publish(msg);

}
