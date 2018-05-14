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
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "justina_tools/JustinaKnowledge.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"
#include "manip_msgs/InverseKinematicsPath.h"
#include "manip_msgs/InverseKinematicsPose.h"
#include "manip_msgs/DirectKinematics.h"

class JustinaManip
{

private:
    static bool is_node_set;
    static tf::TransformListener * tf_listener;
    static ros::ServiceClient cltIKFloatArray;
    static ros::ServiceClient cltIKPath;
    static ros::ServiceClient cltIKPose;
    static ros::ServiceClient cltDK;

    //Publishers for indicating that a goal pose has been reached
    static ros::Subscriber subLaGoalReached;
    static ros::Subscriber subRaGoalReached;
    static ros::Subscriber subLaCurrentPos;
    static ros::Subscriber subRaCurrentPos;
    static ros::Subscriber subTorsoCurrentPos;
    static ros::Subscriber subHdGoalReached;
    static ros::Subscriber subTrGoalReached;
    static ros::Subscriber subObjOnRightHand;
    static ros::Subscriber subObjOnLeftHand;
    static ros::Subscriber subStopRobot;
    //Subscribers for the commands executed by this node
    static ros::Publisher pubLaGoToAngles;
    static ros::Publisher pubRaGoToAngles;
    static ros::Publisher pubHdGoToAngles;
    static ros::Publisher pubLaGoToPoseWrtArm;
    static ros::Publisher pubRaGoToPoseWrtArm;
    static ros::Publisher pubLaGoToPoseWrtRobot;
    static ros::Publisher pubRaGoToPoseWrtRobot;
    static ros::Publisher pubLaGoToPoseWrtArmFeedback;
    static ros::Publisher pubRaGoToPoseWrtArmFeedback;
    static ros::Publisher pubLaGoToPoseWrtRobotFeedback;
    static ros::Publisher pubRaGoToPoseWrtRobotFeedback;
    static ros::Publisher pubLaStopGoTo;
    static ros::Publisher pubRaStopGoTo;
    static ros::Publisher pubLaGoToPoseWrtArmTraj;
    static ros::Publisher pubRaGoToPoseWrtArmTraj;
    static ros::Publisher pubLaGoToPoseWrtRobotTraj;
    static ros::Publisher pubRaGoToPoseWrtRobotTraj;
    static ros::Publisher pubLaGoToLoc;
    static ros::Publisher pubRaGoToLoc;
    static ros::Publisher pubHdGoToLoc;
    static ros::Publisher pubLaMove;
    static ros::Publisher pubRaMove;
    static ros::Publisher pubHdMove;
    static ros::Publisher pubLaCloseGripper;
    static ros::Publisher pubRaCloseGripper;
    static ros::Publisher pubLaOpenGripper;
    static ros::Publisher pubRaOpenGripper;
    static ros::Publisher pubTrGoToPose;
    static ros::Publisher pubTrGoToRelPose;
    //For moving up and down torso
    static ros::Publisher pubTorsoUp;
    static ros::Publisher pubTorsoDown;


    static bool _isLaGoalReached;
    static bool _isRaGoalReached;
    static bool _isHdGoalReached;
    static bool _isTrGoalReached;
    static bool _stopReceived;
    static bool _isObjOnRightHand;
    static bool _isObjOnLeftHand;

public:
    static std::vector<float> _laCurrentPos;
    static std::vector<float> _raCurrentPos;
    static std::vector<float> _torsoCurrentPos;
    
    static bool setNodeHandle(ros::NodeHandle* nh);
    static bool isLaGoalReached();
    static bool isRaGoalReached();
    static bool isHdGoalReached();
    static bool isTorsoGoalReached();
    static bool waitForLaGoalReached(int timeOut_ms);
    static bool waitForRaGoalReached(int timeOut_ms);
    static bool waitForHdGoalReached(int timeOut_ms);
    static bool waitForTorsoGoalReached(int timeOut_ms);
    //Methods for calculating inverse kinematics
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

    //Methods for operating arms and head
    static void startLaGoToArticular(std::vector<float>& articular);
    static void startLaGoToCartesian(std::vector<float>& cartesian);
    static void startLaGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startLaGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw);
    static void startLaGoToCartesian(float x, float y, float z);
    static void startLaGoToCartesianWrtRobot(std::vector<float>& cartesian);
    static void startLaGoToCartesianWrtRobot(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startLaGoToCartesianFeedback(std::vector<float>& cartesian);
    static void startLaGoToCartesianFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startLaGoToCartesianFeedback(float x, float y, float z);
    static void startLaGoToCartesianWrtRobotFeedback(std::vector<float>& cartesian);
    static void startLaGoToCartesianWrtRobotFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startLaGoToCartesianTraj(std::vector<float>& cartesian);
    static void startLaGoToCartesianTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startLaGoToCartesianTraj(float x, float y, float z);
    static void startLaGoToCartesianWrtRobotTraj(std::vector<float>& cartesian);
    static void startLaGoToCartesianWrtRobotTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startLaGoTo(std::string location);
    static void startLaMove(std::string movement);
    static void startLaOpenGripper(float angle);
    static void startLaCloseGripper(float torque);
    static void startRaGoToArticular(std::vector<float>& articular);
    static void startRaGoToCartesian(std::vector<float>& cartesian);
    static void startRaGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startRaGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw);
    static void startRaGoToCartesian(float x, float y, float z);
    static void startRaGoToCartesianWrtRobot(std::vector<float>& cartesian);
    static void startRaGoToCartesianWrtRobot(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startRaGoToCartesianFeedback(std::vector<float>& cartesian);
    static void startRaGoToCartesianFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startRaGoToCartesianFeedback(float x, float y, float z);
    static void startRaGoToCartesianWrtRobotFeedback(std::vector<float>& cartesian);
    static void startRaGoToCartesianWrtRobotFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startRaGoToCartesianTraj(std::vector<float>& cartesian);
    static void startRaGoToCartesianTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startRaGoToCartesianTraj(float x, float y, float z);
    static void startRaGoToCartesianWrtRobotTraj(std::vector<float>& cartesian);
    static void startRaGoToCartesianWrtRobotTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow);
    static void startRaGoTo(std::string location);
    static void startRaMove(std::string movement);
    static void startRaOpenGripper(float angle);
    static void startRaCloseGripper(float torque);
    static void startHdGoTo(float pan, float tilt);
    static void startHdGoTo(std::string location);
    static void startHdMove(std::string movement);
    static void startTorsoGoTo(float goalSpine, float goalWaist, float goalShoulders);
    static void startTorsoGoToRel(float goalRelSpine, float goalRelWaist, float goalRelShoulders);
    static bool laGoToArticular(std::vector<float>& articular, int timeOut_ms);
    static bool laGoToCartesian(std::vector<float>& cartesian, int timeOut_ms);
    static bool laGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms);
    static bool laGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, int timeOut_ms);
    static bool laGoToCartesian(float x, float y, float z, int timeOut_ms);
    static bool laGoToCartesianFeedback(std::vector<float>& cartesian, int timeOut_ms);
    static bool laGoToCartesianFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms);
    static bool laGoToCartesianFeedback(float x, float y, float z, int timeOut_ms);
    static void laStopGoToCartesian();
    static bool laGoToCartesianTraj(std::vector<float>& cartesian, int timeOut_ms);
    static bool laGoToCartesianTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms);
    static bool laGoToCartesianTraj(float x, float y, float z, int timeOut_ms);
    static bool laGoTo(std::string location, int timeOut_ms);
    static bool laMove(std::string movement, int timeOut_ms);
    static bool raGoToArticular(std::vector<float>& articular, int timeOut_ms);
    static bool raGoToCartesian(std::vector<float>& cartesian, int timeOut_ms);
    static bool raGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms);
    static bool raGoToCartesian(float x, float y, float z, float roll, float pitch, float yaw, int timeOut_ms);
    static bool raGoToCartesian(float x, float y, float z, int timeOut_ms);
    static bool raGoToCartesianFeedback(std::vector<float>& cartesian, int timeOut_ms);
    static bool raGoToCartesianFeedback(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms);
    static bool raGoToCartesianFeedback(float x, float y, float z, int timeOut_ms);
    static void raStopGoToCartesian();
    static bool raGoToCartesianTraj(std::vector<float>& cartesian, int timeOut_ms);
    static bool raGoToCartesianTraj(float x, float y, float z, float roll, float pitch, float yaw, float elbow, int timeOut_ms);
    static bool raGoToCartesianTraj(float x, float y, float z, int timeOut_ms);
    static bool raGoTo(std::string location, int timeOut_ms);
    static bool raMove(std::string movement, int timeOut_ms);
    static bool hdGoTo(float pan, float tilt, int timeOut_ms);
    static bool hdGoTo(std::string location, int timeOut_ms);
    static bool hdMove(std::string movement, int timeOut_ms);
    static bool torsoGoTo(float goalSpine, float goalWaist, float goalShoulders, int timeOut_ms);
    static bool torsoGoToRel(float goalRelSpine, float goalRelWaist, float goalRelShoulders, int timeOut_ms);
    static bool objOnRightHand();
    static bool objOnLeftHand();
    static void getRightHandPosition(float &x, float &y, float &z);
    static void getLeftHandPosition(float &x, float &y, float &z);
    static void getLaCurrentPos(std::vector<float>& pos);
    static void getRaCurrentPos(std::vector<float>& pos);
    static void getTorsoCurrentPos(std::vector<float>& pos);
    static bool isLaInPredefPos(std::string id);
    static bool isRaInPredefPos(std::string id);
    //Methods for moving torso up or down
    static void moveTorsoUp(std_msgs::String msg);
    static void moveTorsoDown(std_msgs::String msg);

    //Callbacks for catching goal-reached signals
    static void callbackRobotStop(const std_msgs::Empty::ConstPtr& msg);
    static void callbackLaGoalReached(const std_msgs::Bool::ConstPtr& msg);
    static void callbackRaGoalReached(const std_msgs::Bool::ConstPtr& msg);
    static void callbackHdGoalReached(const std_msgs::Bool::ConstPtr& msg);
    static void callbackTrGoalReached(const std_msgs::Bool::ConstPtr& msg);
    static void callbackObjOnRightHand(const std_msgs::Bool::ConstPtr& msg);
    static void callbackObjOnLeftHand(const std_msgs::Bool::ConstPtr& msg);
    static void callbackLaCurrentPos(const std_msgs::Float32MultiArray::ConstPtr& msg);
    static void callbackRaCurrentPos(const std_msgs::Float32MultiArray::ConstPtr& msg);
    static void callbackTorsoCurrentPos(const std_msgs::Float32MultiArray::ConstPtr& msg);
};
