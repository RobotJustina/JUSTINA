#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "LowLevelControl.h"
#include "tf/transform_listener.h"

class SimpleMoveNode
{
public:
    SimpleMoveNode();
    ~SimpleMoveNode();

    ros::NodeHandle nh;
    ros::Publisher pubGoalReached;
    ros::Publisher pubSpeeds;
    ros::Publisher pubCmdVel;
    ros::Publisher pubHeadGoalPose;
    ros::Subscriber subRobotStop;
    ros::Subscriber subCurrentPose;
    ros::Subscriber subGoalDistance;
    ros::Subscriber subGoalDistAngle;
    ros::Subscriber subGoalPose;
    ros::Subscriber subGoalRelativePose;
    ros::Subscriber subGoalPath;
    ros::Subscriber subGoalLateralDist;
    ros::Subscriber subCollisionRisk;
    tf::TransformListener* tf_listener;

    LowLevelControl control;
    float goalX;
    float goalY;
    float goalTheta;
    float currentX;
    float currentY;
    float currentTheta;
    bool moveBackwards;
    bool moveLateral;
    bool newGoal;
    bool newPath;
    int currentPathPose;
    nav_msgs::Path goalPath;
    bool moveHead;
    bool collisionRisk;

    void initROSConnection();
    void spin();

private:
    void callbackRobotStop(const std_msgs::Empty::ConstPtr& msg);
    void callbackCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void callbackGoalDist(const std_msgs::Float32::ConstPtr& msg);
    void callbackGoalDistAngle(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackGoalPose(const geometry_msgs::Pose2D::ConstPtr& msg);
    void callbackGoalRelPose(const geometry_msgs::Pose2D::ConstPtr& msg);
    void callbackGoalPath(const nav_msgs::Path::ConstPtr& msg);
    void callbackGoalLateralDist(const std_msgs::Float32::ConstPtr& msg);
    void callbackCollisionRisk(const std_msgs::Bool::ConstPtr& msg);
};
