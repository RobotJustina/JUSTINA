#pragma once
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "bbros_bridge/RecognizedSpeech.h"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "navig_msgs/PathFromMap.h"
#include "navig_msgs/PathFromAll.h"
#include "point_cloud_manager/GetRgbd.h"
#include "tf/transform_listener.h"

class JustinaNavigation
{
private:
    static bool is_node_set;
    //Subscriber for checking goal-pose-reached signal
    static ros::Subscriber subGoalReached;
    //Publishers and subscribers for operating the simple_move node
    static ros::Publisher pubSimpleMoveGoalDist;
    static ros::Publisher pubSimpleMoveGoalDistAngle;
    static ros::Publisher pubSimpleMoveGoalPath;
    static ros::Publisher pubSimpleMoveGoalPose;
    static ros::Publisher pubSimpleMoveGoalRelPose;
    //Services for path calculator
    static ros::ServiceClient cltGetMap;
    static ros::ServiceClient cltGetPointCloud;
    static ros::ServiceClient cltPathFromMapAStar; //Path calculation using only the occupancy grid
    static ros::ServiceClient cltPathFromMapWaveFront; //Path calculation using only the occupancy grid
    static ros::ServiceClient cltPathFromAllAStar; //Path calculation using occupancy grid, laser scan and point cloud from kinect
    static ros::ServiceClient cltPathFromAllWaveFront; //Path calculation using occupancy grid, laser scan and point cloud from kinect
    //Publishers and subscribers for localization
    static ros::Subscriber subCurrentRobotPose;
    static tf::TransformListener tf_listener;

    //Variables for navigation
    static float currentRobotX;
    static float currentRobotY;
    static float currentRobotTheta;
    static nav_msgs::Path lastCalcPath;
    static bool _isGoalReached;

public:
    //
    //The startSomething functions, only publish the goal pose or path and return inmediately after starting movement
    //The others, block until a goal-reached signal is received
    //
    
    static bool setNodeHandle(ros::NodeHandle* nh);
    static bool isGoalReached();
    static bool waitForGoalReached(int timeOut_ms);
    static void getRobotPose(float& currentX, float& currentY, float& currentTheta);
    //These methods use the simple_move node
    static void startMoveDist(float distance);
    static void startMoveDistAngle(float distance, float angle);
    static void startMovePath(nav_msgs::Path& path);
    static void startGoToPose(float x, float y, float angle);
    static void startGoToRelPose(float relX, float relY, float relTheta);
    static bool moveDist(float distance, int timeOut_ms);
    static bool moveDistAngle(float distance, float angle, int timeOut_ms);
    static bool movePath(nav_msgs::Path& path, int timeOut_ms);
    static void goToPose(float x, float y, float angle, int timeOut_ms);
    static void goToRelPose(float relX, float relY, float relTheta, int timeOut_ms);

    //These methods use the mvn_pln node.
    static bool startGetClose(float x, float y);
    static bool startGetClose(float x, float y, float angle);
    static bool startGetClose(std::string location);
    static bool GetClose(float x, float y, int timeOut_ms);
    static bool GetClose(float x, float y, float angle, int timeOut_ms);
    static bool GetClose(std::string location, int timeOut_ms);

    //This functions call services, so, they block until a response is received. They use the path_calculator node
    static nav_msgs::OccupancyGrid getOccupancyGrid();
    static bool calcPathFromMapAStar(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromMapAStar(float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromMapAStar(std::string location, nav_msgs::Path& result);
    static bool calcPathFromMapWaveFront(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromMapWaveFront(float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromMapWaveFront(std::string location, nav_msgs::Path& result);
    static bool calcPathFromAllAStar(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromAllAStar(float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromAllAStar(std::string location, nav_msgs::Path& result);
    static bool calcPathFromAllWaveFront(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromAllWaveFront(float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromAllWaveFront(std::string location, nav_msgs::Path& result);

    //Callbacks for subscribers
    static void callbackCurrentRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    static void callbackGoalReached(const std_msgs::Bool::ConstPtr& msg);
};
