#pragma once
#include <iostream>
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
#include "point_cloud_manager/get_rgbd.h"

class JustinaNavigation
{
public:
    static bool is_node_set;
    //Publishers and subscribers for operating the simple_move node
    static ros::Publisher pubSimpleMoveGoalDist;
    static ros::Publisher pubSimpleMoveGoalDistAngle;
    static ros::Publisher pubSimpleMoveGoalPath;
    static ros::Publisher pubSimpleMoveGoalPose;
    static ros::Publisher pubSimpleMoveGoalRelPose;
    //Publishers and subscribers for localization
    static ros::Subscriber subRobotCurrentPose;
    //Subscriber for checking goal-pose-reached signal
    static ros::Subscriber subGoalReached;
    //Services for path calculator
    static ros::ServiceClient cltGetMap;
    static ros::ServiceClient cltGetPointCloud;
    static ros::ServiceClient cltPathFromMapAStar; //Path calculation using only the occupancy grid
    static ros::ServiceClient cltPathFromMapDijkstra; //Path calculation using only the occupancy grid
    static ros::ServiceClient cltPathFromAllAStar; //Path calculation using occupancy grid, laser scan and point cloud from kinect
    static ros::ServiceClient cltPathFromAllDijkstra; //Path calculation using occupancy grid, laser scan and point cloud from kinect

    //Variables for navigation
    static float currentRobotX;
    static float currentRobotY;
    static float currentRobotTheta;
    static nav_msgs::Path lastCalcPath;
    static bool isGoalReached;

    //
    //The startSomething functions, only publish the goal pose or path and return inmediately after starting movement
    //The others, block until a goal-reached signal is received
    //
    
    static bool setNodeHandle(ros::NodeHandle* nh);
    static bool waitForGoalReached(int timeOut_ms);
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
    static bool calcPathFromMapAStar(float startX, float startY, float goalX, float goalY);
    static bool calcPathFromMapAStar(float goalX, float goalY);
    static bool calcPathFromMapAStar(std::string location);
    static bool calcPathFromMapDijkstra(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromMapDijkstrs(float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromMapDijkstra(std::string location, nav_msgs::Path& result);
    static bool calcPathFromAllAStar(float startX, float startY, float goalX, float goalY);
    static bool calcPathFromAllAStar(float goalX, float goalY);
    static bool calcPathFromAllAStar(std::string location);
    static bool calcPathFromAllDijkstra(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromAllDijkstrs(float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromAllDijkstra(std::string location, nav_msgs::Path& result);

    //Callbacks for subscribers
};
