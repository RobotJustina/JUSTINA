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

class JustinaNavigation
{
public:
    static bool is_node_set;
    //Publishers and subscribers for operating the simple_move node
    static ros::Publisher pubSimpleMoveGoalDist;
    static ros::Publisher pubSimpleMoveGoalDistAngle;
    static ros::Publisher pubSimpleMoveGoalPath;
    //Publishers and subscribers for localization
    static ros::Subscriber subRobotCurrentPose;
    //Other publishers and subscribers
    static ros::Subscriber subGoalReached;
    //Services for path calculator
    static ros::ServiceClient srvCltGetMap;
    static ros::ServiceClient srvCltPathAStar;
    static ros::ServiceClient srvCltPathDijkstra;

    //Variables for navigation
    float robotX;
    float robotY;
    float robotTheta;
    nav_msgs::Path lastCalcPath;
    bool isGoalReached;

    static bool setNodeHandle(ros::NodeHandle* nh);
    
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

    static nav_msgs::OccupancyGrid getOccupancyGrid();

    static bool calculatePathAStar(float startX, float startY, float goalX, float goalY);
    static bool calculatePathAStar(float goalX, float goalY);
    static bool calculatePathAStar(std::string location);
    static bool calculatePathDijkstra(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result);
    static bool calculatePathDijkstrs(float goalX, float goalY, nav_msgs::Path& result);
    static bool calculatePathDijkstra(std::string location, nav_msgs::Path& result);
    static bool GetClose
};
