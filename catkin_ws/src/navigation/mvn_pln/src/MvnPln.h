#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include "navig_msgs/PathFromMap.h"
#include "navig_msgs/PlanPath.h"
#include "navig_msgs/Location.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaIROS.h"
#include "point_cloud_manager/GetRgbd.h"

#define SM_INIT 0
#define SM_WAITING_FOR_NEW_TASK 1
#define SM_CALCULATE_PATH 2
#define SM_START_MOVE_PATH 3
#define SM_WAIT_FOR_MOVE_FINISHED 4
#define SM_COLLISION_DETECTED 5
#define SM_CORRECT_FINAL_ANGLE 6
#define SM_WAIT_FOR_ANGLE_CORRECTED 7
#define SM_DETECT_OBSTACLE 8
#define SM_AVOIDANCE_HUMAN 9
#define SM_WAIT_FOR_MOVE_HUMAN 10
#define SM_AVOIDANCE_BAG    11
#define SM_AVOIDANCE_CHAIR  12
#define SM_CALCULATE_PATH_AVOIDANCE_CHAIR  13
#define SM_START_AVOIDANCE_CHAIR  14
#define SM_WAIT_FOR_MOVE_CHAIR  15
#define SM_FINISH_FOR_MOVE_CHAIR  16
#define SM_FINAL    17

class MvnPln
{
public:
    MvnPln();
    ~MvnPln();

private:
    ros::NodeHandle* nh;
    //Publishers and subscribers for the commands executed by this node
    ros::ServiceServer srvPlanPath;
    ros::Subscriber subGetCloseLoc;
    ros::Subscriber subGetCloseXYA;
    ros::Subscriber subClickedPoint; //Used to catch clicks on rviz and modify location positions
    ros::Subscriber subRobotStop;
    ros::Subscriber subEnableAvoidanceTypeObstacle;
    ros::Publisher pubGlobalGoalReached;
    ros::Publisher pubStopWaitGlobalGoalReached;
    ros::Publisher pubLastPath;
    ros::Subscriber subLaserScan;
    ros::Subscriber subCollisionRisk;
    ros::Subscriber subCollisionPoint;
    //Ros stuff for path planning
    ros::ServiceClient cltGetMap;
    ros::ServiceClient cltPathFromMapAStar; //Path calculation using only the occupancy grid
    ros::ServiceClient cltGetRgbdWrtRobot;
    tf::TransformListener tf_listener;

    bool newTask;
    bool resetTask;
    bool correctFinalAngle;
    float goalX;
    float goalY;
    float goalAngle;
    std::map<std::string, std::vector<float> > locations;
    std::map<std::string, int> countObstType;
    int framesCount;
    nav_msgs::Path lastCalcPath;
    bool isLastPathPublished;
    bool collisionDetected;
    float collisionPointX;
    float collisionPointY;
    bool stopReceived;
    int timeoutAvoidanceChair;
    bool _allow_move_lateral;
    bool _clean_goal_map;
    bool _look_at_goal;
    bool _clean_unexplored_map;
    bool _avoidance_type_obstacle;
    int _max_frames_count;
    int _min_frames_avoidance;
    
    sensor_msgs::LaserScan lastLaserScan;

    static bool map_compare(const std::pair<std::string, int>& p1, const std::pair<std::string, int>& p2)
    {
        return p1.second < p2.second;
    }

public:
    void initROSConnection(ros::NodeHandle* nh);
    void spin();
    void allow_move_lateral(bool _allow_move_lateral);
    void clean_goal_map(bool _clean_goal_map);
    void look_at_goal(bool _look_at_goal);
    void clean_unexplored_map(bool _clean_unexplored_map);
    void avoidance_type_obstacle(bool _avoidance_type_obstacle);

    int max_attempts;
    float kinect_minX;
    float kinect_maxX;
    float kinect_minY;
    float kinect_maxY;
    float kinect_minZ;
    float kinect_maxZ;

private:
    bool planPath(float startX, float startY, float goalX, float goalY, nav_msgs::Path& path);
    bool planPath(float startX, float startY, float goalX, float goalY, nav_msgs::Path& path,
                  bool useMap, bool useLaser, bool useKinect);
    bool fillMapWithKinect(nav_msgs::OccupancyGrid &grid);
    void callbackRobotStop(const std_msgs::Empty::ConstPtr& msg);
    bool callbackPlanPath(navig_msgs::PlanPath::Request& req, navig_msgs::PlanPath::Response& resp);
    void callbackClickedPoint(const geometry_msgs::PointStamped::ConstPtr& msg);
    void callbackEnableAvoidanceTypeObstacle(const std_msgs::Bool::ConstPtr& msg);
    void callbackGetCloseLoc(const std_msgs::String::ConstPtr& msg);
    void callbackGetCloseXYA(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void callbackCollisionRisk(const std_msgs::Bool::ConstPtr& msg);
    void callbackGoalReached(const std_msgs::Bool::ConstPtr& msg);
    void callbackCollisionPoint(const geometry_msgs::PointStamped::ConstPtr& msg); 
};
