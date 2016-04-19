#include "justina_tools/JustinaNavigation.h"

static bool is_node_set = false;
//Subscriber for checking goal-pose-reached signal
ros::Subscriber JustinaNavigation::subGoalReached;
//Publishers and subscribers for operating the simple_move node
ros::Publisher JustinaNavigation::pubSimpleMoveGoalDist;
ros::Publisher JustinaNavigation::pubSimpleMoveGoalDistAngle;
ros::Publisher JustinaNavigation::pubSimpleMoveGoalPath;
ros::Publisher JustinaNavigation::pubSimpleMoveGoalPose;
ros::Publisher JustinaNavigation::pubSimpleMoveGoalRelPose;
//Services for path calculator
ros::ServiceClient JustinaNavigation::cltGetMap;
ros::ServiceClient JustinaNavigation::cltGetPointCloud;
ros::ServiceClient JustinaNavigation::cltPathFromMapAStar; //Path calculation using only the occupancy grid
ros::ServiceClient JustinaNavigation::cltPathFromMapWaveFront; //Path calculation using only the occupancy grid
ros::ServiceClient JustinaNavigation::cltPathFromAllAStar; //Path calc using occup grid, laser scan and point cloud from kinect
ros::ServiceClient JustinaNavigation::cltPathFromAllWaveFront; //Path calc using occup grid, laser scan and point cloud from kinect
//Publishers and subscribers for localization
ros::Subscriber JustinaNavigation::subCurrentRobotPose;

    //Variables for navigation
float JustinaNavigation::currentRobotX = 0;
float JustinaNavigation::currentRobotY = 0;
float JustinaNavigation::currentRobotTheta = 0;
nav_msgs::Path JustinaNavigation::lastCalcPath = 0;
bool JustinaNavigation::isGoalReached = 0;

//
//The startSomething functions, only publish the goal pose or path and return inmediately after starting movement
//The others, block until a goal-reached signal is received
//

bool JustinaNavigation::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaNavigation::is_node_set)
        return true;
    if(nh == 0)
        return false;

    //Subscriber for checking goal-pose-reached signal
    JustinaNavigation::subGoalReached = nh->subscribe("/navigation/goal_reached", 1, &JustinaNavigation::callbackGoalReached);
    //Publishers and subscribers for operating the simple_move node
    JustinaNavigation::pubSimpleMoveGoalDist = nh->advertise<std_msgs::Float32>("/navigation/path_planning/simple_move/goal_dist", 1);
    JustinaNavigation::pubSimpleMoveGoalDistAngle = nh->advertise<std_msgs::Float32MultiArray>("/navigation/path_planning/simple_move/goal_dist_angle", 1);
    JustinaNavigation::pubSimpleMoveGoalPath = nh->advertise<nav_msgs::Path>("/navigation/path_planning/simple_move/goal_path", 1);
    JustinaNavigation::pubSimpleMoveGoalPose = nh->advertise<geometry_msgs::Pose2D>("/navigation/path_planning/simple_move/goal_pose", 1);
    JustinaNavigation::pubSimpleMoveGoalRelPose = nh->advertise<geometry_msgs::Pose2D>("/navigation/path_planning/simple_move/goal_rel_pose", 1);
    //Services for path calculator
    JustinaNavigation::cltGetMap;
    JustinaNavigation::cltGetPointCloud;
    JustinaNavigation::cltPathFromMapAStar; //Path calculation using only the occupancy grid
    JustinaNavigation::cltPathFromMapWaveFront; //Path calculation using only the occupancy grid
    JustinaNavigation::cltPathFromAllAStar; //Path calc using occup grid, laser scan and point cloud from kinect
    JustinaNavigation::cltPathFromAllWaveFront; //Path calc using occup grid, laser scan and point cloud from kinect
    //Publishers and subscribers for localization
    JustinaNavigation::subCurrentRobotPose = nh->subscribe("/navigation/localization/current_pose", 1, &JustinaNavigation::callbackCurrentRobotPose);


    JustinaNavigation::is_node_set = true;
    return true;
}

bool JustinaNavigation::waitForGoalReached(int timeOut_ms)
{
}

//These methods use the simple_move node
void JustinaNavigation::startMoveDist(float distance)
{
}

void JustinaNavigation::startMoveDistAngle(float distance, float angle)
{
}

void JustinaNavigation::startMovePath(nav_msgs::Path& path)
{
}

void JustinaNavigation::startGoToPose(float x, float y, float angle)
{
}

void JustinaNavigation::startGoToRelPose(float relX, float relY, float relTheta)
{
}

bool JustinaNavigation::moveDist(float distance, int timeOut_ms)
{
}

bool JustinaNavigation::moveDistAngle(float distance, float angle, int timeOut_ms)
{
}

bool JustinaNavigation::movePath(nav_msgs::Path& path, int timeOut_ms)
{
}

void JustinaNavigation::goToPose(float x, float y, float angle, int timeOut_ms)
{
}

void JustinaNavigation::goToRelPose(float relX, float relY, float relTheta, int timeOut_ms)
{
}

//These methods use the mvn_pln node.
bool JustinaNavigation::startGetClose(float x, float y)
{
}

bool JustinaNavigation::startGetClose(float x, float y, float angle)
{
}

bool JustinaNavigation::startGetClose(std::string location)
{
}

bool JustinaNavigation::GetClose(float x, float y, int timeOut_ms)
{
}

bool JustinaNavigation::GetClose(float x, float y, float angle, int timeOut_ms)
{
}

bool JustinaNavigation::GetClose(std::string location, int timeOut_ms)
{
}

//This functions call services, so, they block until a response is received. They use the path_calculator node
nav_msgs::OccupancyGrid JustinaNavigation::getOccupancyGrid()
{
}

bool JustinaNavigation::calcPathFromMapAStar(float startX, float startY, float goalX, float goalY)
{
}

bool JustinaNavigation::calcPathFromMapAStar(float goalX, float goalY)
{
}

bool JustinaNavigation::calcPathFromMapAStar(std::string location)
{
}

bool JustinaNavigation::calcPathFromMapWaveFront(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result)
{
}

bool JustinaNavigation::calcPathFromMapWaveFront(float goalX, float goalY, nav_msgs::Path& result)
{
}

bool JustinaNavigation::calcPathFromMapWaveFront(std::string location, nav_msgs::Path& result)
{
}

bool JustinaNavigation::calcPathFromAllAStar(float startX, float startY, float goalX, float goalY)
{
}

bool JustinaNavigation::calcPathFromAllAStar(float goalX, float goalY)
{
}

bool JustinaNavigation::calcPathFromAllAStar(std::string location)
{
}

bool JustinaNavigation::calcPathFromAllWaveFront(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result)
{
}

bool JustinaNavigation::calcPathFromAllWaveFront(float goalX, float goalY, nav_msgs::Path& result)
{
}

bool JustinaNavigation::calcPathFromAllWaveFront(std::string location, nav_msgs::Path& result)
{
}


//Callbacks for subscribers
void JustinaNavigation::callbackCurrentRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
}

void JustinaNavigation::callbackGoalReached(const std_msgs::Bool::ConstPtr& msg)
{
}
