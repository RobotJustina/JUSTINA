#include "justina_tools/JustinaNavigation.h"

bool JustinaNavigation::is_node_set = false;
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
tf::TransformListener JustinaNavigation::tf_listener;

//Variables for navigation
float JustinaNavigation::currentRobotX = 0;
float JustinaNavigation::currentRobotY = 0;
float JustinaNavigation::currentRobotTheta = 0;
nav_msgs::Path JustinaNavigation::lastCalcPath;
bool JustinaNavigation::_isGoalReached = 0;

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

    std::cout << "JustinaNavigation.->Setting ros node..." << std::endl;
    //Subscriber for checking goal-pose-reached signal
    subGoalReached = nh->subscribe("/navigation/goal_reached", 1, &JustinaNavigation::callbackGoalReached);
    //Publishers and subscribers for operating the simple_move node
    pubSimpleMoveGoalDist = nh->advertise<std_msgs::Float32>("/navigation/path_planning/simple_move/goal_dist", 1);
    pubSimpleMoveGoalDistAngle= nh->advertise<std_msgs::Float32MultiArray>("/navigation/path_planning/simple_move/goal_dist_angle", 1);
    pubSimpleMoveGoalPath = nh->advertise<nav_msgs::Path>("/navigation/path_planning/simple_move/goal_path", 1);
    pubSimpleMoveGoalPose = nh->advertise<geometry_msgs::Pose2D>("/navigation/path_planning/simple_move/goal_pose", 1);
    pubSimpleMoveGoalRelPose = nh->advertise<geometry_msgs::Pose2D>("/navigation/path_planning/simple_move/goal_rel_pose", 1);
    //Services for path calculator
    cltGetMap = nh->serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    cltGetPointCloud = nh->serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    cltPathFromMapAStar = nh->serviceClient<navig_msgs::PathFromMap>("/navigation/path_planning/path_calculator/a_star_from_map");
    cltPathFromMapWaveFront=nh->serviceClient<navig_msgs::PathFromMap>("/navigation/path_planning/path_calculator/wave_front_from_map");
    cltPathFromAllAStar = nh->serviceClient<navig_msgs::PathFromAll>("/navigation/path_planning/path_calculator/a_star_from_all");
    cltPathFromAllWaveFront=nh->serviceClient<navig_msgs::PathFromAll>("/navigation/path_planning/path_calculator/wave_front_from_all");
    //Publishers and subscribers for localization
    subCurrentRobotPose = nh->subscribe("/navigation/localization/current_pose", 1, &JustinaNavigation::callbackCurrentRobotPose);
    tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
    
    is_node_set = true;
    return true;
}

bool JustinaNavigation::isGoalReached()
{
    return JustinaNavigation::_isGoalReached;
}

bool JustinaNavigation::waitForGoalReached(int timeOut_ms)
{
    int attempts = timeOut_ms / 100;
    ros::Rate loop(10);
    while(ros::ok() && !JustinaNavigation::_isGoalReached && attempts-- >= 0)
    {
        ros::spinOnce();
        loop.sleep();
    }
    return JustinaNavigation::_isGoalReached;
}

void JustinaNavigation::getRobotPose(float& currentX, float& currentY, float& currentTheta)
{
    tf::StampedTransform transform;
    tf::Quaternion q;
    JustinaNavigation::tf_listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    JustinaNavigation::currentRobotX = transform.getOrigin().x();
    JustinaNavigation::currentRobotY = transform.getOrigin().y();
    q = transform.getRotation();
    JustinaNavigation::currentRobotTheta = atan2((float)q.z(), (float)q.w()) * 2;

    currentX = JustinaNavigation::currentRobotX;
    currentY = JustinaNavigation::currentRobotY;
    currentTheta = JustinaNavigation::currentRobotTheta;
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
    std::cout << "JustinaNavigation.->Publishing goal path.." << std::endl;
    JustinaNavigation::pubSimpleMoveGoalPath.publish(path);
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

bool JustinaNavigation::calcPathFromMapAStar(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result)
{
}

bool JustinaNavigation::calcPathFromMapAStar(float goalX, float goalY, nav_msgs::Path& result)
{
}

bool JustinaNavigation::calcPathFromMapAStar(std::string location, nav_msgs::Path& result)
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

bool JustinaNavigation::calcPathFromAllAStar(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result)
{
    std::cout<<"JustinaNavigation.->Calculating path from " << startX << " " << startX << " to " << goalX << " " << goalY << std::endl;
    nav_msgs::GetMap srvGetMap;
    point_cloud_manager::GetRgbd srvGetPointCloud;
    navig_msgs::PathFromAll srvPathFromAll;

    if(!JustinaNavigation::cltGetMap.call(srvGetMap))
    {
        std::cout << "JustinaNavigation.->Cannot get map from map_server." << std::endl;
        return false;
    }
    srvPathFromAll.request.map = srvGetMap.response.map;
    
    if(!JustinaNavigation::cltGetPointCloud.call(srvGetPointCloud))
        std::cout << "JustinaNavigation.->Cannot get point cloud. Path will be calculated without point cloud." << std::endl;
    else
        srvPathFromAll.request.point_cloud = srvGetPointCloud.response.point_cloud;

    srvPathFromAll.request.start_pose.position.x = startX;
    srvPathFromAll.request.start_pose.position.y = startY;
    srvPathFromAll.request.goal_pose.position.x = goalX;
    srvPathFromAll.request.goal_pose.position.y = goalY;
    
    bool success;
    if((success = JustinaNavigation::cltPathFromAllAStar.call(srvPathFromAll)))
        std::cout << "JustinaNavigation.->Path calculated succesfully by path_calculator using A*" << std::endl;
    else
        std::cout << "JustinaNavigation.->Cannot calculate path by path_calculator using A*" << std::endl;
    ros::spinOnce();

    result = srvPathFromAll.response.path;
    return success;
}

bool JustinaNavigation::calcPathFromAllAStar(float goalX, float goalY, nav_msgs::Path& result)
{
}

bool JustinaNavigation::calcPathFromAllAStar(std::string location, nav_msgs::Path& result)
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
    JustinaNavigation::currentRobotX = msg->pose.pose.position.x;
    JustinaNavigation::currentRobotY = msg->pose.pose.position.y;
    JustinaNavigation::currentRobotTheta = atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) * 2;
}

void JustinaNavigation::callbackGoalReached(const std_msgs::Bool::ConstPtr& msg)
{
    JustinaNavigation::_isGoalReached = msg->data;
}
