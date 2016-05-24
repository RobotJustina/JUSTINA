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
//Publishers and subscriber for mvn_pln
ros::ServiceClient JustinaNavigation::cltPlanPath;
ros::Publisher JustinaNavigation::pubMvnPlnGetCloseLoc;
ros::Publisher JustinaNavigation::pubMvnPlnGetCloseXYA;
//Publishers and subscribers for localization
ros::Subscriber JustinaNavigation::subCurrentRobotPose;
tf::TransformListener* JustinaNavigation::tf_listener;

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
    tf_listener = new tf::TransformListener();
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
    //Publishers and subscribers for mvn_pln
    cltPlanPath = nh->serviceClient<navig_msgs::PlanPath>("/navigation/mvn_pln/plan_path");
    pubMvnPlnGetCloseLoc = nh->advertise<std_msgs::String>("/navigation/mvn_pln/get_close_loc", 1);
    pubMvnPlnGetCloseXYA = nh->advertise<std_msgs::Float32MultiArray>("/navigation/mvn_pln/get_close_xya", 1);
    //Publishers and subscribers for localization
    subCurrentRobotPose = nh->subscribe("/navigation/localization/current_pose", 1, &JustinaNavigation::callbackCurrentRobotPose);
    tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
    
    is_node_set = true;
    return true;
}

bool JustinaNavigation::isGoalReached()
{
    std::cout << "JustinaNavigation.->Goal reched: " << JustinaNavigation::_isGoalReached << std::endl;
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
    JustinaNavigation::tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
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
    std_msgs::Float32 msg;
    msg.data = distance;
    JustinaNavigation::_isGoalReached = false;
    pubSimpleMoveGoalDist.publish(msg);
}

void JustinaNavigation::startMoveDistAngle(float distance, float angle)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(distance);
    msg.data.push_back(angle);
    JustinaNavigation::_isGoalReached = false;
    pubSimpleMoveGoalDistAngle.publish(msg);
}

void JustinaNavigation::startMovePath(nav_msgs::Path& path)
{
    std::cout << "JustinaNavigation.->Publishing goal path.." << std::endl;
    JustinaNavigation::_isGoalReached = false;
    JustinaNavigation::pubSimpleMoveGoalPath.publish(path);
}

void JustinaNavigation::startGoToPose(float x, float y, float angle)
{
    geometry_msgs::Pose2D msg;
    msg.x = x;
    msg.y = y;
    msg.theta = angle;
    JustinaNavigation::_isGoalReached = false;
    pubSimpleMoveGoalPose.publish(msg);
}

void JustinaNavigation::startGoToRelPose(float relX, float relY, float relTheta)
{
    geometry_msgs::Pose2D msg;
    msg.x = relX;
    msg.y = relY;
    msg.theta = relTheta;
    JustinaNavigation::_isGoalReached = false;
    pubSimpleMoveGoalRelPose.publish(msg);
}

bool JustinaNavigation::moveDist(float distance, int timeOut_ms)
{
    JustinaNavigation::startMoveDist(distance);
    return JustinaNavigation::waitForGoalReached(timeOut_ms);
}

bool JustinaNavigation::moveDistAngle(float distance, float angle, int timeOut_ms)
{
    JustinaNavigation::startMoveDistAngle(distance, angle);
    return JustinaNavigation::waitForGoalReached(timeOut_ms);
}

bool JustinaNavigation::movePath(nav_msgs::Path& path, int timeOut_ms)
{
    JustinaNavigation::startMovePath(path);
    return JustinaNavigation::waitForGoalReached(timeOut_ms);
}

bool JustinaNavigation::goToPose(float x, float y, float angle, int timeOut_ms)
{
    JustinaNavigation::startGoToPose(x, y, angle);
    return JustinaNavigation::waitForGoalReached(timeOut_ms);
}

bool JustinaNavigation::goToRelPose(float relX, float relY, float relTheta, int timeOut_ms)
{
    JustinaNavigation::startGoToRelPose(relX, relY, relTheta);
    return JustinaNavigation::waitForGoalReached(timeOut_ms);
}

//These methods use the mvn_pln node.
bool JustinaNavigation::planPath(float startX, float startY, float goalX, float goalY, nav_msgs::Path& path)
{
    navig_msgs::PlanPath srv;
    srv.request.start_location_id = "";
    srv.request.goal_location_id = "";
    srv.request.start_pose.position.x = startX;
    srv.request.start_pose.position.y = startY;
    srv.request.goal_pose.position.x = goalX;
    srv.request.goal_pose.position.y = goalY;
    bool success = JustinaNavigation::cltPlanPath.call(srv);
    path = srv.response.path;
    return success;
}

bool JustinaNavigation::planPath(float goalX, float goalY, nav_msgs::Path& path)
{
    float robotX, robotY, robotTheta;
    JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
    return JustinaNavigation::planPath(robotX, robotY, goalX, goalY, path);
}

bool JustinaNavigation::planPath(std::string start_location, std::string goal_location, nav_msgs::Path& path)
{
    navig_msgs::PlanPath srv;
    srv.request.start_location_id = start_location;
    srv.request.goal_location_id = goal_location;
    bool success = JustinaNavigation::cltPlanPath.call(srv);
    path = srv.response.path;
    return success;
}

bool JustinaNavigation::planPath(std::string goal_location, nav_msgs::Path& path)
{
    float robotX, robotY, robotTheta;
    JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
    navig_msgs::PlanPath srv;
    srv.request.start_location_id = "";
    srv.request.goal_location_id = goal_location;
    srv.request.start_pose.position.x = robotX;
    srv.request.start_pose.position.y = robotY;
    bool success = JustinaNavigation::cltPlanPath.call(srv);
    path = srv.response.path;
    return success;
}

bool JustinaNavigation::planPath(std::string start_location, float goalX, float goalY, nav_msgs::Path& path)
{
    navig_msgs::PlanPath srv;
    srv.request.start_location_id = start_location;
    srv.request.goal_location_id = "";
    srv.request.goal_pose.position.x = goalX;
    srv.request.goal_pose.position.y = goalY;
    bool success = JustinaNavigation::cltPlanPath.call(srv);
    path = srv.response.path;
    return success;
}

bool JustinaNavigation::planPath(float startX, float startY, std::string goal_location, nav_msgs::Path& path)
{
    navig_msgs::PlanPath srv;
    srv.request.start_location_id = "";
    srv.request.goal_location_id = goal_location;
    srv.request.start_pose.position.x = startX;
    srv.request.start_pose.position.y = startY;
    bool success = JustinaNavigation::cltPlanPath.call(srv);
    path = srv.response.path;
    return success;
}

void JustinaNavigation::startGetClose(float x, float y)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    JustinaNavigation::_isGoalReached = false;
    pubMvnPlnGetCloseXYA.publish(msg);
}

void JustinaNavigation::startGetClose(float x, float y, float angle)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(angle);
    JustinaNavigation::_isGoalReached = false;
    pubMvnPlnGetCloseXYA.publish(msg);
}

void JustinaNavigation::startGetClose(std::string location)
{
    std_msgs::String msg;
    msg.data = location;
    JustinaNavigation::_isGoalReached = false;
    pubMvnPlnGetCloseLoc.publish(msg);
}

bool JustinaNavigation::getClose(float x, float y, int timeOut_ms)
{
    JustinaNavigation::startGetClose(x, y);
    return JustinaNavigation::waitForGoalReached(timeOut_ms);
}

bool JustinaNavigation::getClose(float x, float y, float angle, int timeOut_ms)
{
    JustinaNavigation::startGetClose(x, y, angle);
    return JustinaNavigation::waitForGoalReached(timeOut_ms);
}

bool JustinaNavigation::getClose(std::string location, int timeOut_ms)
{
    JustinaNavigation::startGetClose(location);
    return JustinaNavigation::waitForGoalReached(timeOut_ms);
}

//This functions call services, so, they block until a response is received. They use the path_calculator node
bool JustinaNavigation::getOccupancyGrid(nav_msgs::OccupancyGrid& map)
{
    nav_msgs::GetMap srvGetMap;
    if(!JustinaNavigation::cltGetMap.call(srvGetMap))
    {
        std::cout << "JustinaNavigation.->Cannot get map from map_server." << std::endl;
        return false;
    }
    map = srvGetMap.response.map;
    return true;
}

bool JustinaNavigation::calcPathFromMapAStar(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result)
{
    std::cout << "JustinaNavig.->Calculating path from " << startX << " " << startX << " to " << goalX <<" " << goalY;
    std::cout << "by A* using only map"<<std::endl;
    nav_msgs::GetMap srvGetMap;
    navig_msgs::PathFromMap srvPathFromMap;

    if(!JustinaNavigation::cltGetMap.call(srvGetMap))
    {
        std::cout << "JustinaNavigation.->Cannot get map from map_server." << std::endl;
        return false;
    }
    srvPathFromMap.request.map = srvGetMap.response.map;

    srvPathFromMap.request.start_pose.position.x = startX;
    srvPathFromMap.request.start_pose.position.y = startY;
    srvPathFromMap.request.goal_pose.position.x = goalX;
    srvPathFromMap.request.goal_pose.position.y = goalY;

    bool success;
    if((success = JustinaNavigation::cltPathFromMapAStar.call(srvPathFromMap)))
        std::cout << "JustinaNavigation.->Path calculated succesfully by path_calculator using A* using only map" << std::endl;
    else
        std::cout << "JustinaNavigation.->Cannot calculate path by path_calculator using A* using only map" << std::endl;
    ros::spinOnce();

    result = srvPathFromMap.response.path;
    return success;
}

bool JustinaNavigation::calcPathFromMapAStar(float goalX, float goalY, nav_msgs::Path& result)
{
    float robotX, robotY, robotTheta;
    JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
    return JustinaNavigation::calcPathFromMapAStar(robotX, robotY, goalX, goalY, result);
}

bool JustinaNavigation::calcPathFromMapWaveFront(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result)
{
    std::cout << "JustinaNavig.->Calculating path from " << startX << " " << startX << " to " << goalX <<" " << goalY;
    std::cout << "by wave front using only map"<<std::endl;
    nav_msgs::GetMap srvGetMap;
    navig_msgs::PathFromMap srvPathFromMap;

    if(!JustinaNavigation::cltGetMap.call(srvGetMap))
    {
        std::cout << "JustinaNavigation.->Cannot get map from map_server." << std::endl;
        return false;
    }
    srvPathFromMap.request.map = srvGetMap.response.map;

    srvPathFromMap.request.start_pose.position.x = startX;
    srvPathFromMap.request.start_pose.position.y = startY;
    srvPathFromMap.request.goal_pose.position.x = goalX;
    srvPathFromMap.request.goal_pose.position.y = goalY;

    bool success;
    if((success = JustinaNavigation::cltPathFromMapWaveFront.call(srvPathFromMap)))
        std::cout << "JustinaNavigation.->Path calculated succesfully by path_calculator using wave front using only map" << std::endl;
    else
        std::cout << "JustinaNavigation.->Cannot calculate path by path_calculator using wave front using only map" << std::endl;
    ros::spinOnce();

    result = srvPathFromMap.response.path;
    return success;
}

bool JustinaNavigation::calcPathFromMapWaveFront(float goalX, float goalY, nav_msgs::Path& result)
{
    float robotX, robotY, robotTheta;
    JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
    return JustinaNavigation::calcPathFromMapWaveFront(robotX, robotY, goalX, goalY, result);
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
    std::cout << "JustinaNavigation.->Received goal reached: " << int(msg->data) << std::endl;
}
