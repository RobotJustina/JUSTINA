#include "justina_tools/JustinaNavigation.h"
    
bool JustinaNavigation::is_node_set = false;
bool JustinaNavigation::_tasksStop = false;
ros::Subscriber JustinaNavigation::subTasksStop;
//Subscriber for checking goal-pose-reached signal
ros::Subscriber JustinaNavigation::subGoalReached;
ros::Subscriber JustinaNavigation::subGlobalGoalReached;
ros::Subscriber JustinaNavigation::subStopWaitGlobalGoalReached;
ros::Subscriber JustinaNavigation::subStopRobot;
//Publishers and subscribers for operating the simple_move node
ros::Publisher JustinaNavigation::pubSimpleMoveDist;
ros::Publisher JustinaNavigation::pubSimpleMoveDistAngle;
ros::Publisher JustinaNavigation::pubSimpleMoveLateral;
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
tf::TransformListener* JustinaNavigation::tf_listener;
//Subscribers for obstacle avoidance
ros::Publisher JustinaNavigation::pubObsAvoidEnable;
ros::Publisher JustinaNavigation::pubEnableDoorDetector;
ros::Publisher JustinaNavigation::pubEnableAvoidanceTypeObstacle;
ros::Subscriber JustinaNavigation::subObsInFront;
ros::Subscriber JustinaNavigation::subCollisionRisk;
ros::Subscriber JustinaNavigation::subDetectedDoor;

//Variables for navigation
float JustinaNavigation::currentRobotX = 0;
float JustinaNavigation::currentRobotY = 0;
float JustinaNavigation::currentRobotTheta = 0;
bool JustinaNavigation::_isGoalReached = 0;
bool JustinaNavigation::_isGlobalGoalReached = 0;
bool JustinaNavigation::_stopWaitGlobalGoalReached;
bool JustinaNavigation::_stopReceived = false;
bool JustinaNavigation::_obstacleInFront = false;
bool JustinaNavigation::_collisionRisk;
bool JustinaNavigation::_detectedDoor = false;

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
    subTasksStop = nh->subscribe("/planning/tasks_stop", 1, &JustinaNavigation::callbackTasksStop);
    subGoalReached = nh->subscribe("/navigation/goal_reached", 1, &JustinaNavigation::callbackGoalReached);
    subGlobalGoalReached = nh->subscribe("/navigation/global_goal_reached", 1, &JustinaNavigation::callbackGlobalGoalReached);
    subStopWaitGlobalGoalReached = nh->subscribe("/navigation/stop_wait_global_goal_reached", 1, &JustinaNavigation::callbackStopWaitGlobalGoalReached);
    subStopRobot = nh->subscribe("/hardware/robot_state/stop", 1, &JustinaNavigation::callbackRobotStop);
    //Publishers and subscribers for operating the simple_move node
    pubSimpleMoveDist = nh->advertise<std_msgs::Float32>("/navigation/path_planning/simple_move/goal_dist", 1);
    pubSimpleMoveDistAngle=nh->advertise<std_msgs::Float32MultiArray>("/navigation/path_planning/simple_move/goal_dist_angle",1);
    pubSimpleMoveLateral = nh->advertise<std_msgs::Float32>("/navigation/path_planning/simple_move/goal_lateral", 1);
    pubSimpleMoveGoalPath = nh->advertise<nav_msgs::Path>("/navigation/path_planning/simple_move/goal_path", 1);
    pubSimpleMoveGoalPose = nh->advertise<geometry_msgs::Pose2D>("/navigation/path_planning/simple_move/goal_pose", 1);
    pubSimpleMoveGoalRelPose = nh->advertise<geometry_msgs::Pose2D>("/navigation/path_planning/simple_move/goal_rel_pose", 1);
    //Services for path calculator
    cltGetMap = nh->serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    cltGetPointCloud = nh->serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    cltPathFromMapAStar = nh->serviceClient<navig_msgs::PathFromMap>("/navigation/path_planning/path_calculator/a_star_from_map");

    //Publishers and subscribers for mvn_pln
    cltPlanPath = nh->serviceClient<navig_msgs::PlanPath>("/navigation/mvn_pln/plan_path");
    pubMvnPlnGetCloseLoc = nh->advertise<std_msgs::String>("/navigation/mvn_pln/get_close_loc", 1);
    pubMvnPlnGetCloseXYA = nh->advertise<std_msgs::Float32MultiArray>("/navigation/mvn_pln/get_close_xya", 1);
    //Subscribers and publishers for obstacle avoidance
    pubObsAvoidEnable = nh->advertise<std_msgs::Bool>("/navigation/obs_avoid/enable", 1);
    pubEnableDoorDetector = nh->advertise<std_msgs::Bool>("/navigation/obs_avoid/enable_door_detector", 1);
    pubEnableAvoidanceTypeObstacle = nh->advertise<std_msgs::Bool>("/navigation/mvn_pln/enable_avoidance_type_obstacle", 1);
    subObsInFront = nh->subscribe("/navigation/obs_avoid/obs_in_front", 1, &JustinaNavigation::callbackObstacleInFront);
    subCollisionRisk = nh->subscribe("/navigation/obs_avoid/collision_risk", 1, &JustinaNavigation::callbackCollisionRisk);
    subDetectedDoor = nh->subscribe("/navigation/obs_avoid/detected_door", 1, &JustinaNavigation::callbackDetectedDoor);
    //Publishers and subscribers for localization
    tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
    
    is_node_set = true;
    return true;
}

void JustinaNavigation::callbackTasksStop(const std_msgs::Empty::ConstPtr& msg)
{
    _tasksStop = true;
}

bool JustinaNavigation::tasksStop(){
    bool tasksStop = _tasksStop;
    _tasksStop = false;
    return tasksStop;
}

bool JustinaNavigation::isGoalReached()
{
    //std::cout << "JustinaNavigation.->Goal reched: " << JustinaNavigation::_isGoalReached << std::endl;
    return JustinaNavigation::_isGoalReached;
}

bool JustinaNavigation::isGlobalGoalReached()
{
    //std::cout << "JustinaNavigation.->Goal reched: " << JustinaNavigation::_isGoalReached << std::endl;
    return JustinaNavigation::_isGlobalGoalReached;
}

bool JustinaNavigation::waitForGoalReached(int timeOut_ms)
{
    int attempts = timeOut_ms / 100;
    ros::Rate loop(10);
    loop.sleep();
    ros::spinOnce();
    JustinaNavigation::_stopReceived = false;
    JustinaNavigation::_isGoalReached = false;
    while(ros::ok() && !JustinaNavigation::_isGoalReached && !JustinaNavigation::_stopReceived && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    JustinaNavigation::_stopReceived = false; //This flag is set True in the subscriber callback
    return JustinaNavigation::_isGoalReached;
}

bool JustinaNavigation::waitForGlobalGoalReached(int timeOut_ms)
{
    int attempts = timeOut_ms / 100;
    ros::Rate loop(10);
    loop.sleep();
    ros::spinOnce();
    JustinaNavigation::_stopWaitGlobalGoalReached = false;
    JustinaNavigation::_isGlobalGoalReached = false;
    JustinaNavigation::_tasksStop = false;
    JustinaNavigation::_stopReceived = false;
    while(ros::ok() && !JustinaNavigation::_isGlobalGoalReached && !JustinaNavigation::_stopReceived && !JustinaNavigation::_stopWaitGlobalGoalReached && attempts-- >= 0 && !JustinaNavigation::tasksStop())
    {
        loop.sleep();
        ros::spinOnce();
    }
    JustinaNavigation::_stopReceived = false; //This flag is set True in the subscriber callback
    JustinaNavigation::_tasksStop = false;
    return JustinaNavigation::_isGlobalGoalReached;
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

void JustinaNavigation::getRobotPoseFromOdom(float& currentX, float& currentY,
		float& currentTheta) {
	tf::StampedTransform transform;
	tf::Quaternion q;
	JustinaNavigation::tf_listener->lookupTransform("/odom", "/base_link",
			ros::Time(0), transform);
	q = transform.getRotation();

	currentX = transform.getOrigin().x();
	currentY = transform.getOrigin().y();
	currentTheta = q.getAngle() * q.getAxis().z();
}

//Methods for obstacle avoidance
bool JustinaNavigation::obstacleInFront()
{
    return JustinaNavigation::_obstacleInFront;
}

bool JustinaNavigation::collisionRisk()
{
    return JustinaNavigation::_collisionRisk;
}
    
bool JustinaNavigation::doorIsOpen(float minConfidence, int timeout)
{
	boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
	boost::posix_time::ptime curr = prev;
    ros::Rate rate(30);
    JustinaNavigation::enableDoorDetector(true);
    int readings = 0;
    int readingsDoorClose = 0;
    JustinaNavigation::_detectedDoor = false;
	while(ros::ok() && (curr - prev).total_milliseconds() < timeout){
        curr = boost::posix_time::second_clock::local_time();
        readings++;
        if(JustinaNavigation::_detectedDoor)
            readingsDoorClose++;
        ros::spinOnce();
        rate.sleep();
    }
    JustinaNavigation::enableDoorDetector(false);
    ros::spinOnce();
    rate.sleep();
    float confidence = ((float) readingsDoorClose / (float) readings);
    if(confidence >= minConfidence)
        return false;
    return true;
}

void JustinaNavigation::enableObstacleDetection(bool enable)
{
    if(enable)
        std::cout << "JustinaNavigation.->Enabling obstacle detection... " << std::endl;
    else
        std::cout << "JustinaNavigation.->Disabling obstacle detection... " << std::endl;
    std_msgs::Bool msg;
    msg.data = enable;
    JustinaNavigation::pubObsAvoidEnable.publish(msg);
}

void JustinaNavigation::enableAvoidanceTypeObstacle(bool enable)
{
	if(enable)
		std::cout << "JustinaNavigation.->Enabling avoidance obstacle type... " << std::endl;
	else
		std::cout << "JustinaNavigation.->Disabling avoidance obstacle type... " << std::endl;
	std_msgs::Bool msg;
	msg.data = enable;
	JustinaNavigation::pubEnableAvoidanceTypeObstacle.publish(msg);
}

void JustinaNavigation::enableDoorDetector(bool enable)
{
	if(enable)
		std::cout << "JustinaNavigation.->Enabling door detector... " << std::endl;
	else
		std::cout << "JustinaNavigation.->Disabling door detector... " << std::endl;
	std_msgs::Bool msg;
	msg.data = enable;
	JustinaNavigation::pubEnableDoorDetector.publish(msg);
}

//These methods use the simple_move node
void JustinaNavigation::startMoveDist(float distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    JustinaNavigation::_isGoalReached = false;
    pubSimpleMoveDist.publish(msg);
    ros::spinOnce();
    ros::Duration(0.033333333).sleep();
}

void JustinaNavigation::startMoveDistAngle(float distance, float angle)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(distance);
    msg.data.push_back(angle);
    JustinaNavigation::_isGoalReached = false;
    pubSimpleMoveDistAngle.publish(msg);
    ros::spinOnce();
    ros::Duration(0.033333333).sleep();
}

void JustinaNavigation::startMoveLateral(float distance)
{
    std::cout << "JustinaNavigation.->Publishing goal lateral distance: " << distance << std::endl;
    std_msgs::Float32 msg;
    msg.data = distance;
    JustinaNavigation::_isGoalReached = false;
    JustinaNavigation::pubSimpleMoveLateral.publish(msg);
    ros::spinOnce();
    ros::Duration(0.033333333).sleep();
}

void JustinaNavigation::startMovePath(nav_msgs::Path& path)
{
    std::cout << "JustinaNavigation.->Publishing goal path.." << std::endl;
    JustinaNavigation::_isGoalReached = false;
    JustinaNavigation::pubSimpleMoveGoalPath.publish(path);
    ros::spinOnce();
    ros::Duration(0.033333333).sleep();
}

void JustinaNavigation::startGoToPose(float x, float y, float angle)
{
    geometry_msgs::Pose2D msg;
    msg.x = x;
    msg.y = y;
    msg.theta = angle;
    JustinaNavigation::_isGoalReached = false;
    pubSimpleMoveGoalPose.publish(msg);
    ros::spinOnce();
    ros::Duration(0.033333333).sleep();
}

void JustinaNavigation::startGoToRelPose(float relX, float relY, float relTheta)
{
    geometry_msgs::Pose2D msg;
    msg.x = relX;
    msg.y = relY;
    msg.theta = relTheta;
    JustinaNavigation::_isGoalReached = false;
    pubSimpleMoveGoalRelPose.publish(msg);
    ros::spinOnce();
    ros::Duration(0.033333333).sleep();
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

bool JustinaNavigation::moveLateral(float distance, int timeOut_ms)
{
    JustinaNavigation::startMoveLateral(distance);
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
    JustinaNavigation::_isGlobalGoalReached = false;
    pubMvnPlnGetCloseXYA.publish(msg);
    ros::spinOnce();
}

void JustinaNavigation::startGetClose(float x, float y, float angle)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(angle);
    JustinaNavigation::_isGlobalGoalReached = false;
    pubMvnPlnGetCloseXYA.publish(msg);
    ros::spinOnce();
}

void JustinaNavigation::startGetClose(std::string location)
{
    std_msgs::String msg;
    msg.data = location;
    JustinaNavigation::_isGlobalGoalReached = false;
    pubMvnPlnGetCloseLoc.publish(msg);
    ros::spinOnce();
}

bool JustinaNavigation::getClose(float x, float y, int timeOut_ms)
{
    JustinaNavigation::startGetClose(x, y);
    return JustinaNavigation::waitForGlobalGoalReached(timeOut_ms);
}

bool JustinaNavigation::getClose(float x, float y, float angle, int timeOut_ms)
{
    JustinaNavigation::startGetClose(x, y, angle);
    return JustinaNavigation::waitForGlobalGoalReached(timeOut_ms);
}

bool JustinaNavigation::getClose(std::string location, int timeOut_ms)
{
    JustinaNavigation::startGetClose(location);
    return JustinaNavigation::waitForGlobalGoalReached(timeOut_ms);
}

bool JustinaNavigation::getStopWaitGlobalGoalReached()
{
	return JustinaNavigation::_stopWaitGlobalGoalReached;
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

void JustinaNavigation::callbackRobotStop(const std_msgs::Empty::ConstPtr& msg)
{
    JustinaNavigation::_stopReceived = true;
}

void JustinaNavigation::callbackGoalReached(const std_msgs::Bool::ConstPtr& msg)
{
    JustinaNavigation::_isGoalReached = msg->data;
    //std::cout << "JustinaNavigation.->Received goal reached: " << int(msg->data) << std::endl;
}

void JustinaNavigation::callbackGlobalGoalReached(const std_msgs::Bool::ConstPtr& msg)
{
    JustinaNavigation::_isGlobalGoalReached = msg->data;
    //std::cout << "JustinaNavigation.->Received global goal reached: " << int(msg->data) << std::endl;
}

void JustinaNavigation::callbackStopWaitGlobalGoalReached(const std_msgs::Empty::ConstPtr& msg)
{
    JustinaNavigation::_stopWaitGlobalGoalReached = true;
    //std::cout << "JustinaNavigation.->Received global goal reached: " << int(msg->data) << std::endl;
}

//Callbacks for obstacle avoidance
void JustinaNavigation::callbackObstacleInFront(const std_msgs::Bool::ConstPtr& msg)
{
    JustinaNavigation::_obstacleInFront = msg->data;
}

void JustinaNavigation::callbackCollisionRisk(const std_msgs::Bool::ConstPtr& msg)
{
    //std::cout << "JustinaNvigation.-<CollisionRisk: " << int(msg->data) << std::endl;
    JustinaNavigation::_collisionRisk = msg->data;
}

void JustinaNavigation::callbackDetectedDoor(const std_msgs::Bool::ConstPtr& msg)
{
    JustinaNavigation::_detectedDoor = msg->data;
}
