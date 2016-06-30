#include "MvnPln.h"

MvnPln::MvnPln()
{
    this->newTask = false;
    this->correctFinalAngle = false;
    this->collisionDetected = false;
    this->stopReceived = false;
    this->isLastPathPublished = false;
}

MvnPln::~MvnPln()
{
}

void MvnPln::initROSConnection(ros::NodeHandle* nh)
{
    this->nh = nh;
    //Publishers and subscribers for the commands executed by this node
    this->subGetCloseLoc = nh->subscribe("/navigation/mvn_pln/get_close_loc", 10, &MvnPln::callbackGetCloseLoc, this);
    this->subGetCloseXYA = nh->subscribe("/navigation/mvn_pln/get_close_xya", 10, &MvnPln::callbackGetCloseXYA, this);
    this->subAddLocation = nh->subscribe("/navigation/mvn_pln/add_location", 10, &MvnPln::callbackAddLocation, this);
    this->subClickedPoint = nh->subscribe("/clicked_point", 1, &MvnPln::callbackClickedPoint, this);
    this->subRobotStop = nh->subscribe("/hardware/robot_state/stop", 10, &MvnPln::callbackRobotStop, this);
    this->pubGlobalGoalReached = nh->advertise<std_msgs::Bool>("/navigation/global_goal_reached", 10);
    this->pubLocationMarkers = nh->advertise<visualization_msgs::Marker>("/hri/rviz/location_markers", 1);
    this->pubLastPath = nh->advertise<nav_msgs::Path>("/navigation/mvn_pln/last_calc_path", 1);
    this->srvPlanPath = nh->advertiseService("/navigation/mvn_pln/plan_path", &MvnPln::callbackPlanPath, this);
    this->subLaserScan = nh->subscribe("/hardware/scan", 1, &MvnPln::callbackLaserScan, this);
    this->subCollisionRisk = nh->subscribe("/navigation/obs_avoid/collision_risk", 10, &MvnPln::callbackCollisionRisk, this);
    this->subCollisionPoint = nh->subscribe("/navigation/obs_avoid/collision_point", 10, &MvnPln::callbackCollisionPoint, this);

    this->cltGetMap = nh->serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    this->cltPathFromMapAStar = nh->serviceClient<navig_msgs::PathFromMap>("/navigation/path_planning/path_calculator/a_star_from_map");
    this->cltGetRgbdWrtRobot = nh->serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
}

bool MvnPln::loadKnownLocations(std::string path)
{
    std::cout << "MvnPln.->Loading known locations from " << path << std::endl;
    std::vector<std::string> lines;
    std::ifstream file(path.c_str());
    std::string tempStr;
    while(std::getline(file, tempStr))
        lines.push_back(tempStr);

    //Extraction of lines without comments
    for(size_t i=0; i< lines.size(); i++)
    {
        size_t idx = lines[i].find("//");
        if(idx!= std::string::npos)
            lines[i] = lines[i].substr(0, idx);
    }

    this->locations.clear();
    float locX, locY, locAngle;
    bool parseSuccess;
    for(size_t i=0; i<lines.size(); i++)
    {
        //std::cout << "MvnPln.->Parsing line: " << lines[i] << std::endl;
        std::vector<std::string> parts;
        std::vector<float> loc;
        boost::split(parts, lines[i], boost::is_any_of(" ,\t"), boost::token_compress_on);
        if(parts.size() < 3)
            continue;
        //std::cout << "MvnPln.->Parsing splitted line: " << lines[i] << std::endl;
        parseSuccess = true;
        std::stringstream ssX(parts[1]);
        if(!(ssX >> locX)) parseSuccess = false;
        std::stringstream ssY(parts[2]);
        if(!(ssY >> locY)) parseSuccess = false;
        loc.push_back(locX);
        loc.push_back(locY);
        if(parts.size() >= 4)
        {
            std::stringstream ssAngle(parts[3]);
            if(!(ssAngle >> locAngle)) parseSuccess = false;
            loc.push_back(locAngle);
        }

        if(parseSuccess)
        {
            this->locations[parts[0]] = loc;
        }
    }
    std::cout << "MvnPln.->Total number of known locations: " << this-locations.size() << std::endl;
    for(std::map<std::string, std::vector<float> >::iterator it=this->locations.begin(); it != this->locations.end(); it++)
    {
        std::cout << "MvnPln.->Location " << it->first << " " << it->second[0] << " " << it->second[1];
        if(it->second.size() > 2)
            std::cout << " " << it->second[2];
        std::cout << std::endl;
    }
    if(this->locations.size() < 1)
        std::cout << "MvnPln.->WARNING: Cannot load known locations from file: " << path << ". There are no known locations." << std::endl;
    
    return true;
}

void MvnPln::spin()
{
    ros::Rate loop(10);
    int currentState = SM_INIT;
    float robotX, robotY, robotTheta;
    float angleError;
    std_msgs::Bool msgGoalReached;
    bool pathSuccess = false;
    float lateralMovement;

    while(ros::ok())
    {
        if(this->stopReceived)
        {
            this->stopReceived = false;
            currentState = SM_INIT;
        }
        switch(currentState)
        {
        case SM_INIT:
            std::cout << "MvnPln.->Current state: " << currentState << ". Waiting for new task..." << std::endl;
            currentState = SM_WAITING_FOR_NEW_TASK;
            break;
        case SM_WAITING_FOR_NEW_TASK:
            if(this->newTask)
            {
                std::cout << "MvnPln.->New task received..." << std::endl;
                this->newTask = false;
                currentState = SM_CALCULATE_PATH;
            }
            break;
        case SM_CALCULATE_PATH:
            std::cout << "MvnPln.->Current state: " << currentState << ". Calculating path using map, kinect and laser" << std::endl;
            std::cout << "MvnPl.->Moving backwards if there is an obstacle before calculating path" << std::endl;
            if(JustinaNavigation::obstacleInFront())
                JustinaNavigation::moveDist(-0.15, 5000);
            if(JustinaNavigation::obstacleInFront())
                JustinaNavigation::moveDist(-0.15, 5000);
            if(JustinaNavigation::obstacleInFront())
                JustinaNavigation::moveDist(-0.15, 5000);
            if(JustinaNavigation::obstacleInFront())
                JustinaNavigation::moveDist(-0.15, 5000);
            std::cout << "MvnPln.->Moving head to search for obstacles in front of the robot" << std::endl;
            JustinaManip::hdGoTo(0, -0.9, 2500);
            
            JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
            pathSuccess = this->planPath(robotX, robotY, this->goalX, this->goalY, this->lastCalcPath);
            if(!pathSuccess)
            {
                std::cout<<"MvnPln.->Cannot calc path to "<<this->goalX<<" "<<this->goalY<<" after several attempts" << std::endl;
                JustinaManip::hdGoTo(0, 0, 2500);
                msgGoalReached.data = false;
                this->pubGlobalGoalReached.publish(msgGoalReached);
                currentState = SM_INIT;
            }
            else
                currentState = SM_START_MOVE_PATH;
            break;
        case SM_START_MOVE_PATH:
            std::cout << "MvnPln.->Current state: " << currentState << ". Starting move path" << std::endl;
            std::cout << "MvnPln.->Turning on collision detection..." << std::endl;
            this->collisionDetected = false;
            JustinaNavigation::enableObstacleDetection(true);
            JustinaNavigation::startMovePath(this->lastCalcPath);
            currentState = SM_WAIT_FOR_MOVE_FINISHED;
            break;
        case SM_WAIT_FOR_MOVE_FINISHED:
            if(JustinaNavigation::isGoalReached())
            {
                std::cout << "MvnPln.->Move path finished succesfully. " << std::endl;
                JustinaNavigation::enableObstacleDetection(false);
                if(this->correctFinalAngle) //This flag is set in the callbacks
                    currentState = SM_CORRECT_FINAL_ANGLE;
                else
                {
                    std::cout << "MnvPln.->Goal point reached successfully!!!!!!!" << std::endl;
                    JustinaNavigation::enableObstacleDetection(false);
                    JustinaManip::hdGoTo(0, 0, 2500);
                    msgGoalReached.data = true;
                    this->pubGlobalGoalReached.publish(msgGoalReached);
                    currentState = SM_INIT;
                }
            }
            else if(this->collisionDetected)
            {
                std::cout << "MvnPln.->COLLISION RISK DETECTED before goal is reached." << std::endl;
                this->collisionDetected = false;
                currentState = SM_COLLISION_DETECTED;
            }
            else if(this->stopReceived)
            {
                std::cout << "MvnPln.->Stop signal received..." << std::endl;
                JustinaNavigation::enableObstacleDetection(false);
                JustinaManip::hdGoTo(0, 0, 2500);
                msgGoalReached.data = false;
                this->pubGlobalGoalReached.publish(msgGoalReached);
                currentState = SM_INIT;
            }
            break;
        case SM_COLLISION_DETECTED:
            std::cout << "MvnPln.->Current state: " << currentState << ". Stopping robot smoothly" << std::endl;
            JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
            //If robot is 0.6 near the goal, it is considered that it has reached the goal
            if(sqrt((robotX - this->goalX)*(robotX - this->goalX) + (robotY - this->goalY)*(robotY - this->goalY)) < 0.3)
            {
                if(this->correctFinalAngle) //This flag is set in the callbacks
                    currentState = SM_CORRECT_FINAL_ANGLE;
                else
                {
                    std::cout << "MnvPln.->Goal point reached successfully!!!!!!!" << std::endl;
                    JustinaNavigation::enableObstacleDetection(false);
                    JustinaManip::hdGoTo(0, 0, 2500);
                    msgGoalReached.data = true;
                    this->pubGlobalGoalReached.publish(msgGoalReached);
                    currentState = SM_INIT;
                }
            }
            else
            {
                if(this->collisionDetected)
                {
                    JustinaNavigation::moveDist(-0.20, 5000);
		    if(this->collisionPointY < 0)
		      lateralMovement = 0.25 + this->collisionPointY + 0.051;
		    else
		      lateralMovement = this->collisionPointY - 0.25 - 0.051;
		    if(lateralMovement > 0.15)
		      lateralMovement = 0.15;
		    if(lateralMovement < -0.15)
		       lateralMovement = -0.15;
		    JustinaNavigation::moveLateral(lateralMovement, 5000);
                    JustinaNavigation::moveDist(0.02, 2500);
                }
                currentState = SM_CALCULATE_PATH;
            }
            break;
        case SM_CORRECT_FINAL_ANGLE:
            std::cout << "MvnPln.->CurrentState: " << currentState << ". Correcting final angle" << std::endl;
            JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
            angleError = this->goalAngle - robotTheta;
            if(angleError > M_PI) angleError -= 2*M_PI;
            if(angleError <= -M_PI) angleError += 2*M_PI;
            JustinaNavigation::startMoveDistAngle(0.0, angleError);
            currentState = SM_WAIT_FOR_ANGLE_CORRECTED;
            break;
        case SM_WAIT_FOR_ANGLE_CORRECTED:
            if(JustinaNavigation::isGoalReached())
            {
                std::cout << "MvnPln.->Angle correction finished succesfully. " << std::endl;
                std::cout << "MnvPln.->Goal point reached successfully!!!!!!!" << std::endl;
                JustinaNavigation::enableObstacleDetection(false);
                msgGoalReached.data = true;
                JustinaManip::hdGoTo(0, 0, 2500);
                this->pubGlobalGoalReached.publish(msgGoalReached);
                currentState = SM_INIT;
            }
            break;
        }
        
        this->pubLocationMarkers.publish(this->getLocationMarkers());
        if(!this->isLastPathPublished)
        {
            this->pubLastPath.publish(this->lastCalcPath);
            this->isLastPathPublished = true;
        }
        ros::spinOnce();
        loop.sleep();
    }
}

visualization_msgs::Marker MvnPln::getLocationMarkers()
{
    visualization_msgs::Marker m;
    m.ns = "map_locations";
    m.header.frame_id = "map";
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration();
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    m.scale.x = 0.15;
    m.scale.y = 0.15;
    m.scale.z = 0.15;
    int id = 0;
    for(std::map<std::string, std::vector<float> >::iterator it=this->locations.begin(); it != this->locations.end(); it++)
    {
        m.id = id;
        geometry_msgs::Point p;
        p.x = it->second[0];
        p.y = it->second[1];
        p.z = 0.075;
        m.points.push_back(p);
    }
    return m;
}

bool MvnPln::planPath(float startX, float startY, float goalX, float goalY, nav_msgs::Path& path)
{
    bool pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, true, true, true);
    if(!pathSuccess)
        pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, true, true, false);
    if(!pathSuccess)
        pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, true, false, true);
    if(!pathSuccess)
        pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, true, false, false);
    if(!pathSuccess)
        pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, false, true, true);
    if(!pathSuccess)
        pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, false, true, false);
    /*
    if(!pathSuccess)
    {
        std::cout<<"MvnPln.->Cannot calc path to "<< goalX<<" "<< goalY<<" using map laser and kinect" << std::endl;
        pathSuccess = this->planPath(startX, startY, goalX, goalY, path, true, true, false);
    }
    if(!pathSuccess)
    {
        std::cout<<"MvnPln.->Cannot calc path to "<< goalX<<" "<< goalY<<" using only map and laser" << std::endl;
        pathSuccess = this->planPath(startX, startY, goalX, goalY, path, true, false, false);
    }
    if(!pathSuccess)
    {
        std::cout<<"MvnPln.->Cannot calc path to "<< goalX<<" "<< goalY<<" using only map" << std::endl;
        pathSuccess = this->planPath(startX, startY, goalX, goalY, path, false, true, true);
    }
    if(!pathSuccess)
    {
        std::cout<<"MvnPln.->Cannot calc path to "<< goalX<<" "<< goalY<<" using only laser and kinect" << std::endl;
        pathSuccess = this->planPath(startX, startY, goalX, goalY, path, false, true, false);
    }
    if(!pathSuccess)
        std::cout << "MvnPln.->Cannot calculate path using only laser :(" << std::endl;*/
    return pathSuccess;
}

bool MvnPln::planPath(float startX, float startY, float goalX, float goalY, nav_msgs::Path& path,
                      bool useMap, bool useLaser, bool useKinect)
{ 
    std::cout << "MvnPln.->Calculating path with augmented map..." << std::endl;
    nav_msgs::OccupancyGrid augmentedMap;

    //
    //If use_map, then gets the map from map_server
    if(useMap)
    {
        nav_msgs::GetMap srvGetMap;        
        std::cout << "MvnPln.->Getting occupancy grid from map server... " << std::endl;
        if(!this->cltGetMap.call(srvGetMap))
        {
            std::cout << "MvnPln.->Cannot get map from map_server." << std::endl;
            return false;
        }
        augmentedMap = srvGetMap.response.map;
    }
    else
    {
        augmentedMap.header.frame_id = "base_link";
        augmentedMap.info.resolution = 0.05;
        augmentedMap.info.width = 1000;
        augmentedMap.info.height = 1000;
        augmentedMap.info.origin.position.x = -25.0;
        augmentedMap.info.origin.position.y = -25.0;
        augmentedMap.data.resize(augmentedMap.info.width*augmentedMap.info.height);
	for (size_t i=0; i < augmentedMap.data.size(); i++)
	    augmentedMap.data[i] = 0;
    }
    float mapOriginX = augmentedMap.info.origin.position.x;
    float mapOriginY = augmentedMap.info.origin.position.y;
    float mapResolution = augmentedMap.info.resolution;
    int mapWidth = augmentedMap.info.width;

    //
    //If use-laser, then set as occupied the corresponding cells
    if(useLaser)
    {
        std::cout << "MvnPln.->Merging laser scan with occupancy grid" << std::endl;
        float robotX, robotY, robotTheta;
        float angle, laserX, laserY;
        int idx;
        JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
        for(int i=0; i < lastLaserScan.ranges.size(); i++)
        {
            if(lastLaserScan.ranges[i] > 0.8 ||  lastLaserScan.ranges[i] < 0.25)
                continue;
            angle = lastLaserScan.angle_min + i*lastLaserScan.angle_increment;
            if(fabs(angle) > 1.5708)
                continue;
            //For each range, cells are free between the robot and the end of the ray
	    /*
            for(float dist=0; dist < lastLaserScan.ranges[i]; dist+=0.05)
            {
                laserX = robotX + dist*cos(angle + robotTheta);
                laserY = robotY + dist*sin(angle + robotTheta);
                idx = (int)((laserY - mapOriginY)/mapResolution) * mapWidth + (int)((laserX - mapOriginX)/mapResolution);
                if(idx >= augmentedMap.data.size() || idx < 0)
                    continue;
                augmentedMap.data[idx] = 0;
            }*/
            //Only the end of the ray is occupied
            laserX = robotX + lastLaserScan.ranges[i]*cos(angle + robotTheta);
            laserY = robotY + lastLaserScan.ranges[i]*sin(angle + robotTheta);
            idx = (int)((laserY - mapOriginY)/mapResolution) * mapWidth + (int)((laserX - mapOriginX)/mapResolution);
            if(idx >= augmentedMap.data.size() || idx < 0)
                continue;
            augmentedMap.data[idx] = 100;
        }
    }

    if(useKinect)
    {
        std::cout << "MvnPln.->Using cloud to augment map" << std::endl;
        point_cloud_manager::GetRgbd srvGetRgbd;
        if(!this->cltGetRgbdWrtRobot.call(srvGetRgbd))
        {
            std::cout << "MvnPln.->Cannot get point cloud :'(" << std::endl;
            return false;
        }
        pcl::PointCloud<pcl::PointXYZRGBA> cloudWrtRobot;
        pcl::PointCloud<pcl::PointXYZRGBA> cloudWrtMap;
        pcl::fromROSMsg(srvGetRgbd.response.point_cloud, cloudWrtRobot);
        tf::StampedTransform transformTf;
        tf_listener.lookupTransform("map", "base_link", ros::Time(0), transformTf);
        Eigen::Affine3d transformEigen;
        tf::transformTFToEigen(transformTf, transformEigen);
        pcl::transformPointCloud(cloudWrtRobot, cloudWrtMap, transformEigen);
        //It augments the map using only a rectangle in front of the robot
        float minX = 0.25;
        float maxX = 0.8;
        float minY = -0.35;
        float maxY = 0.35;
        int counter = 0;
        int idx;
        for(size_t i=0; i<cloudWrtRobot.points.size(); i++)
        {
            pcl::PointXYZRGBA pR = cloudWrtRobot.points[i];
            pcl::PointXYZRGBA pM = cloudWrtMap.points[i];
            idx = (int)((pM.y - mapOriginY)/mapResolution)*mapWidth + (int)((pM.x - mapOriginX)/mapResolution);
            if(pR.x > minX && pR.x < maxX && pR.y > minY && pR.y < maxY && idx < augmentedMap.data.size() && idx >= 0)
            {
                if(pR.z > 0.05)
                    augmentedMap.data[idx] = 100;
                //else
		//augmentedMap.data[idx] = 0;
            }
        }
    }

    navig_msgs::PathFromMap srvPathFromMap;
    srvPathFromMap.request.map = augmentedMap;
    srvPathFromMap.request.start_pose.position.x = startX;
    srvPathFromMap.request.start_pose.position.y = startY;
    srvPathFromMap.request.goal_pose.position.x = goalX;
    srvPathFromMap.request.goal_pose.position.y = goalY;

    bool success;
    if((success = this->cltPathFromMapAStar.call(srvPathFromMap)))
        std::cout << "MvnPln.->Path calculated succesfully by path_calculator using A* using map and laser" << std::endl;
    else
        std::cout << "MvnPln.->Cannot calculate path by path_calculator using A* using map and laser" << std::endl;
    ros::spinOnce();

    path = srvPathFromMap.response.path;
    this->lastCalcPath = path;
    this->isLastPathPublished = false;
    return success;
}

void MvnPln::callbackRobotStop(const std_msgs::Empty::ConstPtr& msg)
{
    this->stopReceived = true;
}

bool MvnPln::callbackPlanPath(navig_msgs::PlanPath::Request& req, navig_msgs::PlanPath::Response& resp)
{
    //If Id is "", then, the metric values are used
    std::cout << "MvnPln.->Plan Path from ";
    if(req.start_location_id.compare("") == 0)
        std::cout << req.start_pose.position.x << " " << req.start_pose.position.y << " to ";
    else std::cout << "\"" << req.start_location_id << "\" to ";
    if(req.goal_location_id.compare("") == 0)
        std::cout << req.goal_pose.position.x << " " << req.goal_pose.position.y << std::endl;
    else std::cout << "\"" << req.goal_location_id << "\"" << std::endl;

    float startX, startY, goalX, goalY;
    if(req.start_location_id.compare("") != 0) //Then, start location should be a known location
    {
        if(this->locations.find(req.start_location_id) == this->locations.end())
        {
            std::cout << "MvnPln.->Cannot calculate path from \"" << req.start_location_id << "\". It is not a known location. " << std::endl;
            return false;
        }
        startX = this->locations[req.start_location_id][0];
        startY = this->locations[req.start_location_id][1];
    }
    else //Then, start location is given in coordinates
    {
        startX = req.start_pose.position.x;
        startY = req.start_pose.position.y;
    }

    if(req.goal_location_id.compare("") != 0) //Then, goal location should be a known location
    {
        if(this->locations.find(req.goal_location_id) == this->locations.end())
        {
            std::cout << "MvnPln.->Cannot calculate path to \"" << req.goal_location_id << "\". It is not a known location. " << std::endl;
            return false;
        }
        goalX = this->locations[req.goal_location_id][0];
        goalY = this->locations[req.goal_location_id][1];
    }
    else //Then, goal location is given in coordinates
    {
        goalX = req.goal_pose.position.x;
        goalY = req.goal_pose.position.y;
    }

    return this->planPath(startX, startY, goalX, goalY, resp.path);
}

void MvnPln::callbackClickedPoint(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    std::cout << "MvnPln.->Clicked point: " << msg->point.x << " " << msg->point.y << std::endl;
}

void MvnPln::callbackGetCloseLoc(const std_msgs::String::ConstPtr& msg)
{
    if(this->locations.find(msg->data) == this->locations.end())
    {
        std::cout << "MvnPln.->Cannot get close to \"" << msg->data << "\". It is not a known location. " << std::endl;
        return;
    }
    this->goalX = this->locations[msg->data][0];
    this->goalY = this->locations[msg->data][1];
    if(this->correctFinalAngle = this->locations[msg->data].size() > 2)
        this->goalAngle = this->locations[msg->data][2];
    this->newTask = true;
    
    std::cout << "MvnPln.->Received desired goal pose: " << msg->data << ": " << this->goalX << " " << this->goalY;
    if(this->correctFinalAngle)
        std::cout << " " << this->goalAngle;

    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubGlobalGoalReached.publish(msgGoalReached);
    std::cout << std::endl;
}

void MvnPln::callbackGetCloseXYA(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //If msg has two values, the robot will try to reach the goal point without correcting the final angle
    //If it has three values, the third one will be the final desired angle.
    if(msg->data.size() < 2)
    {
        std::cout << "MvnPln.->Cannot get close to given coordinates. At least two values are required." << std::endl;
        return;
    }
    this->goalX = msg->data[0];
    this->goalY = msg->data[1];
    if(this->correctFinalAngle = msg->data.size() > 2)
        this->goalAngle = msg->data[2];
    this->newTask = true;
    
    std::cout << "MvnPln.->Received desired goal pose: " << this->goalX << " " << this->goalY;
    if(this->correctFinalAngle)
        std::cout << " " << this->goalAngle;

    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubGlobalGoalReached.publish(msgGoalReached);
    std::cout << std::endl;
}

void MvnPln::callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    this->lastLaserScan = *msg;
}

void MvnPln::callbackCollisionRisk(const std_msgs::Bool::ConstPtr& msg)
{
    //std::cout << "JustinaNvigation.-<CollisionRisk: " << int(msg->data) << std::endl;
    //Collision signal is set to true in a flip-flop manner to ensure the correct sequence in the state machine
    //Whenever a collision is detected, the flag is kept until it is cleared in the state machine
    if(msg->data)
        this->collisionDetected = true;
}

void MvnPln::callbackAddLocation(const navig_msgs::Location::ConstPtr& msg)
{
    //if(this->locations.find(msg->id) != this->locations.end())
    //{
    //    std::cout << "MvnPln.->Cannot add \"" << msg->id << "\" location: Duplicated name. " << std::endl;
    //    return;
    //}
    std::cout << "MvnPln.->Add predefined location: " << msg->position.x << "  " << msg->position.y << "  ";
    if(msg->correct_angle)
        std::cout << msg->orientation << std::endl;
    else
        std::cout << std::endl;
    std::vector<float> p;
    p.push_back(msg->position.x);
    p.push_back(msg->position.y);
    if(msg->correct_angle)
        p.push_back(msg->orientation);
    this->locations[msg->id] = p;
}

void MvnPln::callbackCollisionPoint(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  this->collisionPointX = msg->point.x;
  this->collisionPointY = msg->point.y;
}
