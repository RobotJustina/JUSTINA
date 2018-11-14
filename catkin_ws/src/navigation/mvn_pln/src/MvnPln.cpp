#include "MvnPln.h"

MvnPln::MvnPln()
{
    this->newTask = false;
    this->resetTask = false;
    this->correctFinalAngle = false;
    this->collisionDetected = false;
    this->stopReceived = false;
    this->isLastPathPublished = false;
    this->timeoutAvoidanceChair = 5000;
    this->_allow_move_lateral = false;
    this->max_attempts = 0;
    this->_clean_goal_map = false;
    this->_avoidance_type_obstacle = false;
    this->countObstType["person"] = 0;
    this->countObstType["vase"] = 0;
    this->countObstType["sports ball"] = 0;
    this->countObstType["refrigerator"] = 0;
    this->countObstType["unknown"] = 0;
    this->framesCount = 0;
    this->_max_frames_count = 10;
    this->_min_frames_avoidance = 7;
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
    this->subClickedPoint = nh->subscribe("/clicked_point", 1, &MvnPln::callbackClickedPoint, this);
    this->subEnableAvoidanceTypeObstacle = nh->subscribe("/navigation/mvn_pln/enable_avoidance_type_obstacle", 1, &MvnPln::callbackEnableAvoidanceTypeObstacle, this);
    this->subRobotStop = nh->subscribe("/hardware/robot_state/stop", 10, &MvnPln::callbackRobotStop, this);
    this->pubGlobalGoalReached = nh->advertise<std_msgs::Bool>("/navigation/global_goal_reached", 10);
    this->pubStopWaitGlobalGoalReached = nh->advertise<std_msgs::Empty>("/navigation/stop_wait_global_goal_reached", 1);
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

void MvnPln::spin()
{
    ros::Rate loop(10);
    int currentState = SM_INIT;
    float robotX, robotY, robotTheta;
    float angleError;
    std_msgs::Bool msgGoalReached;
    bool pathSuccess = false;
    float lateralMovement;
    int collision_detected_counter = 0;
                
    std::vector<vision_msgs::VisionObject> yoloObjects;
    std::vector<vision_msgs::VisionObject>::iterator yoloObjectsIt;
	boost::posix_time::ptime prev;
	boost::posix_time::ptime curr;

    while(ros::ok())
    {
        if(this->stopReceived)
        {
            this->stopReceived = false;
            currentState = SM_INIT;
        }
        if(resetTask)
        {
            resetTask = false;
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
                    msgGoalReached.data = false;
                    this->pubGlobalGoalReached.publish(msgGoalReached);
                    currentState = SM_CALCULATE_PATH;
                    collision_detected_counter = 0;
                }
                break;
            case SM_CALCULATE_PATH:
                std::cout << "MvnPln.->Current state: " << currentState << ". Calculating path using map, kinect and laser" << std::endl;
                std::cout << "MvnPl.->Moving backwards if there is an obstacle before calculating path" << std::endl;
                if(JustinaNavigation::obstacleInFront())
                    JustinaNavigation::moveDist(-0.2, 5000);
                if(JustinaNavigation::obstacleInFront())
                    JustinaNavigation::moveDist(-0.2, 5000);
                //if(JustinaNavigation::obstacleInFront())
                //    JustinaNavigation::moveDist(-0.15, 5000);
                //if(JustinaNavigation::obstacleInFront())
                //    JustinaNavigation::moveDist(-0.15, 5000);
                std::cout << "MvnPln.->Moving head to search for obstacles in front of the robot" << std::endl;
                JustinaManip::hdGoTo(0, -0.9, 2500);
                //JustinaManip::hdGoTo(0, -0.9, 2500);
                //JustinaManip::hdGoTo(0, -0.9, 2500);
                JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
                pathSuccess = this->planPath(robotX, robotY, this->goalX, this->goalY, this->lastCalcPath);
                //JustinaIROS::loggingTrajectory(this->lastCalcPath);
                if(!pathSuccess)
                {
                    std::cout<<"MvnPln.->Cannot calc path to "<<this->goalX<<" "<<this->goalY<<" after several attempts" << std::endl;
                    JustinaManip::hdGoTo(0, 0, 2500);
                    msgGoalReached.data = false;
                    this->pubGlobalGoalReached.publish(msgGoalReached);
                    this->pubStopWaitGlobalGoalReached.publish(std_msgs::Empty());
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
                        if(!this->_avoidance_type_obstacle)
                            JustinaNavigation::moveDist(-0.10, 5000);
                        if(this->collisionPointY < 0)
                            lateralMovement = 0.25 + this->collisionPointY + 0.051;
                        else
                            lateralMovement = this->collisionPointY - 0.25 - 0.051;
                        //if(lateralMovement > 0.15)
                        //    lateralMovement = 0.15;
                        //if(lateralMovement < -0.15)
                        //    lateralMovement = -0.15;
                        if(this->_allow_move_lateral)
                            JustinaNavigation::moveLateral(lateralMovement, 5000);
                        //JustinaNavigation::moveDist(0.05, 2500);
                    }
                    if(this->_avoidance_type_obstacle)
                    {
                        this->countObstType["person"] = 0;
                        this->countObstType["sports ball"] = 0;
                        this->countObstType["refrigerator"] = 0;
                        this->countObstType["vase"] = 0;
                        this->countObstType["unknown"] = 0;
                        this->framesCount = 0;
                        //JustinaManip::hdGoTo(0, -0.6, 2000);
                        currentState = SM_DETECT_OBSTACLE;
                    }
                    else
                        currentState = SM_CALCULATE_PATH;
                    if(++collision_detected_counter > this->max_attempts)
                    {
                        std::cout << "MnvPln.->Max attempts after collision detected reached!! max_attempts= " << this->max_attempts << std::endl;
                        JustinaNavigation::enableObstacleDetection(false);
                        JustinaManip::hdGoTo(0, 0, 2500);
                        msgGoalReached.data = true;
                        this->pubGlobalGoalReached.publish(msgGoalReached);
                        currentState = SM_INIT;
                    }
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
            case SM_DETECT_OBSTACLE:
                std::cout << "MvnPln.->CurrentState: " << currentState << ". Detecting object in front" << std::endl;
                JustinaVision::detectObjectsYOLO(yoloObjects);
                framesCount++;
                for(yoloObjectsIt = yoloObjects.begin(); yoloObjectsIt != yoloObjects.end(); yoloObjectsIt++)
                {
                    if(yoloObjectsIt->pose.position.x != 0 && yoloObjectsIt->pose.position.y != 0 && yoloObjectsIt->pose.position.z != 0)
                    {
                        if(yoloObjectsIt->pose.position.x >= kinect_minX && yoloObjectsIt->pose.position.x <= kinect_maxX + 0.4 && 
                                yoloObjectsIt->pose.position.y >= (kinect_minY - 0.2) && yoloObjectsIt->pose.position.y <= (kinect_maxY + 0.2)) 
                        {
                            std::cout << "MvnPln.->CurrentState: " << currentState << ". have detected object in front: " << yoloObjectsIt->id << std::endl;
                            std::map<std::string, int>::iterator countObstTypeIt = countObstType.find(yoloObjectsIt->id);
                            if(countObstTypeIt != countObstType.end())
                                countObstTypeIt->second += 1;
                            else
                            {
                                countObstTypeIt = countObstType.find("unknown");
                                if(countObstTypeIt != countObstType.end())
                                    countObstTypeIt->second += 1;
                            }

                        }
                    }
                }

                if(framesCount == _max_frames_count)
                {
                    if(     this->countObstType["person"] >= _min_frames_avoidance ||
                            this->countObstType["sports ball"] >= _min_frames_avoidance ||
                            this->countObstType["refrigerator"] >= _min_frames_avoidance ||
                            this->countObstType["vase"] >= _min_frames_avoidance)
                        this->countObstType["unknown"] = 0;

                    this->countObstType["refrigerator"] = 0;
                    this->countObstType["sports ball"] = this->countObstType["sports ball"] + this->countObstType["vase"];
                    std::map<std::string, int>::iterator maxIt = std::max_element(this->countObstType.begin(), this->countObstType.end(), map_compare);
                    if(maxIt->second >= _min_frames_avoidance)
                    {
                        if(maxIt->first.compare("unknown") == 0){
                            JustinaVision::enableDetectObjsYOLO(false);
                            currentState = SM_CALCULATE_PATH;
                        }else if(maxIt->first.compare("person") == 0){
                            std::cout << "MvnPln.->CurrentState: " << currentState << ". have detected human in front: " << std::endl;
                            currentState = SM_AVOIDANCE_HUMAN;
                        }
                        else if(maxIt->first.compare("sports ball") == 0 || maxIt->first.compare("vase") == 0)
                            currentState = SM_AVOIDANCE_CHAIR;
                        /*else if(maxit->first.compare("BAG"))
                            currentState = SM_AVOIDANCE_BAG;*/
                    }
                    else{
                        std::cout << "MvnPln.->CurrentState: " << currentState << ". have not detected object in front: " << std::endl;
                        currentState = SM_CALCULATE_PATH;
                    }
                    framesCount = 0;
                    this->countObstType["person"] = 0;
                    this->countObstType["sports ball"] = 0;
                    this->countObstType["refrigerator"] = 0;
                    this->countObstType["vase"] = 0;
                    this->countObstType["unknown"] = 0;
                }
                break;
            case SM_AVOIDANCE_HUMAN:
                std::cout << "MvnPln.->CurrentState: " << currentState << ". Avoidance Human" << std::endl;
                JustinaHRI::waitAfterSay("Human, please not interfere with my navigation behavior, please move", 3000);
                currentState = SM_WAIT_FOR_MOVE_HUMAN;
                break;
            case SM_WAIT_FOR_MOVE_HUMAN:
                std::cout << "MvnPln.->CurrentState: " << currentState << ". Wait for Move Human" << std::endl;
                JustinaVision::detectObjectsYOLO(yoloObjects);
                framesCount++;
                for(yoloObjectsIt = yoloObjects.begin(); yoloObjectsIt != yoloObjects.end(); yoloObjectsIt++)
                {
                    if(yoloObjectsIt->id.compare("person") == 0)
                    {
                        if(yoloObjectsIt->pose.position.x != 0 && yoloObjectsIt->pose.position.y != 0 && yoloObjectsIt->pose.position.z != 0)
                        {
                            if(yoloObjectsIt->pose.position.x >= kinect_minX && yoloObjectsIt->pose.position.x <= kinect_maxX + 0.4 && 
                                    yoloObjectsIt->pose.position.y >= (kinect_minY - 0.2) && yoloObjectsIt->pose.position.y <= (kinect_maxY + 0.2))
                            {
                                std::map<std::string, int>::iterator countObstTypeIt = countObstType.find("person");
                                if(countObstTypeIt != countObstType.end())
                                    countObstTypeIt->second += 1;
                            }
                        }
                    }
                }
                if(framesCount == _max_frames_count)
                {
                    std::map<std::string, int>::iterator countObstTypeIt = countObstType.find("person");
                    if(countObstTypeIt != countObstType.end())
                    {
                        if(countObstTypeIt->second < _min_frames_avoidance)
                        {
                            JustinaVision::enableDetectObjsYOLO(false);
                            currentState = SM_CALCULATE_PATH;
                        }else
                            currentState = SM_AVOIDANCE_HUMAN;
                    }
                    else
                    {
                        JustinaVision::enableDetectObjsYOLO(false);
                        currentState = SM_CALCULATE_PATH;
                    }
                    framesCount = 0;
                    this->countObstType["person"] = 0;
                }
                break;
            case SM_AVOIDANCE_CHAIR:
                std::cout << "MvnPln.->CurrentState: " << currentState << ". Avoidance chair" << std::endl;
                JustinaHRI::waitAfterSay("I detect a pouf in my path, I will try to move it", 3000);
                //JustinaNavigation::moveDist(1.0, 6000);
                currentState = SM_CALCULATE_PATH_AVOIDANCE_CHAIR;
                break;
            case SM_CALCULATE_PATH_AVOIDANCE_CHAIR:
                std::cout << "MvnPln.->CurrentState: " << currentState << ". Calculate path to avoidance chair" << std::endl;
                JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
                pathSuccess = this->planPath(robotX, robotY, this->goalX, this->goalY, this->lastCalcPath, true, false, false);
                //JustinaIROS::loggingTrajectory(this->lastCalcPath);
                if(!pathSuccess)
                {
                    std::cout<<"MvnPln.->Cannot calc path to "<<this->goalX<<" "<<this->goalY<<" after several attempts" << std::endl;
                    JustinaNavigation::moveDist(0.7, 6000);
                    currentState = SM_FINISH_FOR_MOVE_CHAIR;
                }
                else
                    currentState = SM_START_AVOIDANCE_CHAIR;
                break;
            case SM_START_AVOIDANCE_CHAIR:
                std::cout << "MvnPln.->CurrentState: " << currentState << ". Start avoidance chair" << std::endl;
                this->collisionDetected = false;
                JustinaNavigation::enableObstacleDetection(false);
                JustinaNavigation::startMovePath(this->lastCalcPath);
                prev = boost::posix_time::second_clock::local_time();
                currentState = SM_WAIT_FOR_MOVE_CHAIR;
                break;
            case SM_WAIT_FOR_MOVE_CHAIR:
                std::cout << "MvnPln.->CurrentState: " << currentState << ". Wait for move chair" << std::endl;
                curr = boost::posix_time::second_clock::local_time();
                if ((curr - prev).total_milliseconds() >= timeoutAvoidanceChair)
                    currentState = SM_FINISH_FOR_MOVE_CHAIR;
                break;
            case SM_FINISH_FOR_MOVE_CHAIR:
                std::cout << "MvnPln.->CurrentState: " << currentState << ". Finish avoidance Chair" << std::endl;
                JustinaNavigation::enableObstacleDetection(true);
                JustinaHRI::waitAfterSay("I have moved the pouf, I will update my path", 3000);
                currentState = SM_CALCULATE_PATH;
                break;
            case SM_AVOIDANCE_BAG:
                std::cout << "MvnPln.->CurrentState: " << currentState << ". Avoidance Bag" << std::endl;
                currentState = SM_CALCULATE_PATH;
                break;
        }

        if(!this->isLastPathPublished)
        {
            this->pubLastPath.publish(this->lastCalcPath);
            this->isLastPathPublished = true;
        }
        //JustinaIROS::loggingRobotPose();
        ros::spinOnce();
        loop.sleep();
    }
}

void MvnPln::allow_move_lateral(bool _allow_move_lateral)
{
    this->_allow_move_lateral = _allow_move_lateral;
}

void MvnPln::clean_goal_map(bool _clean_goal_map)
{
    this->_clean_goal_map = _clean_goal_map;
}

void MvnPln::look_at_goal(bool _look_at_goal){
    this->_look_at_goal = _look_at_goal;
}

void MvnPln::clean_unexplored_map(bool _clean_unexplored_map){
    this->_clean_unexplored_map = _clean_unexplored_map;
}

void MvnPln::avoidance_type_obstacle(bool _avoidance_type_obstacle){
    this->_avoidance_type_obstacle = _avoidance_type_obstacle;
}

bool MvnPln::planPath(float startX, float startY, float goalX, float goalY, nav_msgs::Path& path)
{
    //bool pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, true, true, true);
    //if(!pathSuccess)
    bool pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, true, false, true);
    if(!pathSuccess)
        pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, true, true, false);
    if(!pathSuccess)
        pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, false, true, true);
    if(!pathSuccess)
        pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, false, false, true);
    if(!pathSuccess)
        pathSuccess =  this->planPath(startX, startY, goalX, goalY, path, false, true, false);
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
            if(lastLaserScan.ranges[i] > 0.8 ||  lastLaserScan.ranges[i] < 0.3)
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
        if(_look_at_goal){
            float robotX, robotY, robotTheta;
            JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
            JustinaManip::hdGoTo(atan2(goalY - robotY, goalX - robotX) - robotTheta, -0.2, 3000);
            if(!fillMapWithKinect(augmentedMap))
                return false;
            JustinaManip::hdGoTo(0.0, -0.9, 3000);
        }
        if(!fillMapWithKinect(augmentedMap))
            return false;
    }

    if(_clean_goal_map){
		std::cout << "MvnPln->Cleaning the cells and neighbors of the goal destination." << std::endl;
		int goalCellX = (int)((goalX - augmentedMap.info.origin.position.x)/augmentedMap.info.resolution);
		int goalCellY = (int)((goalY - augmentedMap.info.origin.position.y)/augmentedMap.info.resolution);
		int goalCell = goalCellY * augmentedMap.info.width + goalCellX;

		augmentedMap.data[goalCell] = 0;

		float growDist = 0.3; // This is the raduis of robot
		int growSteps = (int)(growDist / augmentedMap.info.resolution);
		int boxSize = (2 * growSteps + 1) * (2 * growSteps + 1);
		int* neighbors = new int[boxSize];
		int counter = 0;

		for(int i=-growSteps; i<=growSteps; i++)
			for(int j=-growSteps; j<=growSteps; j++)
			{
				neighbors[counter] = j * augmentedMap.info.width + i;
				counter++;
			}

		for(int j=0; j < boxSize; j++)
			augmentedMap.data[goalCell + neighbors[j]] = 0;
	}
	else
		std::cout << "MvnPln->Not clean the cells and neighbors of the goal destinetion." << std::endl;

	if(_clean_unexplored_map){
		for (size_t i=0; i < augmentedMap.data.size(); i++)
			if(augmentedMap.data[i] < 0)
				augmentedMap.data[i] = 0;
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

bool MvnPln::fillMapWithKinect(nav_msgs::OccupancyGrid &augmentedMap){
    std::cout << "MvnPln.->Using cloud to augment map" << std::endl;
    float mapOriginX = augmentedMap.info.origin.position.x;
    float mapOriginY = augmentedMap.info.origin.position.y;
    float mapResolution = augmentedMap.info.resolution;
    int mapWidth = augmentedMap.info.width;
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
    int counter = 0;
    int idx;

    // TODO REMOVE THIS PRINTS
    std::cout << "MvnPln.->kinect_minX:" << kinect_minX << std::endl;
    std::cout << "MvnPln.->kinect_maxX:" << kinect_maxX << std::endl;
    std::cout << "MvnPln.->kinect_minY:" << kinect_minY << std::endl;
    std::cout << "MvnPln.->kinect_maxY:" << kinect_maxY << std::endl;
    std::cout << "MvnPln.->kinect_minZ:" << kinect_minZ << std::endl;
    std::cout << "MvnPln.->kinect_maxZ:" << kinect_maxZ << std::endl;


    for(size_t i=0; i<cloudWrtRobot.points.size(); i++)
    {
        pcl::PointXYZRGBA pR = cloudWrtRobot.points[i];
        pcl::PointXYZRGBA pM = cloudWrtMap.points[i];
        idx = (int)((pM.y - mapOriginY)/mapResolution)*mapWidth + (int)((pM.x - mapOriginX)/mapResolution);
        if(pR.x > kinect_minX && pR.x < kinect_maxX && pR.y > kinect_minY && pR.y < kinect_maxY && idx < augmentedMap.data.size() && idx >= 0)
        {
            if(pR.z > kinect_minZ && pR.z < kinect_maxZ)
                if((augmentedMap.data[idx]+=3) > 100)
                    augmentedMap.data[idx] = 100;
            //else
            //augmentedMap.data[idx] = 0;
        }
    }
    return true;
}

void MvnPln::callbackRobotStop(const std_msgs::Empty::ConstPtr& msg)
{
    this->stopReceived = true;
}

bool MvnPln::callbackPlanPath(navig_msgs::PlanPath::Request& req, navig_msgs::PlanPath::Response& resp)
{
    JustinaKnowledge::getKnownLocations(locations);
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
    JustinaKnowledge::getKnownLocations(locations);
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
    this->resetTask = true;

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
    this->resetTask = true;

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

void MvnPln::callbackCollisionPoint(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    this->collisionPointX = msg->point.x;
    this->collisionPointY = msg->point.y;
}

void MvnPln::callbackEnableAvoidanceTypeObstacle(const std_msgs::Bool::ConstPtr& msg)
{
	this->_avoidance_type_obstacle = msg->data;
}
