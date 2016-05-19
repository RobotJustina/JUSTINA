#include "MvnPln.h"

MvnPln::MvnPln()
{
    this->newTask = false;
    this->correctFinalAngle = false;
    this->collisionDetected = false;
    this->stopReceived = false;
}

MvnPln::~MvnPln()
{
}

void MvnPln::initROSConnection(ros::NodeHandle* nh)
{
    this->nh = nh;
    //Publishers and subscribers for the commands executed by this node
    this->subGetCloseLoc = nh->subscribe("/navigation/mvn_pln/get_close_loc", 1, &MvnPln::callbackGetCloseLoc, this);
    this->subGetCloseXYA = nh->subscribe("/navigation/mvn_pln/get_close_xya", 1, &MvnPln::callbackGetCloseXYA, this);
    this->subClickedPoint = nh->subscribe("/clicked_point", 1, &MvnPln::callbackClickedPoint, this);
    this->pubGoalReached = nh->advertise<std_msgs::Bool>("/navigation/goal_reached", 1);
    this->pubLocationMarkers = nh->advertise<visualization_msgs::Marker>("/hri/rviz/location_markers", 1);
    this->pubLastPath = nh->advertise<nav_msgs::Path>("/navigation/mvn_pln/last_calc_path", 1);
    this->srvPlanPath = nh->advertiseService("/navigation/mvn_pln/plan_path", &MvnPln::callbackPlanPath, this);
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
    ros::Rate loop(5);
    int currentState = SM_INIT;
    float robotX, robotY, robotTheta;
    float angleError;

    while(ros::ok())
    {
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
            std::cout << "MvnPln.->Current state: " << currentState << ". Calculating path" << std::endl;
            JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
            if(!this->planPath(robotX, robotY, this->goalX, this->goalY, this->lastCalcPath))
            {
                std::cout << "MvnPln.->Cannot calculate path to " << this->goalX << " " << this->goalY << std::endl;
                currentState = SM_INIT;
            }
            else
                currentState = SM_START_MOVE_PATH;
            break;
        case SM_START_MOVE_PATH:
            std::cout << "MvnPln.->Current state: " << currentState << ". Starting move path" << std::endl;
            std::cout << "MvnPln.->Turning on collision detection..." << std::endl;
            JustinaNavigation::startMovePath(this->lastCalcPath);
            //JustinaVision::startCollisionDetection();
            currentState = SM_WAIT_FOR_MOVE_FINISHED;
            break;
        case SM_WAIT_FOR_MOVE_FINISHED:
            if(JustinaNavigation::isGoalReached())
            {
                std::cout << "MvnPln.->Move path finished succesfully. " << std::endl;
                if(this->correctFinalAngle)
                    currentState = SM_CORRECT_FINAL_ANGLE;
                else
                {
                    std::cout << "MnvPln.->Goal point reached successfully!!!!!!!" << std::endl;
                    currentState = SM_INIT;
                }
            }
            else if(this->collisionDetected)
            {
                std::cout << "MvnPln.->Collision detected before goal is reached." << std::endl;
                currentState = SM_COLLISION_DETECTED;
            }
            else if(this->stopReceived)
            {
                std::cout << "MvnPln.->Stop signal received..." << std::endl;
                currentState = SM_INIT;
            }
            break;
        case SM_COLLISION_DETECTED:
            std::cout << "MvnPln.->Current state: " << currentState << ". Stopping robot smoothly" << std::endl;
            std::cout << "MvnPln.->Current state: " << currentState << ". Some day I'll do something intelligent when collision is detected. " << std::endl;
            currentState = SM_CALCULATE_PATH;
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
                currentState = SM_INIT;
            }
            break;
        }
        
        this->pubLocationMarkers.publish(this->getLocationMarkers());
        this->pubLastPath.publish(this->lastCalcPath);
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
    std::cout << "MvnPln.->Getting occupancy grid from map server... " << std::endl;
    std::cout << "MvnPln.->Moving head to get point cloud at left" << std::endl;
    std::cout << "MvnPln.->Moving head to get point cloud at center" << std::endl;
    std::cout << "MvnPln.->Moving head to get point cloud at right" << std::endl;
    std::cout << "MvnPln.->Merging point clouds with occupancy grid" << std::endl;
    std::cout << "MvnPln.->Calculating path with augmented map..." << std::endl;
    //TODO: Call manually (without using JustinaNavigation.h) the path_calculator in order to use
    //The augmented occupancy grid
    bool success = JustinaNavigation::calcPathFromMapAStar(startX, startY, goalX, goalY, path);
    this->lastCalcPath = path;
    return success;
}

bool MvnPln::callbackPlanPath(navig_msgs::PlanPath::Request& req, navig_msgs::PlanPath::Response& resp)
{
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
    std::cout << std::endl;
}
