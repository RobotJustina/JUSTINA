#include "robot_service_manager/navigationtasks.h"

NavigationTasks::NavigationTasks(ros::NodeHandle *t_nh,
        std::string t_getOccupancyGridSrvName,
        std::string t_calcAStarPathFromMapSrvName,
        std::string t_calcWaveFrontPathFromMapSrvName,
        std::string t_planPathSrvName
        ) :
    m_nh(t_nh),
    m_getOccupancyGridSrvName(t_getOccupancyGridSrvName),
    m_calcAStarPathFromMapSrvName(t_calcAStarPathFromMapSrvName),
    m_calcWaveFrontPathFromMapSrvName(t_calcWaveFrontPathFromMapSrvName),
    m_planPathSrvName(t_planPathSrvName)
{
    m_connectionInitialized = false; 
    initRosConnection(m_nh);
}

void NavigationTasks::initRosConnection(ros::NodeHandle *t_nh)
{
    if(m_connectionInitialized)
    {
        /**
         * TODO: Print warning saying that the class is currently initialized.
         */
        return;
    }

    m_nh = t_nh;
    /**
     * Subscribe to the arm ros topics if the communication with 
     * ROS is initialized.
     */
    if(ros::isInitialized())
    {
        /**
         * If no node handler provided, then create a new one.
         */
        if(m_nh == 0)
        {
            m_nh = new ros::NodeHandle;
        }
        m_navStatus.initRosConnection(t_nh);
        m_sensorsTasks.initRosConnection(t_nh);
        /*
         * Initialize subscribers, publishers and verification.
         */
        prepareRosConnection();
    }
    else
    {
        /*
         * TODO: Print error message if ros is not initialized.
         */
    }
}

void NavigationTasks::prepareRosConnection()
{
    /*
     * TODO: Print error messages if one of the object could not be 
     * initialized.
     */
    if(!m_getOccupancyGridSrv)
    {
        if((m_getOccupancyGridSrv =  m_nh->serviceClient<nav_msgs::GetMap>(
                        m_getOccupancyGridSrvName)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_calcAStarPathFromMapSrv)
    {
        if((m_calcAStarPathFromMapSrv=m_nh->serviceClient<
                    navig_msgs::PathFromMap>(m_calcAStarPathFromMapSrvName)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_calcWaveFrontPathFromMapSrv)
    {
        if((m_calcWaveFrontPathFromMapSrv=m_nh->serviceClient<
                    navig_msgs::PathFromMap>(
                        m_calcWaveFrontPathFromMapSrvName)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_planPathSrv)
    {
        if((m_planPathSrv=m_nh->serviceClient<navig_msgs::PlanPath>(
                        m_planPathSrvName)))
        {
            m_connectionInitialized = true;
        } else {}
    }
}

bool NavigationTasks::waitForGoalReached(int t_timeout)
{
    using namespace boost;

    chrono::milliseconds elapsedTime;
    chrono::steady_clock::time_point startTime = chrono::steady_clock::now();
    while(ros::ok() && !m_navStatus.isGoalReached() && 
            elapsedTime.count()<t_timeout)
    {
        elapsedTime = chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now() - startTime
                );
        ros::spinOnce();
    }

    return m_navStatus.isGoalReached();
}

bool NavigationTasks::getOccupancyGrid(nav_msgs::OccupancyGrid &t_map)
{
    if(!m_getOccupancyGridSrv)
    {
        m_getOccupancyGridSrv =  m_nh->serviceClient<nav_msgs::GetMap>(
                m_getOccupancyGridSrvName);
    }
    if(m_getOccupancyGridSrv)
    {
        nav_msgs::GetMap srvGetMap;
        bool success;
        if((success=m_getOccupancyGridSrv.call(srvGetMap)))
        {
            t_map = srvGetMap.response.map;
        }
        return success;
    }
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    return false;
}

bool NavigationTasks::calcAStarPathFromMap(float t_goalX, float t_goalY, 
        nav_msgs::Path &t_path)
{
    //get the current robot pose
    float robotX, robotY, robotAngle;
    m_navStatus.getCurrentPose(robotX, robotY, robotAngle);
    //call to the overriden function
    return calcAStarPathFromMap(robotX, robotY, t_goalX, t_goalY, t_path);
}


bool NavigationTasks::calcAStarPathFromMap(float t_startX, float t_startY, 
        float t_goalX, float t_goalY, nav_msgs::Path &t_path)
{
    if(!m_calcAStarPathFromMapSrv)
    {
        m_calcAStarPathFromMapSrv=m_nh->serviceClient<navig_msgs::PathFromMap>(
                m_calcAStarPathFromMapSrvName);
    }
    if(m_calcAStarPathFromMapSrv)
    {
        nav_msgs::GetMap srvGetMap;
        navig_msgs::PathFromMap srvPathFromMap;

        bool success;
        if((success=m_getOccupancyGridSrv.call(srvGetMap)))
        {
            srvPathFromMap.request.map = srvGetMap.response.map;
            srvPathFromMap.request.start_pose.position.x = t_startX;
            srvPathFromMap.request.start_pose.position.y = t_startY;
            srvPathFromMap.request.goal_pose.position.x = t_goalX;
            srvPathFromMap.request.goal_pose.position.y = t_goalY;
    
            success = m_calcAStarPathFromMapSrv.call(
                    srvPathFromMap);
            ros::spinOnce();
            t_path = srvPathFromMap.response.path;
        }
        return success;
    }
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    return false;
}

bool NavigationTasks::calcWaveFrontPathFromMap(float t_goalX, float t_goalY, 
        nav_msgs::Path &t_path)
{
    //get the current robot pose
    float robotX, robotY, robotAngle;
    m_navStatus.getCurrentPose(robotX, robotY, robotAngle);
    //call to the overriden function
    return calcWaveFrontPathFromMap(robotX, robotY, t_goalX, t_goalY, t_path);
}

bool NavigationTasks::calcWaveFrontPathFromMap(float t_startX, float t_startY, 
        float t_goalX, float t_goalY, nav_msgs::Path &t_path)
{
    if(!m_calcWaveFrontPathFromMapSrv)
    {
        m_calcWaveFrontPathFromMapSrv=m_nh->serviceClient<
                    navig_msgs::PathFromMap>(m_calcWaveFrontPathFromMapSrvName);
    }
    if(m_calcWaveFrontPathFromMapSrv)
    {
        nav_msgs::GetMap srvGetMap;
        navig_msgs::PathFromMap srvPathFromMap;

        bool success;
        if((success=m_getOccupancyGridSrv.call(srvGetMap)))
        {
            srvPathFromMap.request.map = srvGetMap.response.map;
            srvPathFromMap.request.start_pose.position.x = t_startX;
            srvPathFromMap.request.start_pose.position.y = t_startY;
            srvPathFromMap.request.goal_pose.position.x = t_goalX;
            srvPathFromMap.request.goal_pose.position.y = t_goalY;

            success = m_calcWaveFrontPathFromMapSrv.call(srvPathFromMap);
            ros::spinOnce();
            t_path = srvPathFromMap.response.path;
        }
        return success;
    }
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    return false;
}

bool NavigationTasks::planPath(float t_startX, float t_startY, float t_goalX, 
        float t_goalY, nav_msgs::Path &t_path)
{
    if(!m_planPathSrv)
    {
        m_planPathSrv=m_nh->serviceClient<navig_msgs::PlanPath>(
                        m_planPathSrvName);
    }
    if(m_planPathSrv)
    {
        navig_msgs::PlanPath srv;
        srv.request.start_location_id = "";
        srv.request.goal_location_id = "";
        srv.request.start_pose.position.x = t_startX;
        srv.request.start_pose.position.y = t_startY;
        srv.request.goal_pose.position.x = t_goalX;
        srv.request.goal_pose.position.y = t_goalY;
        bool success = m_planPathSrv.call(srv);
        t_path = srv.response.path;
        return success; 
    }
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    return false;
}

bool NavigationTasks::planPath(float t_goalX, float t_goalY, 
        nav_msgs::Path &t_path)
{
    float robotX, robotY, robotTheta;
    m_navStatus.getCurrentPose(robotX, robotY, robotTheta);
    return planPath(robotX, robotY, t_goalX, t_goalY, 
            t_path);
}


bool NavigationTasks::planPath(std::string t_startLocation, 
        std::string t_goalLocation, nav_msgs::Path &t_path)
{
    if(!m_planPathSrv)
    {
        m_planPathSrv=m_nh->serviceClient<navig_msgs::PlanPath>(
                        m_planPathSrvName);
    }
    if(m_planPathSrv)
    {
        navig_msgs::PlanPath srv;
        srv.request.start_location_id = t_startLocation;
        srv.request.goal_location_id = t_goalLocation;
        bool success = m_planPathSrv.call(srv);
        t_path = srv.response.path;
        return success;
    }
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    return false;
}

bool NavigationTasks::planPath(std::string t_goalLocation, 
        nav_msgs::Path &t_path)
{
    if(!m_planPathSrv)
    {
        m_planPathSrv=m_nh->serviceClient<navig_msgs::PlanPath>(
                        m_planPathSrvName);
    }
    if(m_planPathSrv)
    {
        float robotX, robotY, robotTheta;
        m_navStatus.getCurrentPose(robotX, robotY, robotTheta);
        navig_msgs::PlanPath srv;
        srv.request.start_location_id = "";
        srv.request.goal_location_id = t_goalLocation;
        srv.request.start_pose.position.x = robotX;
        srv.request.start_pose.position.y = robotY;
        bool success = m_planPathSrv.call(srv);
        t_path = srv.response.path;
        return success;
    }
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    return false;
}

bool NavigationTasks::planPath(std::string t_startLocation, float t_goalX, 
        float t_goalY, nav_msgs::Path &t_path)
{
    if(!m_planPathSrv)
    {
        m_planPathSrv=m_nh->serviceClient<navig_msgs::PlanPath>(
                        m_planPathSrvName);
    }
    if(m_planPathSrv)
    {
        navig_msgs::PlanPath srv;
        srv.request.start_location_id = t_startLocation;
        srv.request.goal_location_id = "";
        srv.request.goal_pose.position.x = t_goalX;
        srv.request.goal_pose.position.y = t_goalY;
        bool success = m_planPathSrv.call(srv);
        t_path = srv.response.path;
        return success;
    }
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    return false;
}

bool NavigationTasks::planPath(float t_startX, float t_startY, 
        std::string t_goalLocation, nav_msgs::Path &t_path)
{
    if(!m_planPathSrv)
    {
        m_planPathSrv=m_nh->serviceClient<navig_msgs::PlanPath>(
                        m_planPathSrvName);
    }
    if(m_planPathSrv)
    {
        navig_msgs::PlanPath srv;
        srv.request.start_location_id = "";
        srv.request.goal_location_id = t_goalLocation;
        srv.request.start_pose.position.x = t_startX;
        srv.request.start_pose.position.y = t_startY;
        bool success = m_planPathSrv.call(srv);
        t_path = srv.response.path;
        return success;
    }
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    return false;
}

bool NavigationTasks::syncMove(float t_distance, int t_timeout)
{
    m_navStatus.setGoalDist(t_distance);
    return waitForGoalReached(t_timeout);
}

bool NavigationTasks::syncMove(float t_distance, float t_angle, int t_timeout)
{
    m_navStatus.setGoalDistAngle(t_distance, t_angle);
    return waitForGoalReached(t_timeout);
}

bool NavigationTasks::syncMove(nav_msgs::Path &t_path, int t_timeout)
{
    m_navStatus.setGoalPath(t_path);
    return waitForGoalReached(t_timeout);
}

bool NavigationTasks::syncGoToPose(float t_x, float t_y, float t_angle, 
        int t_timeout)
{
    m_navStatus.setGoalPose(t_x, t_y, t_angle);
    return waitForGoalReached(t_timeout);
}

bool NavigationTasks::syncGoToRelPose(float t_relX, float t_relY, 
        float t_relAngle, int t_timeout)
{
    m_navStatus.setGoalRelPose(t_relX, t_relY, t_relAngle);
    return waitForGoalReached(t_timeout);
}

bool NavigationTasks::syncGetClose(float t_x, float t_y, int t_timeout)
{
    m_navStatus.setGetCloseGoal(t_x, t_y);
    return waitForGoalReached(t_timeout);
}

bool NavigationTasks::syncGetClose(float t_x, float t_y, float t_angle, 
        int t_timeout)
{
    m_navStatus.setGetCloseGoal(t_x, t_y, t_angle);
    return waitForGoalReached(t_timeout);
}

bool NavigationTasks::syncGetClose(std::string t_location, int t_timeout)
{
    m_navStatus.setGetCloseGoal(t_location);
    return waitForGoalReached(t_timeout);
}

