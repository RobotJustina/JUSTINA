#include "robot_service_manager/robotarmtasks.h"

RobotArmTasks::RobotArmTasks(ros::NodeHandle *t_nh,
        std::string t_armGoToAnglesTopicName,
        std::string t_armGotoPoseWRTATopicName,
        std::string t_armGoToPoseWRTRTopicName,
        std::string t_armGoToLocationTopicName,
        std::string t_armMoveTopicName,
        std::string t_armGoalReachedTopicName
        ): 
    m_nh(t_nh),
    m_armGoToAnglesTopicName(t_armGoToAnglesTopicName),
    m_armGotoPoseWRTATopicName(t_armGotoPoseWRTATopicName),
    m_armGoToPoseWRTRTopicName(t_armGoToPoseWRTRTopicName),
    m_armGoToLocationTopicName(t_armGoToLocationTopicName),
    m_armMoveTopicName(t_armMoveTopicName),
    m_armGoalReachedTopicName(t_armGoalReachedTopicName)
{
    m_connectionInitialized = false;
    initRosConnection(m_nh);
}

void RobotArmTasks::initRosConnection(ros::NodeHandle *t_nh)
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

void RobotArmTasks::prepareRosConnection()
{
    /*
     * TODO: Print error messages if one of the object could not be 
     * initialized.
     */
    if(!m_armGoToAnglesPub)
    {
        if((m_armGoToAnglesPub = m_nh->advertise<std_msgs::Float32MultiArray>(
                        m_armGoToAnglesTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_armGotoPoseWRTAPub)
    {
        if((m_armGotoPoseWRTAPub = m_nh->advertise<
                    std_msgs::Float32MultiArray>(m_armGotoPoseWRTATopicName, 
                        100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_armGoToPoseWRTRPub)
    {
        if((m_armGoToPoseWRTRPub = m_nh->advertise<
                    std_msgs::Float32MultiArray>(m_armGoToPoseWRTRTopicName, 
                        100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_armGoToLocationPub)
    {
        if((m_armGoToLocationPub = m_nh->advertise<std_msgs::String>(
                        m_armGoToLocationTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_armMovePub)
    {
        if((m_armMovePub = m_nh->advertise<std_msgs::String>(
                        m_armMoveTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_armGoalReachedSub)
    {
        if((m_armGoalReachedSub = m_nh->subscribe(m_armGoalReachedTopicName, 
                        100, &RobotArmTasks::armGoalReachedCallback, this)))
        {
            m_connectionInitialized = true; 
        } else {}
    }
}

void RobotArmTasks::startArmGoToLocation(std::string t_location)
{
    if(!m_armGoToLocationPub)
    {
        //Try to connect.
        m_armGoToLocationPub = m_nh->advertise<std_msgs::String>(
                        m_armGoToLocationTopicName, 100);
    }
    if(m_armGoToLocationPub)
    {
        m_armGoalReached = false;
        //publish the message
        std_msgs::String locationMsg;
        locationMsg.data = t_location;
        m_armGoToLocationPub.publish(locationMsg);
    }
    else
    {
        //TODO: Print error message if the publisher is not valid.
    }
}

void RobotArmTasks::asyncArmGoTo(std::string t_goalPosition)
{
    startArmGoToLocation(t_goalPosition);
}

bool RobotArmTasks::syncArmGoTo(std::string t_goalLocation, int t_timeout)
{
    using namespace boost;

    startArmGoToLocation(t_goalLocation);

    chrono::milliseconds elapsedTime;
    chrono::steady_clock::time_point startTime = chrono::steady_clock::now();
    while(ros::ok() && !isGoalReached() && elapsedTime.count()<t_timeout)
    {
        elapsedTime = chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now() - startTime
                );
        ros::spinOnce();
    }

    if(elapsedTime.count() >= t_timeout || !ros::ok())
    {
        return false;
    }
    
    return m_armGoalReached;
}

void RobotArmTasks::armGoalReachedCallback(
        const std_msgs::Bool::ConstPtr &t_goalReached)
{
    m_armGoalReached = t_goalReached->data;
}

bool RobotArmTasks::isGoalReached()
{
    return m_armGoalReached;
}
