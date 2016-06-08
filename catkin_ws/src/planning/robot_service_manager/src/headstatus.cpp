#include "robot_service_manager/headstatus.h"

HeadStatus::HeadStatus(ros::NodeHandle *nh, std::string headCurrentPoseTopic,
        std::string headGoalPoseTopic, std::string headBatteryTopic):
    m_nh(nh),
    m_headCurrentPoseTopic(headCurrentPoseTopic),
    m_headGoalPoseTopic(headGoalPoseTopic),
    m_headBatteryTopic(headBatteryTopic)
{
    /**
     * Subscribe to the head pose ros topic if the communication with 
     * ROS is initialized.
     */
    m_isInitialized = false; 
    initRosConnection(m_nh);
}

void HeadStatus::initRosConnection(ros::NodeHandle *nh)
{
    if(m_isInitialized)
    {
        /**
         * TODO: Print warning saying that the class is currently initialized.
         */
        return;
    }

    m_nh = nh;
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

void HeadStatus::prepareRosConnection()
{
    /*
     * TODO: Print error messages if one of the object could not be 
     * initialized.
     */
    if(!m_subHeadCurrentPose)
    {
        if((m_subHeadCurrentPose = m_nh->subscribe(m_headCurrentPoseTopic, 100,
                        &HeadStatus::headPoseCallback, this)))
        {
            m_isInitialized = true; 
        } else {}
    }
    if(!m_headPosePublisher) 
    {
        if((m_headPosePublisher = m_nh->advertise<std_msgs::Float32MultiArray>(
                        m_headGoalPoseTopic, 100)))
        {
            m_isInitialized = true; 
        } else {}
    }
    if(!m_subHeadCurrentBattery)
    {
        if((m_subHeadCurrentBattery = m_nh->subscribe(m_headBatteryTopic, 100, 
                        &HeadStatus::headBatteryCallback, this)))
        {
            m_isInitialized = true; 
        } else {}
    }
}

void HeadStatus::headBatteryCallback(const std_msgs::Float32::ConstPtr
        &batteryMsg)
{
    m_headBattery = batteryMsg->data;
}

void HeadStatus::headPoseCallback(const std_msgs::Float32MultiArray::ConstPtr
        &poseMsg)
{
    m_headPan = poseMsg->data[0];
    m_headTilt = poseMsg->data[1];
}

float HeadStatus::getHeadBattery()
{
    return m_headBattery;
}

float HeadStatus::getHeadPan()
{
    return m_headPan;
}

float HeadStatus::getHeadTilt()
{
    return m_headTilt;
}

void HeadStatus::getHeadPose(float &headPan, float &headTilt)
{
    headPan = m_headPan;
    headTilt = m_headTilt;
}

void HeadStatus::setHeadPose(float headPan, float headTilt)
{
    if(!m_headPosePublisher)
    {
        /**
         * Try to connect.
         */
        m_headPosePublisher = m_nh->advertise<std_msgs::Float32MultiArray>(
                        m_headGoalPoseTopic, 100);
    }
    if(m_headPosePublisher)
    {
        std_msgs::Float32MultiArray headPoseMsg;
        headPoseMsg.data.push_back(headPan);
        headPoseMsg.data.push_back(headTilt);
        m_headPosePublisher.publish(headPoseMsg);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}
