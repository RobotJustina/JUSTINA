#include "robot_service_manager/mobilebasestatus.h"

MobileBaseStatus::MobileBaseStatus(ros::NodeHandle *nh, int mbMotors,
        std::string mbSpeedsTopic, std::string mbCmdVelTopic, 
        std::string mbBatteryTopic) : 
    m_nh(nh), 
    m_mbMotors(mbMotors), 
    m_mbSpeedsTopic(mbSpeedsTopic), 
    m_mbCmdVelTopic(mbCmdVelTopic),
    m_mbBatteryTopic(mbBatteryTopic)
{
    m_isInitialized = false; 
    /**
     * Initialize ROS connection.
     */
    initRosConnection(m_nh);
}

void MobileBaseStatus::initRosConnection(ros::NodeHandle *nh)
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

void MobileBaseStatus::prepareRosConnection()
{
    /*
     * TODO: Print error messages if one of the object could not be 
     * initialized.
     */
    if(!m_pubMBSpeeds)
    {
        if((m_pubMBSpeeds= m_nh->advertise<std_msgs::Float32MultiArray>(
                m_mbSpeedsTopic, 100)))
        {
            m_isInitialized = true; 
        } else {}
    }
    if(!m_pubMBCmdVel)
    {
        if((m_pubMBCmdVel = m_nh->advertise<geometry_msgs::Twist>(
                m_mbCmdVelTopic, 100)))
        {
            m_isInitialized = true; 
        } else {}
    }
    if(!m_subMBBattery)
    {
        if((m_subMBBattery = m_nh->subscribe(m_mbBatteryTopic, 100, 
                        &MobileBaseStatus::mbBatteryCallback, this)))
        {
            m_isInitialized = true;
        } else {}
    }
}

void MobileBaseStatus::mbBatteryCallback(
        const std_msgs::Float32::ConstPtr& batteryMsg)
{
    m_mbBattery = batteryMsg->data;
}

void MobileBaseStatus::setMobileBaseSpeeds(std::vector<float> &speedsVector)
{
    if(!m_pubMBSpeeds)
    {
        /**
         * Try to connect.
         */
        m_pubMBSpeeds = m_nh->advertise<std_msgs::Float32MultiArray>(
                m_mbSpeedsTopic, 100);
    }
    if(m_pubMBSpeeds)
    {
        /**
         * Try to publish.
         */
        std_msgs::Float32MultiArray speedsMsg;
        for(int i=0; i<m_mbMotors; i++)
        {
            speedsMsg.data.push_back(speedsVector[i]);
        }
        m_pubMBSpeeds.publish(speedsMsg);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}

void MobileBaseStatus::setMobileBaseCmdVel(float linearX, float linearY, 
        float angular)
{
    if(!m_pubMBCmdVel)
    {
        /**
         * Try to connect.
         */
        m_pubMBCmdVel = m_nh->advertise<geometry_msgs::Twist>(
                m_mbCmdVelTopic, 100);
    }
    if(m_pubMBCmdVel)
    {
        geometry_msgs::Twist cvMsg;
        cvMsg.linear.x = linearX;
        cvMsg.linear.y = linearY;
        cvMsg.linear.z = 0;
        cvMsg.angular.x = 0;
        cvMsg.angular.y = 0;
        cvMsg.angular.z = angular;
        m_pubMBCmdVel.publish(cvMsg);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}
float MobileBaseStatus::getMobileBaseBattery()
{
    return m_mbBattery;
}
