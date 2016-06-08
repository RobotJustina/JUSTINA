#include "robot_service_manager/robotstatus.h"

RobotStatus::RobotStatus(ros::NodeHandle *nh, std::string robotStopTopic) :
    m_nh(nh),
    m_robotStopTopic(robotStopTopic)
{
    /**
     * Subscribe to the arm ros topics if the communication with 
     * ROS is initialized.
     */
    m_isInitialized = false; 
    initRosConnection(m_nh);
}

void RobotStatus::initRosConnection(ros::NodeHandle *nh)
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
        
void RobotStatus::prepareRosConnection()
{
    /*
     * TODO: Print error messages if one of the object could not be 
     * initialized.
     */
    if(!m_pubRobotStopTopic)
    {
        if((m_pubRobotStopTopic = m_nh->advertise<std_msgs::Empty>(
                        m_robotStopTopic, 100)))
        {
            m_isInitialized = true; 
        } else {}
    }
}

void RobotStatus::sendStopHardwareIndication()
{
    if(!m_pubRobotStopTopic)
    {
        /**
         * Try to connect.
         */
        m_pubRobotStopTopic = m_nh->advertise<std_msgs::Empty>(
                        m_robotStopTopic, 100);
    }
    if(m_pubRobotStopTopic)
    {
        std_msgs::Empty emptyMsg;
        m_pubRobotStopTopic.publish(emptyMsg);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}
