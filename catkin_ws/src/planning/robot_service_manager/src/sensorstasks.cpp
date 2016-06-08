#include "robot_service_manager/sensorstasks.h"

SensorsTasks::SensorsTasks(ros::NodeHandle *t_nh, 
        std::string t_srvGetKinnectRGBDName, 
        std::string t_srvGetRobotRGBDName, 
        std::string t_saveCloudTopicName, 
        std::string t_stopSavingCloudTopicName) : 
    m_nh(t_nh),
    m_srvGetKinnectRGBDName(t_srvGetKinnectRGBDName),
    m_srvGetRobotRGBDName(t_srvGetRobotRGBDName),
    m_saveCloudTopicName(t_saveCloudTopicName),
    m_stopSavingCloudTopicName(t_stopSavingCloudTopicName)
{
    m_connectionInitialized = false; 

    //Initialize the class ROS connection
    initRosConnection(m_nh);
}

void SensorsTasks::initRosConnection(ros::NodeHandle *t_nh)
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

void SensorsTasks::prepareRosConnection()
{
    /*
     * TODO: Print error messages if one of the object could not be 
     * initialized.
     */
    if(!m_srvGetKinnectRGBD)
    {
        if((m_srvGetKinnectRGBD = m_nh->serviceClient<
                    point_cloud_manager::GetRgbd>(m_srvGetKinnectRGBDName)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_srvGetRobotRGBD)
    {
        if((m_srvGetRobotRGBD = m_nh->serviceClient<
                    point_cloud_manager::GetRgbd>(m_srvGetRobotRGBDName)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_pubSaveCloudTopic)
    {
        if((m_pubSaveCloudTopic = m_nh->advertise<std_msgs::String>(
                        m_saveCloudTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_pubStopSavingCloud)
    {
        if((m_pubStopSavingCloud = m_nh->advertise<std_msgs::Empty>(
                        m_stopSavingCloudTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
}

bool SensorsTasks::getKinnectRGBD(sensor_msgs::PointCloud2 &t_pointCloud)
{
    if(!m_srvGetKinnectRGBD)
    {
        m_srvGetKinnectRGBD = m_nh->serviceClient<point_cloud_manager::GetRgbd>
            (m_srvGetKinnectRGBDName);
    }
    if(m_srvGetKinnectRGBD)
    {
        point_cloud_manager::GetRgbd srvGetRGBD;
        bool success;
        if((success = m_srvGetKinnectRGBD.call(srvGetRGBD)))
        {
            t_pointCloud = srvGetRGBD.response.point_cloud;
        }
        return success;
    }

    /**
     * TODO: Print error message if the service client is not valid.
     */
    return false;
}

bool SensorsTasks::getRobotRGBD(sensor_msgs::PointCloud2 &t_pointCloud)
{
    if(!m_srvGetRobotRGBD)
    {
        m_srvGetRobotRGBD = m_nh->serviceClient<point_cloud_manager::GetRgbd>
            (m_srvGetRobotRGBDName);
    }
    if(m_srvGetRobotRGBD)
    {
        point_cloud_manager::GetRgbd srvGetRGBD;
        bool success;
        if((success = m_srvGetRobotRGBD.call(srvGetRGBD)))
        {
            t_pointCloud = srvGetRGBD.response.point_cloud;
        }
        return success;
    }
    /**
     * TODO: Print error message if the service client is not valid.
     */
    return false;
}

void SensorsTasks::startSavingCloud(std::string t_fileName)
{
    if(!m_pubSaveCloudTopic)
    {
        m_pubSaveCloudTopic = m_nh->advertise<std_msgs::String>(
                m_saveCloudTopicName, 100);
    }
    if(m_pubSaveCloudTopic)
    {
        std_msgs::String msgString;
        msgString.data = t_fileName;
        m_pubSaveCloudTopic.publish(msgString);
    }
    else
    {
        /**
         * TODO: Print error message if the service client is not valid.
         */
    }
}

void SensorsTasks::stopSavingCloud()
{
    if(!m_pubStopSavingCloud)
    {
        m_pubStopSavingCloud = m_nh->advertise<std_msgs::Empty>(
                m_stopSavingCloudTopicName, 100);
    }
    if(m_pubStopSavingCloud)
    {
        std_msgs::Empty msgEmpty;
        m_pubStopSavingCloud.publish(msgEmpty);
    }
    else
    {
        /**
         * TODO: Print error message if the service client is not valid.
         */
    }
}
