#include "robot_service_manager/objectrecotasks.h"

ObjectRecoTasks::ObjectRecoTasks(ros::NodeHandle *t_nh, 
        std::string t_detectObjectsSrvName,
        std::string t_recoObjectsSrvName) : 
    m_nh(t_nh),
    m_detectObjectsSrvName(t_detectObjectsSrvName),
    m_recoObjectsSrvName(t_recoObjectsSrvName)
{
    m_connectionInitialized = false;

    initRosConnection(m_nh);
}

void ObjectRecoTasks::initRosConnection(ros::NodeHandle *t_nh)
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

void ObjectRecoTasks::prepareRosConnection()
{
    /*
     * TODO: Print error messages if one of the object could not be 
     * initialized.
     */
    if(!m_detectObjectsSrv)
    {
        if((m_detectObjectsSrv = m_nh->
                    serviceClient<vision_msgs::DetectObjects>(
                        m_detectObjectsSrvName)))
        {
            m_connectionInitialized = true;
        } else {}
    }
}

bool ObjectRecoTasks::detectObjects(
        std::vector<vision_msgs::VisionObject> &t_recognizedObjects)
{
    if(!m_detectObjectsSrv)
    {
        //Try to connect.
        m_detectObjectsSrv = m_nh->serviceClient<vision_msgs::DetectObjects>(
                        m_detectObjectsSrvName);
    }
    if(m_detectObjectsSrv)
    {
        vision_msgs::DetectObjects srv;
        if(!m_detectObjectsSrv.call(srv))
        {
            return false;
        }
        t_recognizedObjects = srv.response.recog_objects;
        return true;
    }
    else
    {
        //TODO: Print error message if the publisher is not valid.
    }

}
