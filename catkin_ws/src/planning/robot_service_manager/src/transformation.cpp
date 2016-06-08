#include "robot_service_manager/transformations.h"

Transformations::Transformations(ros::NodeHandle *t_nh, 
        std::string t_IKFloatArrayServiceName, std::string t_IKPathServiceName, 
        std::string t_IKPoseServiceName, std::string t_DKServiceName) : 
    m_nh(t_nh), 
    m_IKFloatArrayServiceName(t_IKFloatArrayServiceName),
    m_IKPathServiceName(t_IKPathServiceName),
    m_IKPoseServiceName(t_IKPoseServiceName),
    m_DKServiceName(t_DKServiceName)
{
    m_connectionInitialized = false; 

    //Initialize the class ROS connection
    initRosConnection(m_nh);
}

void Transformations::initRosConnection(ros::NodeHandle *t_nh)
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

void Transformations::prepareRosConnection()
{
    /*
     * TODO: Print error messages if one of the object could not be 
     * initialized.
     */
    if(!m_srvIKFloatArray)
    {
        if((m_srvIKFloatArray= m_nh->serviceClient<
                    manip_msgs::InverseKinematicsFloatArray>(
                        m_IKFloatArrayServiceName)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_srvIKPath)
    {
        if((m_srvIKPath = m_nh->serviceClient<
                    manip_msgs::InverseKinematicsPath>(m_IKPathServiceName)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_srvIKPose)
    {
        if((m_srvIKPose = m_nh->serviceClient<
                    manip_msgs::InverseKinematicsPose>(m_IKPoseServiceName)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_srvDK)
    {
        if((m_srvDK = m_nh->serviceClient<
                    manip_msgs::DirectKinematics>(m_DKServiceName)))
        {
            m_connectionInitialized = true;
        } else {}
    }
}
bool Transformations::inverseKinematics(std::vector<float> &t_cartesian, 
        std::vector<float> &t_articular)
{
    if(!m_srvIKFloatArray)
    {
        m_srvIKFloatArray = m_nh->serviceClient<
            manip_msgs::InverseKinematicsFloatArray>(
                    m_IKFloatArrayServiceName);
    }
    if(m_srvIKFloatArray)
    {
        bool success;
        manip_msgs::InverseKinematicsFloatArray IKFAsrv;
        IKFAsrv.request.cartesian_pose.data = t_cartesian;
        success = m_srvIKFloatArray.call(IKFAsrv);
        t_articular = IKFAsrv.response.articular_pose.data;

        return success;
    }

    /**
     * TODO: Print error message if the service client is not valid.
     */
    return false;
}

bool Transformations::directKinematics(std::vector<float> &t_cartesian, 
        std::vector<float> &t_articular)
{
    if(!m_srvDK)
    {
        m_srvDK = m_nh->serviceClient<manip_msgs::DirectKinematics>(
                m_DKServiceName);
    }
    if(m_srvDK)
    {
        bool success;
        manip_msgs::DirectKinematics DKsrv;
        DKsrv.request.articular_pose.data = t_articular;
        success = m_srvDK.call(DKsrv);
        t_cartesian = DKsrv.response.cartesian_pose.data;

        return success;
    }

    /**
     * TODO: Print error message if the service client is not valid.
     */
    return false;
}
