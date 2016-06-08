#include "robot_service_manager/navigationstatus.h"

NavigationStatus::NavigationStatus(ros::NodeHandle *t_nh, 
        std::string t_goalReachedTopicName, 
        std::string t_moveGoalDistTopicName, 
        std::string t_moveGoalDistAngleTopicName,
        std::string t_moveGoalPathTopicName,
        std::string t_moveGoalPoseTopicName,
        std::string t_moveGoalRelPoseTopicName,
        std::string t_getCloseLocTopicName,
        std::string t_getCloseXYATopicName, 
        std::string t_currentRobotPoseTopicName) :
    m_nh(t_nh), m_goalReachedTopicName(t_goalReachedTopicName), 
    m_moveGoalDistTopicName(t_moveGoalDistTopicName), 
    m_moveGoalDistAngleTopicName(t_moveGoalDistAngleTopicName),
    m_moveGoalPathTopicName(t_moveGoalPathTopicName),
    m_moveGoalPoseTopicName(t_moveGoalPoseTopicName),
    m_moveGoalRelPoseTopicName(t_moveGoalRelPoseTopicName),
    m_getCloseLocTopicName(t_getCloseLocTopicName),
    m_getCloseXYATopicName(t_getCloseXYATopicName), 
    m_currentRobotPoseTopicName(t_currentRobotPoseTopicName)
{
    m_connectionInitialized = false; 
    m_isGoalReached = false;

    //Initialize the class ROS connection
    initRosConnection(m_nh);
}

void NavigationStatus::initRosConnection(ros::NodeHandle *t_nh)
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
        m_tfListener = new tf::TransformListener();
        m_tfListener->waitForTransform("map", "base_link", ros::Time(0), 
                ros::Duration(5.0));
    }
    else
    {
        /*
         * TODO: Print error message if ros is not initialized.
         */
    }
}

void NavigationStatus::prepareRosConnection()
{
    /*
     * TODO: Print error messages if one of the object could not be 
     * initialized.
     */
    if(!m_goalReachedSub)
    {
        if((m_goalReachedSub = m_nh->subscribe(m_goalReachedTopicName, 100, 
                        &NavigationStatus::goalReachedCallback, this)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_currentRobotPoseSub)
    {
        if((m_currentRobotPoseSub= m_nh->subscribe(m_currentRobotPoseTopicName,
                        100, &NavigationStatus::currentRobotPoseCallback, 
                        this)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_moveGoalDistPub)
    {
        if((m_moveGoalDistPub = m_nh->advertise<std_msgs::Float32>(
                        m_moveGoalDistTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_moveGoalDistAnglePub)
    {
        if((m_moveGoalDistAnglePub = m_nh->advertise<
                    std_msgs::Float32MultiArray>(m_moveGoalDistAngleTopicName, 
                        100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_moveGoalPathPub)
    {
        if((m_moveGoalPathPub= m_nh->advertise<nav_msgs::Path>(
                        m_moveGoalPathTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_moveGoalPosePub)
    {
        if((m_moveGoalPosePub = m_nh->advertise<geometry_msgs::Pose2D>(
                        m_moveGoalPoseTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_moveGoalRelPosePub)
    {
        if((m_moveGoalRelPosePub= m_nh->advertise<geometry_msgs::Pose2D>(
                        m_moveGoalRelPoseTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_getCloseLocPub)
    {
        if((m_getCloseLocPub = m_nh->advertise<std_msgs::String>(
                        m_getCloseLocTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_getCloseXYAPub)
    {
        if((m_getCloseXYAPub = m_nh->advertise<std_msgs::Float32MultiArray>(
                        m_getCloseXYATopicName , 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
}

bool NavigationStatus::isGoalReached()
{
    return m_isGoalReached;
}

void NavigationStatus::getCurrentPose(float &t_x, float &t_y, float &t_angle)
{
    tf::StampedTransform transform;
    tf::Quaternion q;
    m_tfListener->lookupTransform("map", "base_link", 
            ros::Time(0), transform);
    m_currentPositionX = transform.getOrigin().x();
    m_currentPositionY = transform.getOrigin().y();
    q = transform.getRotation();
    m_currentPositionAngle = atan2((float)q.z(), (float)q.w()) * 2;

    t_x = m_currentPositionX;
    t_y = m_currentPositionY;
    t_angle = m_currentPositionAngle;
}

void NavigationStatus::goalReachedCallback(const std_msgs::Bool::ConstPtr 
        &t_recMsg)
{
    m_isGoalReached = t_recMsg->data;
}

void NavigationStatus::currentRobotPoseCallback(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr 
        &t_recMsg)
{
    m_currentPositionX = t_recMsg->pose.pose.position.x;
    m_currentPositionY = t_recMsg->pose.pose.position.y;
    m_currentPositionAngle = atan2(t_recMsg->pose.pose.orientation.z,
            t_recMsg->pose.pose.orientation.w) * 2;
}

void NavigationStatus::setGoalDist(float t_distance)
{
    m_isGoalReached = false;
    if(!m_moveGoalDistPub)
    {
        m_moveGoalDistPub = m_nh->advertise<std_msgs::Float32>(
                        m_moveGoalDistTopicName, 100);
    }
    if(m_moveGoalDistPub)
    {
        std_msgs::Float32 floatMsg;
        floatMsg.data = t_distance;
        m_moveGoalDistPub.publish(floatMsg);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}

void NavigationStatus::setGoalDistAngle(float t_distance, float t_angle)
{
    m_isGoalReached = false;
    if(!m_moveGoalDistAnglePub)
    {
        m_moveGoalDistAnglePub = m_nh->advertise<std_msgs::Float32MultiArray>(
                m_moveGoalDistAngleTopicName, 100);
    }
    if(m_moveGoalDistAnglePub)
    {
        std_msgs::Float32MultiArray faMsg;
        faMsg.data.push_back(t_distance);
        faMsg.data.push_back(t_angle);
        m_moveGoalDistAnglePub.publish(faMsg);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}

void NavigationStatus::setGoalPath(nav_msgs::Path &t_path)
{
    m_isGoalReached = false;
    if(!m_moveGoalPathPub)
    {
        m_moveGoalPathPub= m_nh->advertise<nav_msgs::Path>(
                        m_moveGoalPathTopicName, 100);
    }
    if(m_moveGoalPathPub)
    {
        m_moveGoalPathPub.publish(t_path);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}

void NavigationStatus::setGoalPose(float t_x, float t_y, float t_angle)
{
    m_isGoalReached = false;
    if(!m_moveGoalPosePub)
    {
        m_moveGoalPosePub = m_nh->advertise<geometry_msgs::Pose2D>(
                        m_moveGoalPoseTopicName, 100);
    }
    if(m_moveGoalPosePub)
    {
        geometry_msgs::Pose2D poseMsg;
        poseMsg.x = t_x;
        poseMsg.y = t_y;
        poseMsg.theta = t_angle;
        m_moveGoalPosePub.publish(poseMsg);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}

void NavigationStatus::setGoalRelPose(float t_relX, float t_relY, 
        float t_relAngle)
{
    m_isGoalReached = false;
    if(!m_moveGoalRelPosePub)
    {
        m_moveGoalRelPosePub= m_nh->advertise<geometry_msgs::Pose2D>(
                        m_moveGoalRelPoseTopicName, 100);
    }
    if(m_moveGoalRelPosePub)
    {
        geometry_msgs::Pose2D poseMsg;
        poseMsg.x = t_relX;
        poseMsg.y = t_relY;
        poseMsg.theta = t_relAngle;
        m_moveGoalRelPosePub.publish(poseMsg);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}

void NavigationStatus::setGetCloseGoal(float t_x, float t_y)
{
    m_isGoalReached = false;
    if(!m_getCloseXYAPub)
    {
        m_getCloseXYAPub = m_nh->advertise<std_msgs::Float32MultiArray>(
                m_getCloseXYATopicName , 100);
    }
    if(m_getCloseXYAPub)
    {
        std_msgs::Float32MultiArray faMsg;
        faMsg.data.push_back(t_x);
        faMsg.data.push_back(t_y);
        m_getCloseXYAPub.publish(faMsg);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}

void NavigationStatus::setGetCloseGoal(float t_x, float t_y, float t_angle)
{
    m_isGoalReached = false;
    if(!m_getCloseXYAPub)
    {
        m_getCloseXYAPub = m_nh->advertise<std_msgs::Float32MultiArray>(
                m_getCloseXYATopicName , 100);
    }
    if(m_getCloseXYAPub)
    {
        std_msgs::Float32MultiArray faMsg;
        faMsg.data.push_back(t_x);
        faMsg.data.push_back(t_y);
        faMsg.data.push_back(t_angle);
        m_getCloseXYAPub.publish(faMsg);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}

void NavigationStatus::setGetCloseGoal(std::string t_goalName)
{
    m_isGoalReached = false;
    if(!m_getCloseLocPub)
    {
        m_getCloseLocPub = m_nh->advertise<std_msgs::String>(
                        m_getCloseLocTopicName, 100);
    }
    if(m_getCloseLocPub)
    {
        std_msgs::String strMsg;
        strMsg.data = t_goalName;
        m_getCloseLocPub.publish(strMsg);
    }
    else
    {
        /**
         * TODO: Print error message if the publisher is not valid.
         */
    }
}

