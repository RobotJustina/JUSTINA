#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{
    this->n = new ros::NodeHandle();
    this->pub_SimpleMove_GoalDist = this->n->advertise<std_msgs::Float32>("/navigation/path_planning/simple_move/goal_dist", 1);
    this->pub_Head_GoalPose = this->n->advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    this->pub_La_GoalPose = this->n->advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1);
    this->pub_Ra_GoalPose = this->n->advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 1);
    ros::Subscriber subRobotCurrentPose = this->n->subscribe("/navigation/localization/current_pose", 1, &QtRosNode::callbackRobotCurrentPose, this);
    ros::Subscriber subHeadCurrentPose = this->n->subscribe("/hardware/head/current_pose", 1, &QtRosNode::callbackHeadCurrentPose, this);
    
    ros::Rate loop(10);
    while(ros::ok() && !this->gui_closed)
    {
        //std::cout << "Ros node running..." << std::endl;
        ros::spinOnce();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::call_PathCalculator_WaveFront(float currentX, float currentY, float currentTheta, float goalX, float goalY, float goalTheta)
{
    nav_msgs::GetMap srvGetMap;
    navig_msgs::PathFromMap srvPathFromMap;
    ros::ServiceClient srvCltGetMap = this->n->serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    ros::ServiceClient srvCltPathFromMap = this->n->serviceClient<navig_msgs::PathFromMap>("/navigation/path_planning/path_calculator/wave_front");
    srvCltGetMap.call(srvGetMap);
    ros::spinOnce();
    srvPathFromMap.request.map = srvGetMap.response.map;
    srvPathFromMap.request.start_pose.position.x = currentX;
    srvPathFromMap.request.start_pose.position.y = currentY;
    srvPathFromMap.request.start_pose.orientation.w = cos(currentTheta/2);
    srvPathFromMap.request.start_pose.orientation.z = sin(currentTheta/2);
    srvPathFromMap.request.goal_pose.position.x = goalX;
    srvPathFromMap.request.goal_pose.position.y = goalY;
    srvPathFromMap.request.goal_pose.orientation.w = cos(goalTheta/2);
    srvPathFromMap.request.goal_pose.orientation.z = sin(goalTheta/2);
    if(srvCltPathFromMap.call(srvPathFromMap))
        std::cout << "QtRosNode.->Path calculated succesfully by path_calculator using wavefront" << std::endl;
    else
        std::cout << "QtRosNode.->Cannot calculate path by path_calculator using wavefront" << std::endl;
    ros::spinOnce();
}

void QtRosNode::call_PathCalculator_AStar(float currentX, float currentY, float currentTheta, float goalX, float goalY, float goalTheta)
{
    nav_msgs::GetMap srvGetMap;
    navig_msgs::PathFromMap srvPathFromMap;
    ros::ServiceClient srvCltGetMap = this->n->serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    ros::ServiceClient srvCltPathFromMap = this->n->serviceClient<navig_msgs::PathFromMap>("/navigation/path_planning/path_calculator/a_star");
    srvCltGetMap.call(srvGetMap);
    ros::spinOnce();
    srvPathFromMap.request.map = srvGetMap.response.map;
    srvPathFromMap.request.start_pose.position.x = currentX;
    srvPathFromMap.request.start_pose.position.y = currentY;
    srvPathFromMap.request.start_pose.orientation.w = cos(currentTheta/2);
    srvPathFromMap.request.start_pose.orientation.z = sin(currentTheta/2);
    srvPathFromMap.request.goal_pose.position.x = goalX;
    srvPathFromMap.request.goal_pose.position.y = goalY;
    srvPathFromMap.request.goal_pose.orientation.w = cos(goalTheta/2);
    srvPathFromMap.request.goal_pose.orientation.z = sin(goalTheta/2);
    if(srvCltPathFromMap.call(srvPathFromMap))
        std::cout << "QtRosNode.->Path calculated succesfully by path_calculator using A*" << std::endl;
    else
        std::cout << "QtRosNode.->Cannot calculate path by path_calculator using A*" << std::endl;
    ros::spinOnce();
}

void QtRosNode::publish_SimpleMove_GoalDist(float goalDist)
{
    std_msgs::Float32 msgDist;
    msgDist.data = goalDist;
    this->pub_SimpleMove_GoalDist.publish(msgDist);
    ros::spinOnce();
}

void QtRosNode::publish_Head_GoalPose(float pan, float tilt)
{
    std::cout << "QtRosNode.->Publishing head goal_pose: " << pan << "  " << tilt << std::endl;
    std_msgs::Float32MultiArray msgGoalPose;
    msgGoalPose.data.push_back(pan);
    msgGoalPose.data.push_back(tilt);
    this->pub_Head_GoalPose.publish(msgGoalPose);
}

void QtRosNode::publish_La_GoalPose(std::vector<float> angles)
{
    std::cout << "QtRosNode.->Publishing left arm goal pose: ";
    for(int i=0; i< angles.size(); i++)
        std::cout << angles[i] << " ";
    std::cout << std::endl;
    std_msgs::Float32MultiArray msgLaPose;
    for(int i=0; i < angles.size(); i++)
        msgLaPose.data.push_back(angles[i]);
    this->pub_La_GoalPose.publish(msgLaPose);
}

void QtRosNode::publish_Ra_GoalPose(std::vector<float> angles)
{
    std::cout << "QtRosNode.->Publishing right arm goal pose: ";
    for(int i=0; i< angles.size(); i++)
        std::cout << angles[i] << " ";
    std::cout << std::endl;
    std_msgs::Float32MultiArray msgRaPose;
    for(int i=0; i < angles.size(); i++)
        msgRaPose.data.push_back(angles[i]);
    this->pub_Ra_GoalPose.publish(msgRaPose);
}

void QtRosNode::callbackRobotCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    float currentX = msg->pose.pose.position.x;
    float currentY = msg->pose.pose.position.y;
    float currentTheta = atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) * 2;
    //std::cout << "JustinaGUI.->Current pose: " << currentX << "  " << currentY << "  " << currentTheta << std::endl;
    emit onCurrentRobotPoseReceived(currentX, currentY, currentTheta);
}

void QtRosNode::callbackHeadCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    float pan = msg->data[0];
    float tilt = msg->data[1];
    emit onCurrentHeadPoseReceived(pan, tilt);
}

void QtRosNode::callbackLaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::vector<float> angles;
    for(int i=0; i< msg->data.size(); i++)
        angles.push_back(msg->data[i]);
    emit onCurrentLaPoseReceived(angles);
}

void QtRosNode::callbackRaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::vector<float> angles;
    for(int i=0; i< msg->data.size(); i++)
        angles.push_back(msg->data[i]);
    emit onCurrentRaPoseReceived(angles);
}
