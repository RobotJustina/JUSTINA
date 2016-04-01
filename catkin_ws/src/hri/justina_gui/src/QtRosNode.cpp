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
    ros::Subscriber subCurrentPose = this->n->subscribe("/navigation/localization/current_pose", 1, &QtRosNode::callbackCurrentPose, this);
    
    ros::Rate loop(10);
    while(ros::ok() && !this->gui_closed)
    {
        //std::cout << "Ros node running..." << std::endl;
        ros::spinOnce();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::publish_SimpleMove_GoalDist(float goalDist)
{
    std_msgs::Float32 msgDist;
    msgDist.data = goalDist;
    this->pub_SimpleMove_GoalDist.publish(msgDist);
    ros::spinOnce();
}

void QtRosNode::callbackCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    float currentX = msg->pose.pose.position.x;
    float currentY = msg->pose.pose.position.y;
    float currentTheta = atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) * 2;
    //std::cout << "JustinaGUI.->Current pose: " << currentX << "  " << currentY << "  " << currentTheta << std::endl;
    emit onCurrentPoseReceived(currentX, currentY, currentTheta);
}
