#include "SimpleMoveNode.h"

SimpleMoveNode::SimpleMoveNode()
{
    this->newGoal = true;
    this->moveBackwards = false;
}

SimpleMoveNode::~SimpleMoveNode()
{
}

void SimpleMoveNode::initROSConnection()
{
    this->pubGoalReached = this->nh.advertise<std_msgs::Bool>("simple_move/goal_reached", 1);
    this->pubSpeeds = this->nh.advertise<std_msgs::Float32MultiArray>("/hardware/mobile_base/speeds", 1);
    this->subGoalPose = this->nh.subscribe("simple_move/goal_pose", 1, &SimpleMoveNode::callbackGoalPose, this);
    this->subGoalDistance = this->nh.subscribe("simple_move/goal_dist", 1, &SimpleMoveNode::callbackGoalDist, this);
    this->subGoalRelativePose = this->nh.subscribe("simple_move/goal_rel_pose", 1, &SimpleMoveNode::callbackGoalRelPose, this);
    this->subCurrentPose = this->nh.subscribe("/navigation/localization/current_pose", 1, &SimpleMoveNode::callbackCurrentPose, this);
}

void SimpleMoveNode::spin()
{
    ros::Rate loop(10);
    std_msgs::Float32MultiArray speeds;
    std_msgs::Bool goalReached;
    //First element is the leftSpeed, and the second one is the rightSpeed
    speeds.data.push_back(0);
    speeds.data.push_back(0);
    while(ros::ok())
    {
        if(this->newGoal)
        {
            ros::spinOnce(); //Just to have the most recent position
            control.CalculateSpeeds(currentX, currentY, currentTheta, goalX, goalY, speeds.data[0], speeds.data[1], moveBackwards);
            std::cout << "SimpleMove.->Speeds: " << speeds.data[0] << "  " << speeds.data[0] << std::endl;
            pubSpeeds.publish(speeds);
            float errorX = goalX - currentX;
            float errorY = goalY - currentY;
            float error = sqrt(errorX*errorX + errorY*errorY);
            if(error < 0.05)
            {
                goalReached.data = true;
                pubGoalReached.publish(goalReached);
                speeds.data[0] = 0;
                speeds.data[1] = 0;
                pubSpeeds.publish(speeds);
                this->newGoal = false;
            }
        }
        ros::spinOnce();
        loop.sleep();
    }
}

void SimpleMoveNode::callbackCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    this->currentX = msg->pose.pose.position.x;
    this->currentY = msg->pose.pose.position.y;
    this->currentTheta = atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) * 2;
    //std::cout << "SimpleMove.->Current pose: " << this->currentX << "  " << this->currentY << "  " << this->currentTheta << std::endl;
}

void SimpleMoveNode::callbackGoalPose(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    //Moves to the absolute position given by msg
    this->goalX = msg->x;
    this->goalY = msg->y;
    this->goalTheta = msg->theta;
    this->newGoal = true;
    this->moveBackwards = false;
    std::cout << "SimpleMove.->Received new goal pose: " << msg->x << "  " << msg->y << "  " << msg->theta << std::endl;
}

void SimpleMoveNode::callbackGoalDist(const std_msgs::Float32::ConstPtr& msg)
{
    //Moves the distance 'msg' in the current direction. If dist < 0, robot moves backwards
    this->goalX = this->currentX + msg->data*cos(this->currentTheta);
    this->goalY = this->currentY + msg->data*sin(this->currentTheta);
    this->goalTheta = atan2(goalY - currentY, goalX - currentX);
    this->newGoal = true;
    this->moveBackwards = msg->data < 0;
    std::cout << "SimpleMove.->Received new goal distance: " << msg->data << std::endl;
}

void SimpleMoveNode::callbackGoalRelPose(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    //Moves the relative position given by 
    this->goalX = this->currentX + msg->x*cos(this->currentTheta) - msg->y*sin(this->currentTheta);
    this->goalY = this->currentY + msg->x*sin(this->currentTheta) + msg->y*cos(this->currentTheta);
    this->goalTheta = this->currentTheta + msg->theta;
    this->newGoal = true;
    this->moveBackwards = false;
    std::cout << "SimpleMove.->Received new relative goal pose: " << msg->x << "  " << msg->y << "  " << msg->theta << std::endl;
}
