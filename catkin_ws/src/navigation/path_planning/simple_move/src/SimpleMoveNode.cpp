#include "SimpleMoveNode.h"

SimpleMoveNode::SimpleMoveNode()
{
    this->newGoal = false;
    this->newPath = false;
    this->moveBackwards = false;
    this->currentPathPose = 0;
}

SimpleMoveNode::~SimpleMoveNode()
{
}

void SimpleMoveNode::initROSConnection()
{
    this->pubGoalReached = this->nh.advertise<std_msgs::Bool>("/navigation/goal_reached", 1);
    this->pubSpeeds = this->nh.advertise<std_msgs::Float32MultiArray>("/hardware/mobile_base/speeds", 1);
    this->pubHeadGoalPose = this->nh.advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    this->subRobotStop = this->nh.subscribe("/hardware/robot_state/stop", 1, &SimpleMoveNode::callbackRobotStop, this);
    this->subGoalDistance = this->nh.subscribe("simple_move/goal_dist", 1, &SimpleMoveNode::callbackGoalDist, this);
    this->subGoalDistAngle = this->nh.subscribe("simple_move/goal_dist_angle", 1, &SimpleMoveNode::callbackGoalDistAngle, this);
    this->subGoalPose = this->nh.subscribe("simple_move/goal_pose", 1, &SimpleMoveNode::callbackGoalPose, this);
    this->subGoalRelativePose = this->nh.subscribe("simple_move/goal_rel_pose", 1, &SimpleMoveNode::callbackGoalRelPose, this);
    this->subGoalPath = this->nh.subscribe("simple_move/goal_path", 1, &SimpleMoveNode::callbackGoalPath, this);
    this->subCurrentPose = this->nh.subscribe("/navigation/localization/current_pose", 1, &SimpleMoveNode::callbackCurrentPose, this);
    //TODO: Read robot diameter from param server or urdf or something similar
    this->control.SetRobotParams(0.48);
    this->tf_listener = new tf::TransformListener();
}

void SimpleMoveNode::spin()
{
    ros::Rate loop(20);
    std_msgs::Float32MultiArray speeds;
    std_msgs::Bool goalReached;
    //First element is the leftSpeed, and the second one is the rightSpeed
    speeds.data.push_back(0);
    speeds.data.push_back(0);
    tf::StampedTransform transform;
    tf::Quaternion q;
    tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
    bool correctFinalAngle = false;
    float errorAngle;
    std_msgs::Float32MultiArray headAngles;
    headAngles.data.push_back(0);
    headAngles.data.push_back(0);
    
    while(ros::ok())
    {
        if(this->newGoal)
        {
            tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
            this->currentX = transform.getOrigin().x();
            this->currentY = transform.getOrigin().y();
            q = transform.getRotation();
            this->currentTheta = atan2((float)q.z(), (float)q.w()) * 2;

            float errorX = this->goalX - this->currentX;
            float errorY = this->goalY - this->currentY;
            float error = sqrt(errorX*errorX + errorY*errorY);
            if(error < 0.05)
            {
                speeds.data[0] = 0;
                speeds.data[1] = 0;
                pubSpeeds.publish(speeds);
                this->newGoal = false;
                correctFinalAngle = true;
            }
            else
            {
                control.CalculateSpeeds(this->currentX, this->currentY, this->currentTheta, this->goalX, this->goalY,
                                        speeds.data[0], speeds.data[1], moveBackwards);
                //std::cout << "SimpleMove.->Speeds: " << speeds.data[0] << "  " << speeds.data[1] << std::endl;
                pubSpeeds.publish(speeds);
            }
        }
        if(correctFinalAngle)
        {
            tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
            q = transform.getRotation();
            this->currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
            errorAngle = this->goalTheta - this->currentTheta;
            if(errorAngle > M_PI) errorAngle -= 2*M_PI;
            if(errorAngle <= -M_PI) errorAngle += 2*M_PI;

            if(fabs(errorAngle) < 0.05)
            {
                goalReached.data = true;
                pubGoalReached.publish(goalReached);
                speeds.data[0] = 0;
                speeds.data[1] = 0;
                pubSpeeds.publish(speeds);
                correctFinalAngle = false;
            }
            else
            {
                control.CalculateSpeeds(this->currentTheta, this->goalTheta, speeds.data[0], speeds.data[1]);
                pubSpeeds.publish(speeds);
            }
        }
        if(this->newPath)
        {
            tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
            this->currentX = transform.getOrigin().x();
            this->currentY = transform.getOrigin().y();
            q = transform.getRotation();
            this->currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
            
            float error = 0;
            float localGoalX, localGoalY, errorX, errorY;
            float tolerance;
            //This makes the robot to always try to reach the point 0.4m ahead the current position
            do
            {
                localGoalX = this->goalPath.poses[this->currentPathPose].pose.position.x;
                localGoalY = this->goalPath.poses[this->currentPathPose].pose.position.y;
                errorX = localGoalX - currentX;
                errorY = localGoalY - currentY;
                error = sqrt(errorX*errorX + errorY*errorY);
                tolerance = (this->goalPath.poses.size() - this->currentPathPose)*0.05 + 0.1;
                if(tolerance > 0.4)
                    tolerance = 0.4;
            }while(error < tolerance && ++this->currentPathPose < this->goalPath.poses.size());
            
            if(this->currentPathPose == this->goalPath.poses.size())
            {
                std::cout << "SimpleMove.->Last pose of goal path reached (Y)" << std::endl;
                goalReached.data = true;
                pubGoalReached.publish(goalReached);
                speeds.data[0] = 0;
                speeds.data[1] = 0;
                pubSpeeds.publish(speeds);
                headAngles.data[0] = 0;
                headAngles.data[1] = 0;
                if(this->moveHead)
                    this->pubHeadGoalPose.publish(headAngles);
                this->newPath = false;
            }
            else
            {
                //std::cout << "SimpleMove.->Goal: " << goalX << "  " << goalY << std::endl;
                control.CalculateSpeeds(this->currentX, this->currentY, this->currentTheta, localGoalX, localGoalY,
                                        speeds.data[0], speeds.data[1], moveBackwards);
                //std::cout << "SimpleMove.->Speeds: " << speeds.data[0] << "  " << speeds.data[1] << std::endl;
                pubSpeeds.publish(speeds);
                //Calculation of the head angle for pointing to the next poses. Head looks 30 cells forward
                int cellToLook = this->currentPathPose + 20;
                headAngles.data[0] = 0; //pan
                headAngles.data[1] = -0.9; //tilt
                if(cellToLook < this->goalPath.poses.size())
                {
                    float diffY = this->goalPath.poses[cellToLook].pose.position.y - this->currentY;
                    float diffX = this->goalPath.poses[cellToLook].pose.position.x - this->currentX; 
                    float temp = atan2(diffY, diffX);
                    float lookingAngle = temp - this->currentTheta;
                    if(lookingAngle > M_PI) lookingAngle -= 2*M_PI;
                    if(lookingAngle <= -M_PI) lookingAngle += 2*M_PI;
                    headAngles.data[0] = lookingAngle;
                }
                if(this->moveHead)
                    this->pubHeadGoalPose.publish(headAngles);
            }
        }
        ros::spinOnce();
        loop.sleep();
    }
}

void SimpleMoveNode::callbackRobotStop(const std_msgs::Empty::ConstPtr& msg)
{
    this->newPath = false;
    this->newGoal = false;
    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubGoalReached.publish(msgGoalReached);
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
    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubGoalReached.publish(msgGoalReached);
    std::cout << "SimpleMove.->Received new goal pose: " << msg->x << "  " << msg->y << "  " << msg->theta << std::endl;
}

void SimpleMoveNode::callbackGoalDist(const std_msgs::Float32::ConstPtr& msg)
{
    //Moves the distance 'msg' in the current direction. If dist < 0, robot moves backwards
    tf::StampedTransform transform;
    tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
    this->currentX = transform.getOrigin().x();
    this->currentY = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    this->currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
    
    this->goalX = this->currentX + msg->data*cos(this->currentTheta);
    this->goalY = this->currentY + msg->data*sin(this->currentTheta);
    if(msg->data > 0)
        this->goalTheta = atan2(goalY - currentY, goalX - currentX);
    else this->goalTheta = this->currentTheta;
    this->newGoal = true;
    this->moveBackwards = msg->data < 0;
    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubGoalReached.publish(msgGoalReached);
    std::cout << "SimpleMove.->Received new goal distance: " << msg->data << std::endl;
}

void SimpleMoveNode::callbackGoalDistAngle(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //Moves the distance 'msg' in the current direction. If dist < 0, robot moves backwards
    tf::StampedTransform transform;
    tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
    this->currentX = transform.getOrigin().x();
    this->currentY = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    this->currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
    
    this->goalX = this->currentX + msg->data[0]*cos(this->currentTheta + msg->data[1]);
    this->goalY = this->currentY + msg->data[0]*sin(this->currentTheta + msg->data[1]);

    if(msg->data[0] > 0)
        this->goalTheta = atan2(goalY - currentY, goalX - currentX);
    else this->goalTheta = this->currentTheta + msg->data[1];
    if(this->goalTheta > M_PI) this->goalTheta -= 2*M_PI;
    if(this->goalTheta <= -M_PI) this->goalTheta += 2*M_PI;

    this->newGoal = true;
    this->moveBackwards = msg->data[0] < 0;
    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubGoalReached.publish(msgGoalReached);
    std::cout << "SimpleMove.->Received new goal distance: " << msg->data[0] << " and angle: " << msg->data[1] << std::endl;
}

void SimpleMoveNode::callbackGoalRelPose(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    //Moves the relative position given by
    tf::StampedTransform transform;
    tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
    this->currentX = transform.getOrigin().x();
    this->currentY = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    this->currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
    
    this->goalX = this->currentX + msg->x*cos(this->currentTheta) - msg->y*sin(this->currentTheta);
    this->goalY = this->currentY + msg->x*sin(this->currentTheta) + msg->y*cos(this->currentTheta);
    this->goalTheta = this->currentTheta + msg->theta;
    this->newGoal = true;
    this->moveBackwards = false;
    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubGoalReached.publish(msgGoalReached);
    std::cout << "SimpleMove.->Received new relative goal pose: " << msg->x << "  " << msg->y << "  " << msg->theta << std::endl;
}

void SimpleMoveNode::callbackGoalPath(const nav_msgs::Path::ConstPtr& msg)
{
    this->currentPathPose = 0;
    this->goalPath = *msg;
    this->newPath = true;
    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubGoalReached.publish(msgGoalReached);
    std::cout << "SimpleMove.->Received new goal path with " << msg->poses.size() << " poses. " << std::endl;
}
