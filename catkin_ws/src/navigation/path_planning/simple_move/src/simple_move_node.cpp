#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "tf/transform_listener.h"

#define SM_INIT 0
#define SM_GOAL_POSE_ACCEL 1
#define SM_GOAL_POSE_CRUISE 2
#define SM_GOAL_POSE_DECCEL 3
#define SM_GOAL_POSE_CORRECT_ANGLE 4
#define SM_GOAL_POSE_FINISH 10
#define SM_GOAL_PATH_ACCEL 5
#define SM_GOAL_PATH_CRUISE 6
#define SM_GOAL_PATH_DECCEL 7
#define SM_GOAL_PATH_FINISH 8
#define SM_COLLISION_RISK 9

float goal_distance  = 0;
float goal_angle     = 0;
bool  move_lateral   = 0;
bool  new_pose       = false;
bool  new_path       = false;
bool  collision_risk = false;
nav_msgs::Path goal_path;
bool stop = false;

void callback_robot_stop(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "SimpleMove.->Stop signal received" << std::endl;
    stop     = true;
    new_pose = false;
    new_path = false;
}

void callback_goal_dist(const std_msgs::Float32::ConstPtr& msg)
{
    std::cout << "SimpleMove.->New move received: goal dist= " << msg->data << std::endl;     
    goal_distance = msg->data;
    goal_angle    = 0;
    move_lateral  = false;
    new_pose = true;
    new_path = false;
    stop     = false;
}

void callback_goal_dist_angle(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "SimpleMove.->New move received: goal dist= " << msg->data[0] << " and goal angle= " << msg->data[1] << std::endl;
    goal_distance = msg->data[0];
    goal_angle    = msg->data[1];
    move_lateral  = false;
    new_pose = true;
    new_path = false;
    stop     = false;
}

void callback_goal_path(const nav_msgs::Path::ConstPtr& msg)
{
    std::cout << "SimpleMove.->New path received with " << msg->poses.size() << " points" << std::endl;
    goal_path = *msg;
    move_lateral  = false;
    new_pose = false;
    new_path = true;
    stop     = false;
}

void callback_goal_lateral_dist(const std_msgs::Float32::ConstPtr& msg)
{
    std::cout << "SimpleMove.->New lateral distance received with dist= " << msg->data << std::endl;
    goal_distance = msg->data;
    goal_angle    = 0;
    move_lateral  = true;
    new_pose = true;
    new_path = false;
    stop     = false;
}

void callback_collision_risk(const std_msgs::Bool::ConstPtr& msg)
{
    collision_risk = msg->data;
}

geometry_msgs::Twist calculate_speeds(float robot_x, float robot_y, float robot_t, float goal_x, float goal_y,
        float cruise_speed, bool backwards)
{
    //Control constants
    float alpha = 0.6548;
    float beta = 0.1;
    float max_angular = 0.5;

    //Error calculation
    float angle_error = 0;
    if(backwards) angle_error = atan2(robot_y - goal_y, robot_x - goal_x) - robot_t;
    else angle_error = atan2(goal_y - robot_y, goal_x - robot_x) - robot_t;
    if(angle_error >   M_PI) angle_error -= 2*M_PI;
    if(angle_error <= -M_PI) angle_error += 2*M_PI;

    if(backwards) cruise_speed *= -1;
    geometry_msgs::Twist result;
    result.linear.x  = cruise_speed * exp(-(angle_error * angle_error) / alpha);
    result.linear.y  = 0;
    result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
    return result;
}

geometry_msgs::Twist calculate_speeds_lateral(float robot_x, float robot_y, float robot_t, float goal_x,
        float goal_y, float cruise_speed)
{
    //Control constants
    float alpha = 0.6548;
    float beta = 0.2;
    float max_angular = 0.5;

    //Error calculation
    float angle_error = atan2(robot_x - goal_x ,goal_y - robot_y) - robot_t;
    if(angle_error >   M_PI) angle_error -= 2*M_PI;
    if(angle_error <= -M_PI) angle_error += 2*M_PI;
    bool backwards = angle_error > M_PI/2 || angle_error < -M_PI/2; //Then we should move laterally but "backwards"
    if(backwards) angle_error += M_PI;
    if(angle_error >   M_PI) angle_error -= 2*M_PI;

    if(backwards) cruise_speed *= -1;
    geometry_msgs::Twist result;
    result.linear.x  = 0;
    result.linear.y  = cruise_speed * exp(-(angle_error * angle_error) / alpha);
    result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
    return result;

}

geometry_msgs::Twist calculate_speeds(float robot_angle, float goal_angle)
{
    //Control constants
    float beta = 0.2;
    float max_angular = 0.5;

    float angle_error = goal_angle - robot_angle;
    if(angle_error >   M_PI) angle_error -= 2*M_PI;
    if(angle_error <= -M_PI) angle_error += 2*M_PI;

    geometry_msgs::Twist result;
    result.linear.x  = 0;
    result.linear.y  = 0;
    result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
    return result;
}

void get_robot_position_wrt_map(tf::TransformListener& tf_listener, float& robot_x, float& robot_y, float& robot_t)
{
    tf::StampedTransform transform;
    tf_listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    robot_x = transform.getOrigin().x();
    robot_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    robot_t = atan2((float)q.z(), (float)q.w()) * 2;
}

void get_robot_position_wrt_odom(tf::TransformListener& tf_listener, float& robot_x, float& robot_y, float& robot_t)
{
    tf::StampedTransform transform;
    tf_listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
    robot_x = transform.getOrigin().x();
    robot_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    robot_t = atan2((float)q.z(), (float)q.w()) * 2;
}

void get_goal_position_wrt_odom(float goal_distance, float goal_angle, tf::TransformListener& tf_listener,
        float& goal_x, float& goal_y, float& goal_t, bool lateral)
{
    tf::StampedTransform transform;
    tf_listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
    float robot_x = transform.getOrigin().x();
    float robot_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    float robot_t = atan2((float)q.z(), (float)q.w()) * 2;

    if(lateral) goal_angle += M_PI / 2;
    goal_x = robot_x + goal_distance * cos(robot_t + goal_angle);
    goal_y = robot_y + goal_distance * sin(robot_t + goal_angle);
    goal_t = robot_t + goal_angle;
    if(lateral) goal_t -= M_PI / 2;
    if(goal_t >   M_PI) goal_t -= 2*M_PI;
    if(goal_t <= -M_PI) goal_t += 2*M_PI;
}

void get_next_goal_from_path(float& robot_x, float& robot_y, float& robot_t, float& goal_x, float& goal_y,
        int& next_pose_idx, tf::TransformListener& tf_listener)
{
    get_robot_position_wrt_map(tf_listener, robot_x, robot_y, robot_t);
    if(next_pose_idx >= goal_path.poses.size()) next_pose_idx = goal_path.poses.size() - 1;
    float error = 0;
    do
    {
        goal_x = goal_path.poses[next_pose_idx].pose.position.x;
        goal_y = goal_path.poses[next_pose_idx].pose.position.y;
        error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
    }while(error < 0.25 && ++next_pose_idx < goal_path.poses.size());
}

void get_head_angles(float robot_x, float robot_y, float robot_t, int next_pose_idx, float& head_pan, float& head_tilt)
{
    next_pose_idx += 20;
    if(next_pose_idx >= goal_path.poses.size())
        next_pose_idx = goal_path.poses.size() - 1;
    float goal_x = goal_path.poses[next_pose_idx].pose.position.x;
    float goal_y = goal_path.poses[next_pose_idx].pose.position.y;
    float error = atan2(goal_y - robot_y, goal_x - robot_x) - robot_t;
    if(error >   M_PI) error -= 2*M_PI;
    if(error <= -M_PI) error += 2*M_PI;
    head_pan  = error;
    head_tilt = -0.9;
}

int main(int argc, char** argv)
{
    bool move_head = false;
    for(int i=0; i < argc; i++)
    {
        std::string str_param(argv[i]);
        if(str_param.compare("--move_head") == 0)
            move_head = true;
    }
    std::cout << "INITIALIZING A REALLY GOOD SIMPLE MOVE NODE BY MARCOSOFT..." << std::endl;

    //
    //VARIABLES FOR ROS CONNECTION
    ros::init(argc, argv, "simple_move");
    ros::NodeHandle n;
    ros::Publisher  pub_goal_reached     = n.advertise<std_msgs::Bool>("/navigation/goal_reached", 1);                           
    ros::Publisher  pub_speeds           = n.advertise<std_msgs::Float32MultiArray>("/hardware/mobile_base/speeds", 1);          
    ros::Publisher  pub_cmd_vel          = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);  
    ros::Publisher  pub_head             = n.advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/hd_goto_angles", 1); 
    ros::Subscriber sub_robotStop        = n.subscribe("/hardware/robot_state/stop", 1, callback_robot_stop);               
    ros::Subscriber sub_goalDistance     = n.subscribe("simple_move/goal_dist", 1, callback_goal_dist);                     
    ros::Subscriber sub_goalDistAngle    = n.subscribe("simple_move/goal_dist_angle", 1, callback_goal_dist_angle);          
    ros::Subscriber sub_goalPath         = n.subscribe("simple_move/goal_path", 1, callback_goal_path);                     
    ros::Subscriber sub_goalLateralDist  = n.subscribe("simple_move/goal_lateral", 1, callback_goal_lateral_dist);           
    ros::Subscriber sub_gollisionRisk    = n.subscribe("/navigation/obs_avoid/collision_risk", 10, callback_collision_risk);
    tf::TransformListener tf_listener;
    ros::Rate loop(20);

    tf::StampedTransform transform;
    tf::Quaternion q;
    try
    {
        tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
        tf_listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(10.0));
        tf_listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        tf_listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
    }
    catch(...)
    {
        std::cout << "SimpleMove.->Cannot get tranforms for robot's pose calculation... :'(" << std::endl;
        return -1;
    }

    int state = SM_INIT;
    std_msgs::Bool msg_goal_reached;
    std_msgs::Float32MultiArray msg_head;
    geometry_msgs::Twist twist;
    geometry_msgs::Twist zero_twist;
    zero_twist.linear.x  = 0;
    zero_twist.linear.y  = 0;
    zero_twist.angular.z = 0;
    msg_head.data.resize(2);
    float robot_x = 0;
    float robot_y = 0;
    float robot_t = 0;
    float goal_x = 0;
    float goal_y = 0;
    float goal_t = 0;
    float error  = 0;
    float cruise_speed  = 0;
    float global_goal_x = 0;
    float global_goal_y = 0;
    int next_pose_idx = 0;
    int attempts = 0;

    while(ros::ok())
    {
        if(stop)
        {
            stop = false;
            state = SM_INIT;
            msg_goal_reached.data = false;
            pub_cmd_vel.publish(zero_twist);
            pub_goal_reached.publish(msg_goal_reached);
        }
        if(new_pose || new_path)
            state = SM_INIT;
        //
        //STATE MACHINE FOR PLANNING MOVEMENTS
        switch(state)
        {
            case SM_INIT:
                cruise_speed = 0;
                if(new_pose)
                {
                    get_goal_position_wrt_odom(goal_distance, goal_angle, tf_listener, goal_x, goal_y, goal_t, move_lateral);
                    state = SM_GOAL_POSE_ACCEL;
                    new_pose = false;
                    attempts = (int)((fabs(goal_distance)+0.1)/0.2*60 + fabs(goal_angle)/0.5*60);
                    msg_goal_reached.data = false;
                    pub_goal_reached.publish(msg_goal_reached);
                }
                else if(new_path)
                {
                    state = SM_GOAL_PATH_ACCEL;
                    new_path = false;
                    next_pose_idx = 0;
                    global_goal_x = goal_path.poses[goal_path.poses.size() - 1].pose.position.x;
                    global_goal_y = goal_path.poses[goal_path.poses.size() - 1].pose.position.y;
                    msg_goal_reached.data = false;
                    pub_goal_reached.publish(msg_goal_reached);
                }
                break;


            case SM_GOAL_POSE_ACCEL:
                cruise_speed += 0.007;
                get_robot_position_wrt_odom(tf_listener, robot_x, robot_y, robot_t);
                error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
                if(error < 0.035)
                    state = SM_GOAL_POSE_CORRECT_ANGLE;
                else
                {
                    if(error < cruise_speed)
                        state = SM_GOAL_POSE_DECCEL;
                    else if(cruise_speed >= 0.2)
                        state = SM_GOAL_POSE_CRUISE;
                    else
                        state = SM_GOAL_POSE_ACCEL;

                    if(move_lateral)
                        twist = calculate_speeds_lateral(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed);
                    else
                        twist = calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed, goal_distance < 0);
                    pub_cmd_vel.publish(twist);
                }
                if(--attempts <= 0)
                    state = SM_GOAL_POSE_FINISH;
                break;


            case SM_GOAL_POSE_CRUISE:
                get_robot_position_wrt_odom(tf_listener, robot_x, robot_y, robot_t);
                error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
                if(error < cruise_speed)
                    state = SM_GOAL_POSE_DECCEL;

                if(move_lateral)
                    twist = calculate_speeds_lateral(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed);
                else
                    twist = calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed, goal_distance < 0);
                pub_cmd_vel.publish(twist);
                if(--attempts <= 0)
                    state = SM_GOAL_POSE_FINISH;
                break;


            case SM_GOAL_POSE_DECCEL:
                cruise_speed -= 0.007;
                get_robot_position_wrt_odom(tf_listener, robot_x, robot_y, robot_t);
                error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
                if(error < 0.035 || cruise_speed <= 0)
                    state = SM_GOAL_POSE_CORRECT_ANGLE;

                if(move_lateral)
                    twist = calculate_speeds_lateral(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed);
                else
                    twist = calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed, goal_distance < 0);
                pub_cmd_vel.publish(twist);
                if(--attempts <= 0)
                    state = SM_GOAL_POSE_FINISH;
                break;


            case SM_GOAL_POSE_CORRECT_ANGLE:
                get_robot_position_wrt_odom(tf_listener, robot_x, robot_y, robot_t);
                error = fabs(goal_t - robot_t);
                if(error < 0.015)
                    state = SM_GOAL_POSE_FINISH;
                else
                {
                    twist = calculate_speeds(robot_t, goal_t);
                    pub_cmd_vel.publish(twist);
                }
                if(--attempts <= 0)
                    state = SM_GOAL_POSE_FINISH;
                break;


            case SM_GOAL_POSE_FINISH:
                std::cout << "SimpleMove.->Successful move with dist=" << goal_distance << " angle=" << goal_angle << std::endl;
                state = SM_INIT;
                msg_goal_reached.data = true;
                pub_goal_reached.publish(msg_goal_reached);
                pub_cmd_vel.publish(zero_twist);
                break;


            case SM_GOAL_PATH_ACCEL:
                cruise_speed += 0.01;
                get_next_goal_from_path(robot_x, robot_y, robot_t, goal_x, goal_y, next_pose_idx, tf_listener);
                error =sqrt((global_goal_x - robot_x)*(global_goal_x - robot_x) + (global_goal_y - robot_y)*(global_goal_y - robot_y));
                if(error < 0.05)
                    state = SM_GOAL_PATH_FINISH;
                else if(collision_risk)
                {
                    std::cout << "SimpleMove.->Collision risk detected!!!!!!" << std::endl;
                    msg_goal_reached.data = false;
                    pub_cmd_vel.publish(zero_twist);
                    pub_goal_reached.publish(msg_goal_reached);
                    state = SM_INIT;
                }
                else
                {
                    if(error < cruise_speed*1.5)
                        state = SM_GOAL_PATH_DECCEL;
                    else if(cruise_speed >= 0.35)
                        state = SM_GOAL_PATH_CRUISE;

                    twist = calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed, false);
                    pub_cmd_vel.publish(twist);
                }
                break;


            case SM_GOAL_PATH_CRUISE:
                if(collision_risk)
                {
                    std::cout << "SimpleMove.->Collision risk detected!!!!!!" << std::endl;
                    msg_goal_reached.data = false;
                    pub_cmd_vel.publish(zero_twist);
                    pub_goal_reached.publish(msg_goal_reached);
                    state = SM_INIT;
                }
                else
                {
                    get_next_goal_from_path(robot_x, robot_y, robot_t, goal_x, goal_y, next_pose_idx, tf_listener);
                    error =sqrt((global_goal_x-robot_x)*(global_goal_x-robot_x) + (global_goal_y-robot_y)*(global_goal_y-robot_y));
                    if(error < cruise_speed*1.5)
                        state = SM_GOAL_PATH_DECCEL;
                    twist = calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed, false);
                    pub_cmd_vel.publish(twist);
                }
                if(move_head)
                {
                    get_head_angles(robot_x, robot_y, robot_t, next_pose_idx, msg_head.data[0], msg_head.data[1]);
                    pub_head.publish(msg_head);
                }
                break;


            case SM_GOAL_PATH_DECCEL:
                if(collision_risk)
                {
                    std::cout << "SimpleMove.->Collision risk detected!!!!!!" << std::endl;
                    msg_goal_reached.data = false;
                    pub_cmd_vel.publish(zero_twist);
                    pub_goal_reached.publish(msg_goal_reached);
                    state = SM_INIT;
                }
                else
                {
                    get_next_goal_from_path(robot_x, robot_y, robot_t, goal_x, goal_y, next_pose_idx, tf_listener);
                    error =sqrt((global_goal_x-robot_x)*(global_goal_x-robot_x) + (global_goal_y-robot_y)*(global_goal_y-robot_y));
                    //cruise_speed -= 0.01;
                    cruise_speed = (0.01 - 0.36) / (0.05 - 0.54) * (error - 0.54) + 0.36;
                    if(error < 0.05 || cruise_speed <= 0)
                        state = SM_GOAL_PATH_FINISH;
                    twist = calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, cruise_speed, false);
                    pub_cmd_vel.publish(twist);
                }
                if(move_head)
                {
                    get_head_angles(robot_x, robot_y, robot_t, next_pose_idx, msg_head.data[0], msg_head.data[1]);
                    pub_head.publish(msg_head);
                }
                break;


            case SM_GOAL_PATH_FINISH:
                std::cout << "SimpleMove.->Path succesfully executed. (Y)" << std::endl;
                msg_goal_reached.data = true;
                pub_cmd_vel.publish(zero_twist);
                pub_goal_reached.publish(msg_goal_reached);
                state = SM_INIT;
                break;

            default:
                std::cout << "SimpleMove.->A VERY STUPID PERSON PROGRAMMED THIS SHIT. SORRY. :'(" << std::endl;
                return -1;
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
