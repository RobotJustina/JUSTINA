#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_listener.h"
#include "navig_msgs/PlanPath.h"
#include "justina_tools/JustinaTools.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

ros::NodeHandle* n;
ros::Publisher   pub_cmd_vel;
ros::Publisher   pub_head_pose;
ros::Publisher   pubObsAvoidEnable;
ros::ServiceClient cltRgbdRobotDownsampled;
bool legsFound = false;
bool enableHumanFollower = false;
bool collisionRisk = false;
float goalRobotX, goalRobotY, goalMapX, goalMapY;
tf::TransformListener * tf_listener;

void callback_legs_pose(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if(msg->header.frame_id.compare("base_link") != 0)
    {
        std::cout << "LegFinder.->WARNING!! Leg positions must be expressed wrt robot" << std::endl;
        return;
    }
    tf::StampedTransform transform;
    tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
    tf::Vector3 v(msg->point.x, msg->point.y, msg->point.z);
    v = transform * v;
    goalRobotX = msg->point.x;
    goalRobotY = msg->point.y;
    goalMapX = v.x();
    goalMapY = v.y();
    //pub_cmd_vel.publish(calculate_speeds(msg->point.x, msg->point.y));
}

geometry_msgs::Twist calculate_speeds(float goal_x, float goal_y)
{
    //Control constants
    //float alpha = 0.6548;
    float alpha =1.2;//= 0.9;
    float beta = 0.3;
    float max_angular = 0.8;//0.7

    //Error calculation
    float angle_error = atan2(goal_y, goal_x);
    geometry_msgs::Twist result;
    result.linear.x  = 0;
    result.linear.y  = 0;
    if(fabs(angle_error) >= M_PI_4 / 4.0f)
        result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
    else
        result.angular.z = 0;
    return result;
}

void callbackLegsFound(const std_msgs::Bool::ConstPtr& msg)
{
    legsFound = msg->data;
}

void callback_enable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
        std::cout << "LegFinder.->Enable recevied" << std::endl;
    else
        std::cout << "LegFinder.->Disable recevied" << std::endl;
    
    std_msgs::Float32MultiArray msgHead;
    msgHead.data.push_back(0.0);
    msgHead.data.push_back(-0.9);
    pub_head_pose.publish(msgHead);
    pubObsAvoidEnable.publish(msg);
    enableHumanFollower = msg->data;
}

void callbackCollisionRisk(const std_msgs::Bool::ConstPtr& msg)
{
    collisionRisk = msg->data;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING HUMAN FOLLOWER NAVIGATION..." << std::endl;
    ros::init(argc, argv, "human_follower");
    n = new ros::NodeHandle();
    tf_listener = new tf::TransformListener();
    ros::Subscriber sub_enable = n->subscribe("/hri/human_following/start_follow", 1, callback_enable);
    ros::Subscriber sub_legs_pose = n->subscribe("/hri/leg_finder/leg_poses", 1, callback_legs_pose);
    ros::Subscriber subCollisionRisk = n->subscribe("/navigation/obs_avoid/collision_risk", 1, callbackCollisionRisk);
    ros::Subscriber subLegsFound = n->subscribe("/hri/leg_finder/legs_found", 1, callbackLegsFound);
    cltRgbdRobotDownsampled = n->serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot_downsampled");
    ros::ServiceClient cliGetPlanPath = n->serviceClient<navig_msgs::PlanPath>("/navigation/mvn_pln/plan_path");
    pub_cmd_vel   = n->advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    pub_head_pose = n->advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    ros::Publisher pubSimpleMoveGoalPath = n->advertise<nav_msgs::Path>("/navigation/path_planning/simple_move/goal_path", 1);
    ros::Publisher pubSimpleMoveDistAngle = n->advertise<std_msgs::Float32MultiArray>("/navigation/path_planning/simple_move/goal_dist_angle",1);
    ros::Publisher pubRobotStop = n->advertise<std_msgs::Empty>("/hardware/robot_state/stop", 1);
    pubObsAvoidEnable = n->advertise<std_msgs::Bool>("/navigation/obs_avoid/enable", 1);
    float robotX, robotY;
    ros::Rate loop(30);

    while(ros::ok() && cv::waitKey(1) != 'q')
    {
        if(enableHumanFollower){
            tf::StampedTransform transform;
            tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
            robotX = transform.getOrigin().x();
            robotY = transform.getOrigin().y();
            float distance = sqrt(goalRobotX * goalRobotX + goalRobotY * goalRobotY);
            distance -= 0.7;
            if(distance <   0) distance = 0; //Robot will stop at 0.8 m from walker
            if(distance <= 0.35 && legsFound){
                std_msgs::Float32MultiArray msg;
                msg.data.push_back(0.0);
                msg.data.push_back(atan2(goalRobotY , goalRobotX));
                std_msgs::Float32MultiArray head_poses;
                head_poses.data.push_back(atan2(goalRobotY, goalRobotX));
                head_poses.data.push_back(-0.2);
                pub_head_pose.publish(head_poses);
                pubSimpleMoveDistAngle.publish(msg);
                //pub_cmd_vel.publish(calculate_speeds(msg->point.x, msg->point.y));
            }else if(distance > 0.35 && !collisionRisk){
                navig_msgs::PlanPath srv;
                srv.request.start_location_id = "";
                srv.request.goal_location_id = "";
                srv.request.start_pose.position.x = robotX;
                srv.request.start_pose.position.y = robotY;
                srv.request.goal_pose.position.x = goalMapX;
                srv.request.goal_pose.position.y = goalMapY;
                srv.request.useMap = true;
                srv.request.useLaser = false;
                srv.request.useKinect = false;
                if(cliGetPlanPath.call(srv))
                    pubSimpleMoveGoalPath.publish(srv.response.path);
            }
            else{
                std_msgs::Empty msg;
                pubRobotStop.publish(msg); 
            }
        }

        collisionRisk = false;
        loop.sleep();
        ros::spinOnce();
    }
    delete n;
}
