#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32.h"

#include "justina_tools/JustinaTools.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

ros::NodeHandle* n;
ros::Subscriber  sub_legs_pose;
ros::Publisher   pub_cmd_vel;
ros::Publisher   pub_head_pose;
ros::ServiceClient cltRgbdRobotDownsampled;
bool move_head = false;
float repulsiveForce = 0.0;
float KInfRep = 0.03;
bool usePotFields = false;

bool getKinectDataFromJustina( cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobotDownsampled.call(srv))
    {
        std::cout << "CubesSegmentation.->Cannot get point cloud" << std::endl;
        return false;
    }
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}

geometry_msgs::Twist calculate_speeds(float goal_x, float goal_y)
{
    //Control constants
    float alpha =0.6548;// 0.6548 ;//= 0.9; // = 1.2
    float beta = 0.3;
    float max_angular = 0.7;//0.7 // 0.8 // 0.7

    //Error calculation
    float angle_error = atan2(goal_y, goal_x);
    float distance    = sqrt(goal_x*goal_x + goal_y*goal_y);
    distance -= 0.7;
    if(distance <   0) distance = 0; //Robot will stop at 0.8 m from walker
    if(distance > 0.25) distance = 0.25; //Distance is used as speed, so, robot will move at 0.5 max
    geometry_msgs::Twist result;
    if(distance > 0)
    {
        result.linear.x  = distance * exp(-(angle_error * angle_error) / alpha);
        result.linear.y  = 0;
        //std::cout << usePotFields << std::endl;
        if(usePotFields)
            result.linear.y = KInfRep * repulsiveForce;
        std::cout << result.linear.y << std::endl;
        result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
    }
    else
    {
        result.linear.x  = 0;
        result.linear.y  = 0;
        if(fabs(angle_error) >= M_PI_4 / 6.0f)
            result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
        else
            result.angular.z = 0;
    }
    return result;
}

void callback_legs_pose(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if(msg->header.frame_id.compare("base_link") != 0)
    {
        std::cout << "LegFinder.->WARNING!! Leg positions must be expressed wrt robot" << std::endl;
        return;
    }
    pub_cmd_vel.publish(calculate_speeds(msg->point.x, msg->point.y));
    if(move_head)
    {
        if(sqrt(msg->point.x * msg->point.x + msg->point.y * msg->point.y) > 0.23){
            std_msgs::Float32MultiArray head_poses;
            head_poses.data.push_back(atan2(msg->point.y, msg->point.x));
            head_poses.data.push_back(-0.9);
            pub_head_pose.publish(head_poses);
        }
        else{
            std_msgs::Float32MultiArray head_poses;
            head_poses.data.push_back(0.0);
            head_poses.data.push_back(0.0);
            pub_head_pose.publish(head_poses);
        }
    }
}

void callback_pot_fields(const std_msgs::Float32::ConstPtr& msg){
    repulsiveForce = msg->data;
}

void callback_enable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        std::cout << "LegFinder.->Enable recevied" << std::endl;
        sub_legs_pose = n->subscribe("/hri/leg_finder/leg_poses", 1, callback_legs_pose);      
    }
    else
        sub_legs_pose.shutdown();
}

int main(int argc, char** argv)
{
    move_head = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("--move_head") == 0)
            move_head = true;
    }

    std::cout << "INITIALIZING HUMAN FOLLOWER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "human_follower");
    n = new ros::NodeHandle();
    ros::param::get("~use_pot_fields", usePotFields);
    ros::param::get("~k_inf_rep", KInfRep);
    ros::Subscriber sub_enable = n->subscribe("/hri/human_following/start_follow", 1, callback_enable);
    pub_cmd_vel   = n->advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    pub_head_pose = n->advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    cltRgbdRobotDownsampled = n->serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot_downsampled");
    ros::Subscriber subRepulsiveForce = n->subscribe<std_msgs::Float32>("/navigation/potential_fields/repulsive_force", 1, callback_pot_fields);
    ros::Rate loop(30);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    delete n;
}
