#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"

#include "justina_tools/JustinaTools.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

ros::NodeHandle* n;
ros::Subscriber  sub_legs_pose;
ros::Publisher   pub_cmd_vel;
ros::Publisher   pub_head_pose;
ros::ServiceClient cltRgbdRobotDownsampled;
bool move_head = false;

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
    //float alpha = 0.6548;
    float alpha =1.2;//= 0.9;
    float beta = 0.3;
    float max_angular = 0.8;//0.7

    //Error calculation
    float angle_error = atan2(goal_y, goal_x);
    float distance    = sqrt(goal_x*goal_x + goal_y*goal_y);
    distance -= 0.7;
    if(distance <   0) distance = 0; //Robot will stop at 0.8 m from walker
    if(distance > 0.35) distance = 0.35; //Distance is used as speed, so, robot will move at 0.5 max
    geometry_msgs::Twist result;
    if(distance > 0)
    {
        result.linear.x  = distance * exp(-(angle_error * angle_error) / alpha);
        result.linear.y  = 0;
        result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
    }
    else
    {
        result.linear.x  = 0;
        result.linear.y  = 0;
        if(fabs(angle_error) >= M_PI_4 / 4.0f)
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
        std_msgs::Float32MultiArray head_poses;
        head_poses.data.push_back(atan2(msg->point.y, msg->point.x));
        head_poses.data.push_back(-0.2);
        pub_head_pose.publish(head_poses);
    }
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
    ros::Subscriber sub_enable = n->subscribe("/hri/human_following/start_follow", 1, callback_enable);
    pub_cmd_vel   = n->advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    pub_head_pose = n->advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    cltRgbdRobotDownsampled = n->serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot_downsampled");
    ros::Rate loop(20);

    while(ros::ok() && cv::waitKey(1) != 'q')
    {
        
        cv::Mat bgrImg;
        cv::Mat xyzCloud;
        getKinectDataFromJustina(bgrImg, xyzCloud);
        if(bgrImg.cols < 1 || bgrImg.rows < 1)
            return false;
        int counter = 0;
        float meanX = 0;
        float meanY = 0;
        for(int i=0; i< xyzCloud.cols; i++)
            for(int j=0; j< xyzCloud.rows; j++)
            {
                cv::Vec3f p = xyzCloud.at<cv::Vec3f>(j,i);
                if(p[2] < 0.02)
                {
                    bgrImg.data[3*(j*bgrImg.cols + i)] = 0;
                    bgrImg.data[3*(j*bgrImg.cols + i) + 1] = 0;
                    bgrImg.data[3*(j*bgrImg.cols + i) + 2] = 0;
                }
                /*if(p[0] >= minX && p[0] <= maxX && p[1] >= minY && p[1] <= maxY && p[2] >= z_threshold && p[2] < 1.0) 
                {
                    counter++;
                    meanX += p[0];
                    meanY += p[1];
                }*/
            }
        // cv::imshow("OBSTACLE DETECTOR BY MARCOSOFT", bgrImg);
        
        ros::spinOnce();
        loop.sleep();
    }
    delete n;
}
