#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

void callbackCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    //This topic constains the robot pose w.r.t map. It is published by the localization nod (amcl)
    float robotX = msg->pose.pose.position.x;
    float robotY = msg->pose.pose.position.y;
    float robotTheta = atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) * 2;
}

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //This topic contains the laser readings. It is published by hokuyo-node
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LEG FINDER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "leg_finder");
    ros::NodeHandle n;
    ros::Subscriber subRobotPose = n.subscribe("/navigation/localization/current_pose", 1, callbackCurrentPose);
    ros::Subscriber subLaserScan = n.subscribe("/hardware/scan", 1, callbackLaserScan);
    ros::Publisher pubLegPose = n.advertise<geometry_msgs::PoseArray>("/hri/human_following/leg_poses", 1);
    //This topic should contain the absolute position of the detected legs. 
    //An array is used due to the possibility of finding more than one pair of legs.
    //Header indicates the frame w.r.t the positions are expressed. Use of 'map' frame is preferred.
    ros::Rate loop(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
}
