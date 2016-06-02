#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"

bool obsInFront = false;
float minSearchAngle = 0; //This angles will be determined according to the point which is 0.5 m ahead
float maxSearchAngle = 0; //In the calculated path

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBSTACLE DETECTOR (ONLY LASER) NODE BY MARCOSOFT... " << std::endl;
    ros::init(argc, argv, "obs_detect");
    ros::NodeHandle n;
    ros::Subscriber subLaserScan = n.subscribe("/hardware/scan", 1, callbackLaserScan);
    ros::Publisher pubObstacleInFront = n.advertise<std_msgs::Bool>("/navigation/obs_avoid/obs_in_front", 1);
    ros::Rate loop(10);

    std_msgs::Bool msgObsInFront;
    msgObsInFront.data = obsInFront;
    while(ros::ok())
    {
        pubObstacleInFront.publish(msgObsInFront);
        ros::spinOnce();
        loop.sleep();
    }
}
