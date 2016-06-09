#include <iostream>
#include <cmath>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "LegFinder.h"

std::vector<float> laser_ranges;
std::vector<float> laser_angles;
bool laserUpdate = false;

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ranges.clear();
    laser_angles.clear();
    for(size_t i=0; i < msg->ranges.size(); i++)
    {
        laser_ranges.push_back(msg->ranges[i]);
        laser_angles.push_back(msg->angle_min + i*msg->angle_increment);
    }
    laserUpdate = true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LEG FINDER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "leg_finder");
    ros::NodeHandle n;
    ros::Subscriber subLaserScan = n.subscribe("/hardware/scan", 1, callbackLaserScan);
    ros::Publisher pubLegPose = n.advertise<geometry_msgs::PointStamped>("/hri/human_following/leg_poses", 1);
    ros::Publisher pubLegLost = n.advertise<std_msgs::Empty>("/hri/human_following/leg_lost", 1);
    tf::TransformListener tf_listener;
        
    LegFinder legs = LegFinder();
    pcl::PointXYZ legPos;
    float distan;
    float robotX, robotY, robotTheta;
    geometry_msgs::PointStamped msgLegs;
    tf::StampedTransform transform;
    tf::Quaternion q;

    ros::Rate loop(10);
    tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
    msgLegs.header.frame_id = "base_link";
    bool frontalLegsFound = false;

    while(ros::ok())
    {
        if(laserUpdate)
        {
            laserUpdate = false;
            /*tf_listener.lookupTransform("map", "base_link", ros::Time(0), transform);
            robotX = transform.getOrigin().x();
            robotY = transform.getOrigin().y();
            q = transform.getRotation();
            robotTheta = atan2((float)q.z(), (float)q.w()) * 2;
            legs.setRobotPose(0, 0, robotTheta);*/
            if(!frontalLegsFound)
            {
                legs.laserCallback(laser_ranges, laser_angles);
                legs.findPiernasFrente(legPos, 2.0, 0.3);
                frontalLegsFound = legs.isThereMotionlessLegInFront();
            }
            else //If cannot find the best legs, it will try to find frontal legs again
                legs.findBestLegs(laser_ranges, laser_angles, legPos, distan);
            
            //std::cout << "LegFinder.->Motionless legs in front? :" << (int)legs.isThereMotionlessLegInFront() << std::endl;
            //std::cout<<legsPos.x<<" "<<legsPos.y<<std::endl;
            msgLegs.point.x=legPos.x;
            msgLegs.point.y=legPos.y;
            pubLegPose.publish(msgLegs);
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
