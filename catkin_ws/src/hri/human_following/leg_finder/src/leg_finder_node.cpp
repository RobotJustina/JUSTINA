#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "justina_tools/JustinaTools.h"
#include "LegFinder.h"
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

pcl::PointXYZ robotPos = pcl::PointXYZ();
pcl::PointCloud<pcl::PointXYZ>::Ptr laserWrtMap(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCyl(new pcl::PointCloud<pcl::PointXYZ>);

bool poseUpdate = false; 
bool laserUdate = false; 

void callbackCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    //This topic constains the robot pose w.r.t map. It is published by the localization nod (amcl)
    robotPos.x = msg->pose.pose.position.x;
    robotPos.y = msg->pose.pose.position.y;
    robotPos.z = atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) * 2;
    poseUpdate = true; 
}

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //This topic contains the laser readings. It is published by hokuyo-node
    
    JustinaTools::laserScanToPclWrtRobot(msg,laserWrtMap);
    JustinaTools::laserScanToPclCylindrical(msg,laserCyl);
    //std::cout<<"Size laserCyl "<<laserCyl->size()<<" Size laserWrtMap "<<laserWrtMap->size() <<std::endl;
    laserUdate = true; 
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LEG FINDER ...." << std::endl;
    ros::init(argc, argv, "leg_finder");
    ros::NodeHandle n;
    ros::Subscriber subRobotPose = n.subscribe("/navigation/localization/current_pose", 1, callbackCurrentPose);
    ros::Subscriber subLaserScan = n.subscribe("/hardware/scan", 1, callbackLaserScan);
    ros::Publisher pubLegPose = n.advertise<geometry_msgs::PointStamped>("/hri/human_following/leg_poses", 1);
    geometry_msgs::PointStamped msgLegs;
    JustinaTools::setNodeHandle(&n);
    LegFinder legs = LegFinder();
    pcl::PointXYZ legsPos = pcl::PointXYZ();
    //This topic should contain the absolute position of the detected legs. 
    //An array is used due to the possibility of finding more than one pair of legs.
    //Header indicates the frame w.r.t the positions are expressed. Use of 'map' frame is preferred.
    ros::Rate loop(10);

    while(ros::ok())
    {
        if( laserUdate) //&& poseUpdate)
        {
            legs.findBestLegs(laserCyl,laserWrtMap,robotPos,legsPos);
            //std::cout<<legsPos.x<<" "<<legsPos.y<<std::endl;
            laserUdate = false; 
            poseUpdate = false;
            msgLegs.header.frame_id="base_link";
            msgLegs.point.x=legsPos.x;
            msgLegs.point.y=legsPos.y;
            pubLegPose.publish(msgLegs);
        }



        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
