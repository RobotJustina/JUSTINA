#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "occupancy_grid_utils/ray_tracer.h"

geometry_msgs::Pose sensorPose;

void callbackCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    sensorPose = msg->pose.pose;
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LASER SIMULATOR BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "laser_simulator");
    ros::NodeHandle n;
    ros::Rate loop(10);

    nav_msgs::GetMap srvGetMap;
    ros::service::waitForService("/navigation/localization/static_map");
    ros::ServiceClient srvCltGetMap = n.serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    srvCltGetMap.call(srvGetMap);
    nav_msgs::OccupancyGrid map = srvGetMap.response.map;
    ros::Subscriber subCurrentPose = n.subscribe("/navigation/localization/current_pose", 1, callbackCurrentPose);
    sensor_msgs::LaserScan scanInfo;
    scanInfo.header.frame_id = "laser_link";
    scanInfo.angle_min = -2;
    scanInfo.angle_max = 2;
    scanInfo.angle_increment = 0.007;
    scanInfo.scan_time = 0.1;
    scanInfo.range_min = 0.01;
    scanInfo.range_max = 4.0;
    sensor_msgs::LaserScan simulatedScan;
    ros::Publisher pubScan = n.advertise<sensor_msgs::LaserScan>("scan", 1);
    
    while(ros::ok())
    {
        simulatedScan = *occupancy_grid_utils::simulateRangeScan(map, sensorPose, scanInfo);
        pubScan.publish(simulatedScan);
        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}
