#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "occupancy_grid_utils/ray_tracer.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


int main(int argc, char** argv)
{
    std::string file_name = "";
    bool use_bag = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("--bag") == 0)
        {
            use_bag = true;
            file_name = argv[++i];
        }
    }
    
    std::cout << "INITIALIZING LASER SIMULATOR BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "laser_simulator");
    ros::NodeHandle n;
    ros::Rate loop(10);

    nav_msgs::GetMap srvGetMap;
    ros::service::waitForService("/navigation/localization/static_map");
    ros::ServiceClient srvCltGetMap = n.serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    srvCltGetMap.call(srvGetMap);
    nav_msgs::OccupancyGrid map = srvGetMap.response.map;
    sensor_msgs::LaserScan scanInfo;
    scanInfo.header.frame_id = "laser_link";
    scanInfo.angle_min = -2;
    scanInfo.angle_max = 2;
    scanInfo.angle_increment = 0.007;
    scanInfo.scan_time = 0.1;
    scanInfo.range_min = 0.01;
    scanInfo.range_max = 4.0;
    sensor_msgs::LaserScan simulatedScan;
    sensor_msgs::LaserScan::Ptr msgFromBag;
    ros::Publisher pubScan = n.advertise<sensor_msgs::LaserScan>("scan", 1);

    tf::TransformListener listener;
    geometry_msgs::Pose sensorPose;
    geometry_msgs::Pose lastSensorPose;
    sensorPose.position.x = 0;
    sensorPose.position.y = 0;
    sensorPose.position.z = 0;
    sensorPose.orientation.x = 0;
    sensorPose.orientation.y = 0;
    sensorPose.orientation.z = 0;
    sensorPose.orientation.w = 1;

    if(use_bag)
    {
        rosbag::Bag bag;
        bag.open(file_name, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery("/hardware/scan"));
        while(ros::ok())
        {
            foreach(rosbag::MessageInstance const m, view)
            {
                msgFromBag = m.instantiate<sensor_msgs::LaserScan>();
                if(msgFromBag == NULL)
                {
                    loop.sleep();
                    continue;
                }
                msgFromBag->header.stamp = ros::Time::now();
                pubScan.publish(*msgFromBag);
                ros::spinOnce();
                loop.sleep();
                if(!ros::ok())
                    break;
            }
        }
        bag.close();
	return 0;
    }
    
    while(ros::ok())
    {
	tf::StampedTransform transform;
	tf::Quaternion q;
	try
	{
	    listener.lookupTransform("map", "base_link", ros::Time(0), transform);
	    sensorPose.position.x = transform.getOrigin().x();
	    sensorPose.position.y = transform.getOrigin().y();
	    q = transform.getRotation();
	    sensorPose.orientation.z = q.z();
	    sensorPose.orientation.w = q.w();
	}
	catch(...){std::cout << "LaserSimulator.-> Cannot get transform from base_link to map" << std::endl;}
	
        simulatedScan = *occupancy_grid_utils::simulateRangeScan(map, sensorPose, scanInfo);
	simulatedScan.header.stamp = ros::Time::now();
        pubScan.publish(simulatedScan);
        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}
