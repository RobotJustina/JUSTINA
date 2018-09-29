#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
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

bool simulated = false;
bool is_rear = false;
sensor_msgs::LaserScan realLaserScan;
ros::NodeHandle* nh;
ros::Subscriber subRealLaserScan;
ros::ServiceClient srvCltGetMap;
ros::Publisher pubScan1;
ros::Publisher pubScan2;
bool dynamicMap = false;
nav_msgs::OccupancyGrid map;
bool first_scan = true;

void callback_laser_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    realLaserScan = *msg;
    if(first_scan)
        first_scan = false;
}

void callback_simulated(const std_msgs::Bool::ConstPtr &msg)
{
    simulated = msg->data;
    if(simulated)
    {
        std::cout << "LaserManager.->CHANGING LASER TO SIMULATED MODE!!!" << std::endl;
        subRealLaserScan.shutdown();
    }
    else
    {
        std::cout << "LaserManager.->CHANGING LASER TO REAL MODE !!!" << std::endl;
        subRealLaserScan = nh->subscribe("/hardware/real_scan", 1, callback_laser_scan);
    }
    nav_msgs::GetMap srvGetMap;
    ros::service::waitForService("/navigation/localization/static_map");
    srvCltGetMap = nh->serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    srvCltGetMap.call(srvGetMap);
    map = srvGetMap.response.map;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LASER MANAGER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "laser_manager");
    ros::NodeHandle n;
    nh = &n;
    
    std::string file_name = "";
    bool use_bag = false;
    if(ros::param::has("~bag"))
    {
        ros::param::get("~bag", file_name);
        use_bag = true;
    }
    if(ros::param::has("~simul"))
        ros::param::get("~simul", simulated);
    if(ros::param::has("~rear"))
        ros::param::get("~rear", is_rear);
    if(ros::param::has("~dynamic_map"))
        ros::param::get("~dynamic_map", dynamicMap);

    ros::Rate loop(60);
    ros::Rate loop_bag(10);    

    if(!simulated)
    {
        std::cout << "LaserManager.->USING LASER IN REAL MODE !!!" << std::endl;
        subRealLaserScan = nh->subscribe("/hardware/real_scan", 1, callback_laser_scan);
    }

    nav_msgs::GetMap srvGetMap;
    ros::service::waitForService("/navigation/localization/static_map");
    srvCltGetMap = n.serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    srvCltGetMap.call(srvGetMap);
    map = srvGetMap.response.map;

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
    pubScan1 = n.advertise<sensor_msgs::LaserScan>("/erlc/scan_1", 1);
    pubScan2 = n.advertise<sensor_msgs::LaserScan>("/erlc/scan_2", 1);
    ros::Subscriber subSimulated = n.subscribe("/simulated", 1, callback_simulated);

    tf::TransformListener listener;
    geometry_msgs::Pose sensorPose;
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
                loop_bag.sleep();
                if(!ros::ok())
                    break;
            }
        }
        bag.close();
        return 0;
    }

    //tf::Quaternion zrot(0,0,1,0);
    while(ros::ok())
    {
        if(simulated)
        {
            tf::StampedTransform transform;
            tf::Quaternion q;
            try
            {
                if(is_rear)
                    listener.lookupTransform("map", "laser_link_rear", ros::Time(0), transform);
                else
                    listener.lookupTransform("map", "laser_link", ros::Time(0), transform);
                sensorPose.position.x = transform.getOrigin().x();
                sensorPose.position.y = transform.getOrigin().y();
                q = transform.getRotation();
                sensorPose.orientation.x = q.x();
                sensorPose.orientation.y = q.y();
                sensorPose.orientation.z = q.z();
                sensorPose.orientation.w = q.w();
            }
            catch(...){std::cout << "LaserSimulator.-> Cannot get transform from base_link to map" << std::endl;}

            simulatedScan = *occupancy_grid_utils::simulateRangeScan(map, sensorPose, scanInfo);
            simulatedScan.header.stamp = ros::Time::now();
            if(is_rear)
            {
                simulatedScan.header.frame_id = "laser_link_rear";
                pubScan2.publish(simulatedScan);
            }
            else
                pubScan1.publish(simulatedScan);
            pubScan.publish(simulatedScan);
        }
        else
        {
            if(!first_scan){
                if(is_rear){
                    realLaserScan.header.frame_id = "laser_link_rear";
                    //for(int i = 0; i < realLaserScan.ranges.size(); i++){
                    //    if(realLaserScan.ranges[i] > 1.2)
                    //        realLaserScan.ranges[i] = 0;
                    //}
                    pubScan2.publish(realLaserScan);
                }else{
                    realLaserScan.header.frame_id = "laser_link";
                    pubScan1.publish(realLaserScan);
                }
                pubScan.publish(realLaserScan);
            }
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
