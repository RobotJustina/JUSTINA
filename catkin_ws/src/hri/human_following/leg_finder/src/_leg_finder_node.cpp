#include <iostream>
#include <cmath>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "LegFinder.h"

std::vector<float> laser_ranges;
std::vector<float> laser_angles;
bool laserUpdate = false;
bool enable = false;
bool frontalLegsFound = false;
int legsFound_c=0;
int legsLoses_c=0;

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

void callbackEnable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
        std::cout << "LegFinder.->Received enable signal." << std::endl;
    else
    {
        std::cout << "LegFinder.->Received disabled signal." << std::endl;
        frontalLegsFound = false;
    }
    enable = msg->data;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LEG FINDER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "leg_finder");
    ros::NodeHandle n;
    ros::Subscriber subLaserScan = n.subscribe("/hardware/scan", 1, callbackLaserScan);
    ros::Subscriber subEnableDetection = n.subscribe("/hri/leg_finder/enable", 1, callbackEnable);
    ros::Publisher pubLegPose = n.advertise<geometry_msgs::PointStamped>("/hri/leg_finder/leg_poses", 1);
    ros::Publisher pubLegsFound = n.advertise<std_msgs::Bool>("/hri/leg_finder/legs_found", 1);
    tf::TransformListener tf_listener;
    
    ros::NodeHandle pnh("~");
    std::string frame_id;
    pnh.getParam("frame_id", frame_id);
    if(frame_id.compare("") == 0)
        frame_id = "laser_link";
        
    LegFinder legs = LegFinder();
    pcl::PointXYZ legPos;
    float distan;
    geometry_msgs::PointStamped msgLegs;
    std_msgs::Bool msgLegsFound;

    ros::Rate loop(10);
    msgLegs.header.frame_id = frame_id;

    while(ros::ok())
    {
        if(laserUpdate && enable)
        {
            laserUpdate = false;
            if(!frontalLegsFound)
            {
                legs.laserCallback(laser_ranges, laser_angles);
                legs.findPiernasFrente(legPos, 2.0, 0.3);
                if(frontalLegsFound = legs.isThereMotionlessLegInFront())
                    msgLegsFound.data=true;
                
                else
                    msgLegsFound.data=false; 
                
                pubLegsFound.publish(msgLegsFound);
            }
            else
            {
                legs.findBestLegs(laser_ranges, laser_angles, legPos, distan);   
                //std::cout << "LegFinder.->Motionless legs in front? :" << (int)legs.isThereMotionlessLegInFront() << std::endl;
                //std::cout<<legsPos.x<<" "<<legsPos.y<<std::endl;
                msgLegs.point.x=legPos.x;
                msgLegs.point.y=legPos.y;

                if(legPos.x == 0 && legPos.y == 0){
                    legsLoses_c++;
                    legsFound_c=0;
                    if(legsLoses_c > 15)
                        msgLegsFound.data=false;
                       
                }
                else{
                    legsFound_c++;
                    legsLoses_c=0;
                    if(legsFound_c > 7)
                        msgLegsFound.data=true;
                }

                pubLegsFound.publish(msgLegsFound);
                pubLegPose.publish(msgLegs);
            }
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
