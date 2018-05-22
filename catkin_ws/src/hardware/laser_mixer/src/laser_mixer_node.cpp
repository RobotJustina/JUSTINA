#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

sensor_msgs::LaserScan msg_laser_hokuyo;
sensor_msgs::LaserScan msg_laser_kinect;

void callback_kinect(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    msg_laser_kinect = *msg;
}

void callback_hokuyo(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    msg_laser_hokuyo = *msg;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LASER MIXER NODE BY MARCOSOFT..."  << std::endl;
    ros::init(argc, argv, "laser_mixer");
    ros::NodeHandle n;
    ros::Subscriber sub_laser_kinect = n.subscribe("/hardware/scan_from_kinect", 1, callback_kinect);
    ros::Subscriber sub_laser_hokuyo = n.subscribe("/hardware/scan", 1, callback_hokuyo);
    ros::Publisher  pub_laser = n.advertise<sensor_msgs::LaserScan>("/hardware/scan_augmented", 1);
    ros::Rate loop(10);

    try{ msg_laser_hokuyo = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hardware/scan", ros::Duration(10));}
    catch(...)
    {
	std::cout << "LaserMixer.->Cannot get /hardware/scan topic" << std::endl;
	return 1;
    }
    std::cout << "LaserMixer.-> Hokuyo parameters: angle_min=" << msg_laser_hokuyo.angle_min << "\tangle_max=";
    std::cout << msg_laser_hokuyo.angle_max << "\tangle_increment" << msg_laser_hokuyo.angle_increment << std::endl;

    n.setParam("/hardware/kinect2laser/angle_min", msg_laser_hokuyo.angle_min);
    n.setParam("/hardware/kinect2laser/angle_max", msg_laser_hokuyo.angle_min + msg_laser_hokuyo.angle_increment*msg_laser_hokuyo.ranges.size());
    n.setParam("/hardware/kinect2laser/angle_increment", msg_laser_hokuyo.angle_increment);

    system("rosnode kill /hardware/kinect2laser");

    try{ msg_laser_kinect = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hardware/scan_from_kinect", ros::Duration(10));}
    catch(...)
    {
	std::cout << "LaserMixer.->Cannot get /hardware/scan_from_kinect topic" << std::endl;
	return 1;
    }

    if(msg_laser_hokuyo.ranges.size() != msg_laser_kinect.ranges.size())
    {
	std::cout << "LaserMixer.->ERROR!! scan from hokuyo and scan from kinect must have the same size!!" << std::endl;
	std::cout << "LaserMixer.->Hokuyo size=" << msg_laser_hokuyo.ranges.size() << "\tfrom kinect size=" << msg_laser_kinect.ranges.size() << std::endl;
	return 1;
    }

    sensor_msgs::LaserScan msg_augmented = msg_laser_hokuyo;
    
    while(ros::ok())
    {
	for(size_t i=0; i < msg_laser_hokuyo.ranges.size(); i++)
	    msg_augmented.ranges[i] = fmin(msg_laser_hokuyo.ranges[i], msg_laser_kinect.ranges[i]);

	msg_augmented.header.stamp = ros::Time::now();
	pub_laser.publish(msg_augmented);
	ros::spinOnce();
	loop.sleep();
    }
}
