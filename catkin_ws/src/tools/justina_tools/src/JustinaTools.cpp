#include "justina_tools/JustinaTools.h"

bool JustinaTools::is_node_set = false;
tf::TransformListener* JustinaTools::tf_listener;
int JustinaTools::counter = 0;

bool JustinaTools::setNodeHandle(ros::NodeHandle* nh)
{
	tf_listener = new tf::TransformListener();
    tf_listener->waitForTransform("map", "laser_link", ros::Time(0), ros::Duration(10.0));
}
void JustinaTools::laserScanToStdVectors(sensor_msgs::LaserScan& readings, std::vector<float>& robotX, std::vector<float>& robotY, std::vector<float>& mapX, std::vector<float>& mapY)
{
	
}

void JustinaTools::laserScanToPclWrtRobot(const sensor_msgs::LaserScan::ConstPtr& readings, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	tf::StampedTransform transformTf;
    tf_listener->lookupTransform("base_link","laser_link", ros::Time(0), transformTf);
    //std::cout<<"ya encontre la tr "<< (++counter) << std::endl;
    Eigen::Affine3d transformEigen;
    tf::transformTFToEigen(transformTf, transformEigen);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLaser(new pcl::PointCloud<pcl::PointXYZ>);
	cloudLaser->width = readings->ranges.size();
	cloudLaser->height = 1;
	cloudLaser->points.resize(cloudLaser->width * cloudLaser->height);
	for (size_t i = 0; i < cloudLaser->points.size(); ++i)
	{
		cloudLaser->points[i].x = readings->ranges[i] * cos(readings->angle_min + i * readings->angle_increment);
		cloudLaser->points[i].y = readings->ranges[i] * sin(readings->angle_min + i * readings->angle_increment);
	}
    pcl::transformPointCloud(*cloudLaser, *cloud, transformEigen);
        
}

void JustinaTools::laserScanToPclCylindrical(const sensor_msgs::LaserScan::ConstPtr& readings, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	//tf::StampedTransform transformTf;
    //tf_listener->lookupTransform("map","laser_link", ros::Time(0), transformTf);
	//Eigen::Affine3d transformEigen;
    //tf::transformTFToEigen(transformTf, transformEigen);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLaser(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = readings->ranges.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = readings->ranges[i];
		cloud->points[i].y =readings->angle_min + i * readings->angle_increment;
	}
	//pcl::transformPointCloud(*cloudLaser, *cloud, transformEigen);
}
