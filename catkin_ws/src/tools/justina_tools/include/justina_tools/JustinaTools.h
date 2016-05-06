#pragma once
#include <vector>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "point_cloud_manager/GetRgbd.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"
#include "sensor_msgs/LaserScan.h"

class JustinaTools
{
private:
	static bool is_node_set;
	static tf::TransformListener* tf_listener;
	static int counter;

public:
	static bool setNodeHandle(ros::NodeHandle* nh);
	static void laserScanToStdVectors(sensor_msgs::LaserScan& readings, std::vector<float>& robotX, std::vector<float>& robotY, std::vector<float>& mapX, std::vector<float>& mapY);
	static void laserScanToPclWrtMap(const sensor_msgs::LaserScan::ConstPtr& readings, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    static void PointCloud2Msg_ToCvMat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest);
    static void PointCloud2Msg_ToCvMat(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest);
};
