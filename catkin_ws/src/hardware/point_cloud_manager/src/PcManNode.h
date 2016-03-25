#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "point_cloud_manager/get_rgbd.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

class PcManNode
{
public:
    PcManNode();
    ~PcManNode();

    bool InitNode(ros::NodeHandle* n, bool debugMode);
    void spin();

private:
    tf::TransformListener tf_listener;
    ros::NodeHandle* n;
    ros::Publisher pubKinectFrame;
    ros::Publisher pubRobotFrame;
    ros::ServiceServer srvRgbdKinect;
    ros::ServiceServer srvRgbdRobot;
    sensor_msgs::PointCloud2 msgCloudKinect;
    sensor_msgs::PointCloud2 msgCloudRobot;

    bool debugMode;
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudKinect;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudRobot;
    pcl::Grabber* interface;
    std::string kinectFrame;
    std::string baseFrame;

    void point_cloud_callback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
    bool kinectRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &res);
    bool robotRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &res);
};
