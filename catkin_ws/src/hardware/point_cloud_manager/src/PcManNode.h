#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/algorithm/string/predicate.hpp>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "point_cloud_manager/get_rgbd.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
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
    ros::Subscriber subSavePointCloud;
    ros::Subscriber subStopSavingCloud;
    ros::ServiceServer srvRgbdKinect;
    ros::ServiceServer srvRgbdRobot;
    sensor_msgs::PointCloud2 msgCloudKinect;
    sensor_msgs::PointCloud2 msgCloudRobot;

    bool debugMode;
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudKinect;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudRobot;
    std::string kinectFrame;
    std::string baseFrame;
    bool saveCloud;
    std::string cloudFilePath;
    //pcl::visualization::CloudViewer viewer;

    void point_cloud_callback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
    void callback_save_cloud(const std_msgs::String::ConstPtr& msg);
    void callback_stop_saving_cloud(const std_msgs::Empty::ConstPtr& msg);
    bool kinectRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &res);
    bool robotRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &res);
};
