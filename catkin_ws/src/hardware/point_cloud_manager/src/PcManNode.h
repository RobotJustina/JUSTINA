#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "point_cloud_manager/get_rgbd.h"
#include "tf/transform_listener.h"

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

    bool debugMode;
    cv::Mat depthMap;
    cv::Mat bgrImage;
    cv::VideoCapture capture;
    
    sensor_msgs::PointCloud2 cvMat2PointCloud2(cv::Mat& srcBgr, cv::Mat& srcDepth);
    bool kinectRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &res);
    bool robotRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &res);
};
