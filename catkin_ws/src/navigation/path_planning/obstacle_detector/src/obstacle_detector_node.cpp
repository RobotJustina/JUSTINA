#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Path.h"
#include "justina_tools/JustinaTools.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ObstacleDetector.h"

ros::Subscriber subPointCloud;
ros::NodeHandle* node;
nav_msgs::Path lastPath;
pcl::PointCloud<pcl::PointXYZRGBA> cloud;
ObstacleDetector obstacleDetector;

void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
    //std::cout << "ObstacleDetector.->Received: width: " << bgrImg.cols << " height: " << bgrImg.rows << std::endl;
    cv::imshow("OBSTACLE DETECTOR BY MARCOSOFT", bgrImg);
    pcl::fromROSMsg(*msg, cloud);
}

void callbackStartDetection(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "ObstacleDetector.->Starting obstacle detection..." << std::endl;
    subPointCloud = node->subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, callbackPointCloud);
    cv::namedWindow("OBSTACLE DETECTOR BY MARCOSOFT", cv::WINDOW_AUTOSIZE);
}

void callbackStopDetection(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "ObstacleDetector.->Stopping obstacle detection..." << std::endl;
    subPointCloud.shutdown();
    cv::destroyWindow("OBSTACLE DETECTOR BY MARCOSOFT");
}

void callbackLastCalcPath(const nav_msgs::Path::ConstPtr& msg)
{
    lastPath = *msg;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBSTACLE DETECTOR NODE... " << std::endl;
    ros::init(argc, argv, "obstacle_detector");
    ros::NodeHandle n;
    node = &n;
    ros::Subscriber subStartDetection = n.subscribe("/navigation/obstacle_detector/start_detect", 1, callbackStartDetection);
    ros::Subscriber subStopDetection = n.subscribe("/navigation/obstacle_detector/stop_detect", 1, callbackStopDetection);
    ros::Subscriber subLastCalcPath = n.subscribe("/navigation/mvn_pln/last_calc_path", 1, callbackLastCalcPath);
    ros::Publisher pubObstacleDetected = n.advertise<std_msgs::Bool>("/navigation/obstacle_detected", 1);
    ros::Rate loop(10);

    while(ros::ok() && cv::waitKey(15) != 27)
    {
        ros::spinOnce();
        loop.sleep();
    }
}
