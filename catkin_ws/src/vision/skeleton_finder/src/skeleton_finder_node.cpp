#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Empty.h"
#include "vision_msgs/Skeletons.h"
#include "justina_tools/JustinaTools.h"

ros::Subscriber subPointCloud;
ros::NodeHandle* node;
vision_msgs::Skeletons msgSkeletons;

void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
}

void callbackStartRecog(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "SkeletonFinder.->Starting skeleton recognition..." << std::endl;
    subPointCloud = node->subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, callbackPointCloud);
}

void callbackStopRecog(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "SkeletonFinder.->Stopping skeleton recognition..." << std::endl;
    subPointCloud.shutdown();
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING SKELETON FINDER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "skeleton_finder");
    ros::NodeHandle n;
    node = &n;
    ros::Subscriber subStartRecog = n.subscribe("/vision/skeleton_finder/start_recog", 1, callbackStartRecog);
    ros::Subscriber subStopRecog = n.subscribe("/vision/skeleton_finder/stop_recog", 1, callbackStartRecog);
    ros::Publisher pubSkeletons = n.advertise<vision_msgs::Skeletons>("/vision/skeleton_finder/skeletons", 1);
    ros::Rate loop(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
}
