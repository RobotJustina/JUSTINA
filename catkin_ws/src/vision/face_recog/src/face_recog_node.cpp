#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Empty.h"
#include "vision_msgs/VisionObject.h"
#include "justina_tools/JustinaTools.h"

ros::Subscriber subPointCloud;
ros::NodeHandle* node;

void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
    //std::cout << "FaceRecognizer.->Received: width: " << bgrImg.cols << " height: " << bgrImg.rows << std::endl;
    cv::imshow("FACE RECOGNIZER BY MARCOSOFT", bgrImg);
}

void callbackStartRecog(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "FaceRecognizer.->Starting face recognition..." << std::endl;
    subPointCloud = node->subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, callbackPointCloud);
}

void callbackStopRecog(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "FaceRecognizer.->Stopping face recognition..." << std::endl;
    subPointCloud.shutdown();
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING FACE RECOGNIZER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "face_recognizer");
    ros::NodeHandle n;
    node = &n;
    ros::Subscriber subStartRecog = n.subscribe("/vision/face_recognizer/start_recog", 1, callbackStartRecog);
    ros::Subscriber subStopRecog = n.subscribe("/vision/face_recognizer/stop_recog", 1, callbackStartRecog);
    ros::Publisher pubFaces = n.advertise<vision_msgs::VisionObject>("/vision/face_recognizer/faces", 1);
    ros::Rate loop(30);
    cv::namedWindow("FACE RECOGNIZER BY MARCOSOFT", cv::WINDOW_AUTOSIZE);
    while(ros::ok() && cv::waitKey(15) != 27)
    {
        ros::spinOnce();
        loop.sleep();
    }
}
