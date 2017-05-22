#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem/path.hpp>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "point_cloud_manager/GetRgbd.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

cv::Mat depthMap;
cv::Mat bgrImage;
pcl::PointCloud<pcl::PointXYZRGBA> cloudWrtKinect;
pcl::PointCloud<pcl::PointXYZRGBA> cloudWrtRobot;
pcl::PointCloud<pcl::PointXYZRGBA> downsampled;
bool saveCloud = false;
sensor_msgs::PointCloud2 msgCloudKinect;
sensor_msgs::PointCloud2 msgCloudRobot;
sensor_msgs::PointCloud2 msgDownsampled;
sensor_msgs::PointCloud2 msgDownsampledRobot;
tf::TransformListener* tf_listener;
std::string cloudFilePath = "";

void cvMatToPcl(cv::Mat& depth, cv::Mat& bgr, pcl::PointCloud<pcl::PointXYZRGBA>& cloud)
{
    if(depth.rows != bgr.rows || depth.cols != bgr.cols)
        return;

    cloud.width  = depth.cols;
    cloud.height = depth.rows;
    cloud.points.resize(cloud.width*cloud.height);
    cloud.is_dense = false;
    for(int i=0; i < depth.rows; i++)
        for(int j=0; j < depth.cols; j++)
        {
            cv::Vec3f p = depth.at<cv::Vec3f>(i,j);
            int idx = i*cloud.width + j;
            cloud.points[idx].x = p[0];
            cloud.points[idx].y = -p[1];
            cloud.points[idx].z = p[2];
            cloud.points[idx].b = bgr.data[3*idx + 0];
            cloud.points[idx].g = bgr.data[3*idx + 1];
            cloud.points[idx].r = bgr.data[3*idx + 2];
            cloud.points[idx].a = 1;
        }
}

void callback_save_cloud(const std_msgs::String::ConstPtr& msg)
{
    if(!boost::filesystem::portable_posix_name(msg->data))
    {
        std::cout << "KinectMan.->Cannot save point cloud: Invalid file name. " << std::endl;
        return;
    }
    if(boost::algorithm::ends_with(msg->data, ".pcd"))
        cloudFilePath = msg->data;
    else
        cloudFilePath = msg->data + ".pcd";

    //cloudFilePath = default_path + cloudFilePath;
    //std::cout << "PointCloudMan.->Start saving point cloud at: " << this->cloudFilePath << std::endl;
    //this->saveCloud = true;
}

void callback_stop_saving_cloud(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "KinectMan.->Stop saving point cloud" << std::endl;
    saveCloud = false;
}

bool kinectRgbd_callback(point_cloud_manager::GetRgbd::Request &req, point_cloud_manager::GetRgbd::Response &resp)
{
    cvMatToPcl(depthMap, bgrImage, cloudWrtKinect);
    pcl::toROSMsg(cloudWrtKinect, msgCloudKinect);
    msgCloudKinect.header.frame_id = "kinect_link";
    msgCloudKinect.header.stamp = ros::Time::now();
    resp.point_cloud = msgCloudKinect;
    return true;
}

bool robotRgbd_callback(point_cloud_manager::GetRgbd::Request &req, point_cloud_manager::GetRgbd::Response &resp)
{
    cvMatToPcl(depthMap, bgrImage, cloudWrtKinect);
    pcl::toROSMsg(cloudWrtKinect, msgCloudKinect);
    msgCloudKinect.header.frame_id = "kinect_link";
    msgCloudKinect.header.stamp = ros::Time::now();
    tf_listener->waitForTransform("base_link", "kinect_link", msgCloudKinect.header.stamp, ros::Duration(0.5));
    pcl_ros::transformPointCloud("base_link", msgCloudKinect, msgCloudRobot, *tf_listener);
    msgCloudRobot.header.frame_id = "base_link";
    resp.point_cloud = msgCloudRobot;
    return true;
}

int main(int argc, char** argv)
{
    std::string videoFile = "";
    bool useVideo = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
        {
            videoFile = argv[++i];
            useVideo = true;
        }
    }
    
    std::cout << "INITIALIZING KINECT MANAGER BY MARCOSOF ..." << std::endl;
    if(useVideo)
        std::cout << "KinectMan.->Using video file: " << videoFile << std::endl;
    else
        std::cout << "KinectMan.->Using real kinect..." << std::endl;
    ros::init(argc, argv, "kinect_man");
    ros::NodeHandle n;
    tf_listener = new tf::TransformListener();
    ros::Publisher pubKinectFrame = n.advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_kinect",1);                           
    ros::Publisher pubRobotFrame  = n.advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot", 1);                            
    ros::Publisher pubRobotFrameDownsampled = n.advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot_downsampled", 1);     
    ros::Subscriber subSavePointCloud = n.subscribe("/hardware/point_cloud_man/save_cloud", 1, callback_save_cloud);               
    ros::Subscriber subStopSavingCloud = n.subscribe("/hardware/point_cloud_man/stop_saving_cloud", 1, callback_stop_saving_cloud);
    ros::ServiceServer srvRgbdKinect = n.advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_kinect", kinectRgbd_callback);      
    ros::ServiceServer srvRgbdRobot = n.advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_robot", robotRgbd_callback);         

    cv::VideoCapture capture;
    if(useVideo)
    {
        std::cout << "KinectMan.->Trying to open video " << videoFile << std::endl;
        capture.open(videoFile);
    }
    else
    {
        std::cout << "KinectMan.->Triying to initialize kinect sensor... " << std::endl;
        capture.open(CV_CAP_OPENNI);
    }

    if(!capture.isOpened())
    {
        std::cout << "KinectMan.->Cannot open kinect :'(" << std::endl;
        return 1;
    }
    capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON);
    std::cout << "KinectMan.->Kinect sensor started :D" << std::endl;

    ros::Rate loop(30);
    tf_listener->waitForTransform("base_link", "kinect_link", ros::Time(0), ros::Duration(10.0));
    
    while(ros::ok() && cv::waitKey(20) != 27)
    {
        if(!capture.grab())
        {
            loop.sleep();
            ros::spinOnce();
            continue;
        }
        capture.retrieve(depthMap, CV_CAP_OPENNI_POINT_CLOUD_MAP);
        capture.retrieve(bgrImage, CV_CAP_OPENNI_BGR_IMAGE);

        if(pubKinectFrame.getNumSubscribers() > 0)
        {
            cvMatToPcl(depthMap, bgrImage, cloudWrtKinect);
            pcl::toROSMsg(cloudWrtKinect, msgCloudKinect);
            msgCloudKinect.header.frame_id = "kinect_link";
            msgCloudKinect.header.stamp = ros::Time::now();
            pubKinectFrame.publish(msgCloudKinect);
        }
        if(pubRobotFrame.getNumSubscribers() > 0)
        {
            cvMatToPcl(depthMap, bgrImage, cloudWrtKinect);
            pcl::toROSMsg(cloudWrtKinect, msgCloudKinect);
            msgCloudKinect.header.frame_id = "kinect_link";
            msgCloudKinect.header.stamp = ros::Time::now();
            tf_listener->waitForTransform("base_link", "kinect_link", msgCloudKinect.header.stamp, ros::Duration(0.2));
            pcl_ros::transformPointCloud("base_link", msgCloudKinect, msgCloudRobot, *tf_listener);
            pubRobotFrame.publish(msgCloudRobot);   
        }
        if(pubRobotFrameDownsampled.getNumSubscribers() > 0)
        {
            cvMatToPcl(depthMap, bgrImage, cloudWrtKinect);
            downsampled.width = cloudWrtKinect.width/3;
            downsampled.height = cloudWrtKinect.height/3;
            downsampled.is_dense = cloudWrtKinect.is_dense;
            downsampled.points.resize(downsampled.width*downsampled.height);
            for(int i=0; i < downsampled.width; i++)
                for(int j=0; j < downsampled.height; j++)
                    downsampled.points[j*downsampled.width + i] = cloudWrtKinect.points[3*(j*cloudWrtKinect.width + i)];
            pcl::toROSMsg(downsampled, msgDownsampled);
            msgDownsampled.header.frame_id = "kinect_link";
            msgDownsampled.header.stamp = ros::Time::now();
            tf_listener->waitForTransform("base_link", "kinect_link", msgDownsampled.header.stamp, ros::Duration(0.2));
            pcl_ros::transformPointCloud("base_link", msgDownsampled, msgDownsampledRobot, *tf_listener);
            msgDownsampledRobot.header.frame_id = "base_link";
            pubRobotFrameDownsampled.publish(msgDownsampledRobot);
        }
        //cv::imshow("KINECT TEST", bgrImage);
        ros::spinOnce();
    }
    delete tf_listener;
}
