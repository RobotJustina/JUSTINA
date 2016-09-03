#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"

void cvMat2Pcl(cv::Mat& matColor, cv::Mat& matDepth, pcl::PointCloud<pcl::PointXYZRGBA>& pclFrame)
{
    if(matColor.rows != matDepth.rows || matColor.cols != matDepth.cols)
        return;
    if(matColor.channels() != 3)
    {
        std::cout << "Color image should have 3 channels. :'(" << std::endl;
        return;
    }
    
    pclFrame.width = matColor.cols;
    pclFrame.height = matColor.rows;
    pclFrame.points.resize(pclFrame.width*pclFrame.height);
    pclFrame.is_dense = false;

    for(size_t i=0; i < matDepth.rows; i++)
        for(size_t j=0; j < matDepth.cols; j++)
        {
            cv::Vec3f p = matDepth.at<cv::Vec3f>(i,j);
            int idx = i*pclFrame.width + j;
            pclFrame.points[idx].x = p[0];
            pclFrame.points[idx].y = -p[1];
            pclFrame.points[idx].z = p[2];
            pclFrame.points[idx].b = matColor.data[3*idx + 0];
            pclFrame.points[idx].g = matColor.data[3*idx + 1];
            pclFrame.points[idx].r = matColor.data[3*idx + 2];
            pclFrame.points[idx].a = 1;
        }   
}

void extractPlane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclInput, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclColored,
                  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPlane, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclNotPlane)
{
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(pclInput);
    seg.segment(*inliers, *coeff);

    for(size_t i=0; i < inliers->indices.size(); i++)
    {
        pclColored->points[inliers->indices[i]].r = 255;
        pclColored->points[inliers->indices[i]].g = 0;
        pclColored->points[inliers->indices[i]].b = 0;
    }
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING COLOR_RECOGNIZER..." << std::endl;
    ros::init(argc, argv, "color_recog");
    ros::NodeHandle n;
    ros::Rate loop(30);

    std::cout << "ColorRecognizer.->Triying to initialize kinect sensor... " << std::endl;
    cv::VideoCapture capture(CV_CAP_OPENNI);
    if(!capture.isOpened())
    {
        std::cout << "ColorRecognizer.->Cannot open kinect :'(" << std::endl;
        return 1;
    }
    capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON);
    std::cout << "ColorRecognizer.->Kinect sensor started :D" << std::endl;

    cv::Mat matDepth;
    cv::Mat matColor;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclFrame(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclNotPlane(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPlane(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::visualization::CloudViewer viewer("Original");
    while(ros::ok() && cv::waitKey(10) != 27 && !viewer.wasStopped())
    {
        if(!capture.grab())
        {
            loop.sleep();
            ros::spinOnce();
            continue;
        }
        capture.retrieve(matDepth, CV_CAP_OPENNI_POINT_CLOUD_MAP);
        capture.retrieve(matColor, CV_CAP_OPENNI_BGR_IMAGE);

        cvMat2Pcl(matColor, matDepth, *pclFrame);
        extractPlane(pclFrame,  pclFrame, pclPlane, pclNotPlane);
        
        cv::imshow("Original", matColor);
        viewer.showCloud(pclFrame);
        ros::spinOnce();
    }
}
