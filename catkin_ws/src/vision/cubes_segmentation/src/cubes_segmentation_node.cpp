#include <iostream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "justina_tools/JustinaTools.h"
#include "geometry_msgs/Point.h"
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

ros::NodeHandle* node;
ros::ServiceClient cltRgbdRobot;


bool GetImagesFromJustina( cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "CubesSegmentation.->Cannot get point cloud" << std::endl;
        return false;
    }
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}


void callbackCubeSeg(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	float minX = 0.10, maxX = 1.0;
	float minY = -0.3, maxY = 0.3;
	float minZ = 0.7, maxZ = 2.0;

	cv::Vec3f centroid (0.0, 0.0, 0.0); 

	cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat imageHSV;
    cv::Mat maskHSV;


    
    GetImagesFromJustina(bgrImg,xyzCloud);

    cv::cvtColor(bgrImg,imageHSV,CV_BGR2HSV);
        
    //inRange(imageHSV,Scalar(0,70,50), Scalar(0,255,255),maskHSV);
    
    inRange(imageHSV,Scalar(172,150,0), Scalar(179,254,255),maskHSV);//color rojo

    cv::Mat maskXYZ;
	cv::inRange(xyzCloud,cv::Scalar(minX, minY,minZ),cv::Scalar(maxX,maxY,maxZ),maskXYZ);

	cv::Mat mask;
	maskXYZ.copyTo(mask,maskHSV);
	cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(1.5, 1.5));
	cv::morphologyEx(mask,mask,cv::MORPH_ERODE,kernel,cv::Point(-1,-1),1);
	cv::morphologyEx(mask,mask,cv::MORPH_DILATE,kernel,cv::Point(-1,-1),7);

	
	cv::Point imgCentroid(0,0);
	int numPoints = 0;
	for (int i = 0; i < mask.rows; ++i)
	{
		for (int j = 0; j < mask.cols; ++j)
		{
			if (mask.at<uchar>(i,j)>0)
			{
				centroid += xyzCloud.at<cv::Vec3f>(i,j);
				imgCentroid += cv::Point(j,i);
				++numPoints;
			}
		}
	}
	if (numPoints == 0)
	{
		std::cout << "CubesSegmentation.->Cannot get centroid " << std::endl;
		//return centroid;
	}
	centroid /= numPoints;
	imgCentroid /= numPoints;

	std::cout << "centroid: " << centroid << std::endl;
	
	cv::Mat maskedImage;
	bgrImg.copyTo(maskedImage,mask);
	cv::circle(maskedImage,imgCentroid,5,cv::Scalar(0,255,0),-1);
    
    imshow("centroid", maskedImage);

}


int main(int argc, char** argv)
{
	
    std::cout << "Initializing Cubes Segmentation by Hugo..." << std::endl;
    ros::init(argc, argv, "cubes_segmentation");
    ros::NodeHandle n;
    node = &n;
    

    ros::Subscriber subStarSegment = n.subscribe("/vision/cubes_segmentation/start_segment", 1, callbackCubeSeg);
    cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    
    ros::Rate loop(30);
    
    std::cout << "CubesSegmentation.->Running..." << std::endl;
    
    while(ros::ok() && cv::waitKey(1) != 'q')
    {
        ros::spinOnce();
        loop.sleep();
    }

    subStarSegment.shutdown();
    cv::destroyAllWindows();   
}


