#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"

#include "justina_tools/JustinaTools.h"

#include "plane3D.hpp"
#include "findPlaneRansac.hpp"


int main(int argc, char** argv)
{
	std::cout << "INITIALIZING OBJECT ANGLE CALCULATOR BY  EDGAR-II" << std::endl;
	// Initializing ROS node
	ros::init(argc, argv, "obj_angle_calc");
	ros::NodeHandle n;

	ros::ServiceClient cltRgbdRobot;

	point_cloud_manager::GetRgbd srv;


	//cv::namedWindow("Kinect depth");
	//cv::namedWindow("Kinect BGR");

	int xmin, ymin, H, W;
	int i = 0;
	cv::Mat imgBGR;
	cv::Mat imgDepth;
	cv::Mat randomSamples;
	cv::Mat consensus;
	cv::Mat croppedImage;

	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

	xmin = 70;
	ymin = 60;
	W = 520;
	H = 390;


	while( ros::ok() && cv::waitKey(15) != 27)
	{


		if(!cltRgbdRobot.call(srv))
		{
			std::cout << "Angle_Calc.-> Cannot get point cloud.... :(  " << std::endl;
			return -1;
		}

		JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imgBGR, imgDepth);

		cv::Rect myROI(xmin, ymin, W, H);
		croppedImage = imgDepth(myROI);

		if( i == 0)
		{
			randomSamples = randomSample(3, croppedImage);
			std::cout << "croppedImage_C:   " << croppedImage.cols << std::endl;
			std::cout << "croppedImage_R:   " << croppedImage.rows << std::endl;
			consensus = findPlaneConsensus(randomSamples, croppedImage, 0.001);
			i = 1;
		}

		//cv::imshow("Kinect depth", consensus);
		//cv::imshow("Kinect BGR", imgBGR);

		cv::imshow("Image cropped", croppedImage);
		cv::imshow("Image Plane", consensus);

		//std::cout << "Sample:   " << randomSamples << std::endl;

		//cv::rectangle(imgDepth, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));
		//cv::imshow("Original depth", imgDepth);
	}


	return 0;
}
