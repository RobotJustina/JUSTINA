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


/*
	std::cout << "Kinect manager.-> Triying to initialize kinect sensor...." << std::endl;
	cv::VideoCapture capture(CV_CAP_OPENNI);
	if(!capture.isOpened())
	{
		std::cout << "Kinect manager.-> Cannot open kinect..... :´(" << std::endl;
		return -1;
	}
	capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON);
	std::cout << "Kinect manager.-> Kinect sensor started :D" << std::endl;
	std::cout << "Kinect manager.-> Sistem ready to use" << std::endl;
*/

	//cv::namedWindow("Kinect depth");
	//cv::namedWindow("Kinect BGR");

	int xmin, ymin, H, W;
	cv::Mat imgBGR;
	cv::Mat imgDepth;
	cv::Mat randomSamples;
	cv::Mat consensus;
	cv::Mat croppedImage;
	cv::Point3d p0, p1, p2;

	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

	xmin = 70;
	ymin = 60;
	W = 520;
	H = 390;


	while( ros::ok() && cv::waitKey(15) != 27)
	{
/*
		capture.grab();
		capture.retrieve(imgDepth, CV_CAP_OPENNI_POINT_CLOUD_MAP);
		capture.retrieve(imgBGR, CV_CAP_OPENNI_BGR_IMAGE);
*/

		//Comentar estas lineas para llamar al servicio de la nube de puntos
		//respecto del robot


		if(!cltRgbdRobot.call(srv))
		{
			std::cout << "Angle_Calc.-> Cannot get point cloud.... :(  " << std::endl;
			return -1;
		}

		JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imgBGR, imgDepth);

		cv::Rect myROI(xmin, ymin, W, H);
		croppedImage = imgDepth(myROI);

		p0 = croppedImage.at<cv::Point3d>(300,220);
		p1 = croppedImage.at<cv::Point3d>(350,250);
		p2 = croppedImage.at<cv::Point3d>(250,270);

		randomSamples = randomSample(3, croppedImage);
		consensus = findPlaneConsensus(randomSamples, croppedImage, 0.0005);

		//cv::imshow("Kinect depth", consensus);
		//cv::imshow("Kinect BGR", imgBGR);
		cv::imshow("Image cropped", croppedImage);

		//std::cout << "Sample:   " << randomSamples << std::endl;

		//std::cout << "p_0 " << croppedImage.at<cv::Point3d>(300,120) << std::endl;
		//std::cout << "p_1 " << croppedImage.at<cv::Point3d>(350,150) << std::endl;
		//std::cout << "p_2 " << croppedImage.at<cv::Point3d>(250,170) << std::endl;
		//std::cout << "--------------------------------------" << std::endl;

		//cv::rectangle(imgDepth, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));
		//cv::imshow("Original depth", imgDepth);
	}


	return 0;
}
