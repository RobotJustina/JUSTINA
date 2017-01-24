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

	cv::namedWindow("Kinect depth");
	cv::namedWindow("Kinect BGR");

	cv::Mat imgBGR;
	cv::Mat imgDepth;
	cv::Mat randomSamples;
	cv::Mat consensus;
	cv::Mat croppedImage;

	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

	while( ros::ok() && cv::waitKey(15) != 27)
	{
		if(!cltRgbdRobot.call(srv))
		{
			std::cout << "Angle_Calc.-> Cannot get point cloud.... :(  " << std::endl;
			return -1;
		}

		JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imgBGR, imgDepth);

		cv::Rect myROI(25, 35, 520, 430);
		croppedImage = imgDepth(myROI);

		randomSamples = randomSample(3, croppedImage);
		//consensus = findPlaneConsensus(randomSamples, croppedImage, 0.005);

		//cv::imshow("Kinect depth", consensus);
		//cv::imshow("Kinect BGR", imgBGR);
		cv::imshow("Image cropped", croppedImage);
	}

	return 0;
}
