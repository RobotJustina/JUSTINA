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

	//plane3D p(cv::Point3f(0.0, 0.0, 0.0), cv::Point3f(0.0, 1.0, 0.0), cv::Point3f(1.0, 0.0, 0.0) );

	//cv::namedWindow("Kinect depth");
	//cv::namedWindow("Kinect BGR");

	int xmin, ymin, H, W;
	cv::Mat imgBGR;
	cv::Mat imgDepth;
	cv::Mat randomSamples;
	cv::Mat consensus;
	cv::Mat croppedImage;

	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

	xmin = 50;
	ymin = 40;
	W = 540;
	H = 410;
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

		randomSamples = randomSample(3, croppedImage);
		consensus = findPlaneConsensus(randomSamples, croppedImage, 0.005);

		//cv::imshow("Kinect depth", consensus);
		//cv::imshow("Kinect BGR", imgBGR);
		cv::imshow("Image cropped", croppedImage);
		//cv::rectangle(imgDepth, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));
		//cv::imshow("Original depth", imgDepth);
	}

	return 0;
}
