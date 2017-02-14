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
	int attemps;
	float threshold;

	cv::Mat imgBGR;
	cv::Mat imgDepth;
	cv::Point3f px;
	plane3D bestPlane;
	cv::Mat croppedDepth;
	cv::Mat croppedBRG;

	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

	xmin = 70;
	ymin = 60;
	W = 520;
	H = 390;

	attemps = 150;		// Numero de iteraciones para RANSAC
	threshold = 0.01;	// Distancia al plano en metros


	while( ros::ok() && cv::waitKey(15) != 27)
	{


		if(!cltRgbdRobot.call(srv))
		{
			std::cout << "Angle_Calc.-> Cannot get point cloud.... :(  " << std::endl;
			return -1;
		}

		JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imgBGR, imgDepth);

		cv::Rect myROI(xmin, ymin, W, H);
		croppedDepth = imgDepth(myROI);
		croppedBRG = imgBGR(myROI);

		if( i == 0)
		{
			//randomSamples = randomSample(3, croppedDepth);
			bestPlane= findPlaneConsensus(croppedDepth, threshold, attemps);

			for(int j = 0; j < croppedDepth.rows; j++)
				for (int i = 0; i < croppedDepth.cols; i++)
				{
					// Calculamos la distancia de cada uno de los puntos al plano
					px = croppedDepth.at<cv::Point3f>(j, i);
					// Camparamos si la distancia está dentro de la tolerancia
					if (bestPlane.DistanceToPoint(px, false) < threshold)
						croppedBRG.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 255, 0);
				}

			cv::imshow("plane 3D", croppedBRG);
			i = 1;
		}

		//cv::imshow("Kinect depth", consensus);
		cv::imshow("Kinect BGR", imgBGR);

		cv::imshow("Image cropped", croppedDepth);

		//cv::imshow("Image Plane", consensus);

		//cv::rectangle(imgDepth, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));
		//cv::imshow("Original depth", imgDepth);
	}


	return 0;
}
