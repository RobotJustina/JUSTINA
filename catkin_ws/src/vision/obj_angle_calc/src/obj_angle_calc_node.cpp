#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include "ros/ros.h"

#include "justina_tools/JustinaTools.h"

#include "plane3D.hpp"
#include "findPlaneRansac.hpp"


int main(int argc, char** argv)
{
	std::cout << "INITIALIZING OBJECT ANGLE CALCULATOR BY  EDGAR-II" << std::endl;

	int attemps;
	int xmin, ymin, H, W;
	float threshold;

	cv::Mat imgBGR;
	cv::Mat imgDepth;
	cv::Mat croppedDepth;
	cv::Mat croppedBRG;
	cv::Vec4f planeComp;
	cv::Point3f px;

	plane3D bestPlane;

	// Initializing ROS node
	ros::init(argc, argv, "obj_angle_calc");
	ros::NodeHandle n;
	ros::ServiceClient cltRgbdRobot;

	point_cloud_manager::GetRgbd srv;
	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

	xmin = 30;
	ymin = 40;
	W = 560;
	H = 430;

	// *** Parametros de RANSAC *** //
	attemps = 50;		// Numero de iteraciones para RANSAC
	threshold = 0.02;	// Distancia al plano en metros

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
		croppedBRG = imgBGR.clone();
		croppedBRG = croppedBRG(myROI);
		//boost::this_thread::sleep( boost::posix_time::milliseconds(100) );


		bestPlane = findPlaneConsensus(croppedDepth, threshold, attemps);
		std::cout << "planeComp:  " << bestPlane.GetPlaneComp() << std::endl;
		if(bestPlane.GetNormal() != cv::Point3f(1.0, 1.0, 1.0) )
		{
			for(int j = 0; j < croppedDepth.rows; j++)
				for (int i = 0; i < croppedDepth.cols; i++)
				{
					// Calculamos la distancia de cada uno de los puntos al plano
					px = croppedDepth.at<cv::Point3f>(j, i);
					// Camparamos si la distancia está dentro de la tolerancia
					if (bestPlane.DistanceToPoint(px, false) < threshold)
						croppedBRG.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 255, 0);
				}
		}
		else
			std::cout << "I can't found the plane....   :( " << std::endl;

		std::cout << "--------------------------------------" << std::endl;
		cv::imshow("plane 3D", croppedBRG);

		//cv::imshow("Kinect BGR", imgBGR);
		//cv::imshow("Image cropped", croppedDepth);

		cv::rectangle(imgBGR, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));
		//cv::rectangle(imgDepth, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));
		cv::imshow("Original RGB", imgBGR);
		//cv::imshow("Original depth", imgDepth);
	}


	return 0;
}
