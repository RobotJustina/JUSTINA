#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include "ros/ros.h"

#include "justina_tools/JustinaTools.h"

#include "plane3D.hpp"
#include "findPlaneRansac.hpp"
#include "objExtract.hpp"


int main(int argc, char** argv)
{
	std::cout << "INITIALIZING OBJECT ANGLE CALCULATOR BY  EDGAR-II" << std::endl;

	int attemps;
	int xmin, ymin, H, W;
	int z_number;
	float threshold;
	float z_plane;

	cv::Mat imgBGR;
	cv::Mat imgDepth;
	cv::Mat croppedDepth;
	cv::Mat croppedBRG;
	cv::Mat planeBGR;
	cv::Mat objectsBGR;
	cv::Mat objectsDepth;
	cv::Vec4f planeComp;
	cv::Point3f px;

	plane3D bestPlane;

	z_plane = 0.0;
	z_number = 0;
	xmin = 30;
	ymin = 40;
	W = 560;
	H = 430;

	// Initializing ROS node
	ros::init(argc, argv, "obj_angle_calc");
	ros::NodeHandle n;
	ros::ServiceClient cltRgbdRobot;

	point_cloud_manager::GetRgbd srv;
	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

	/* ######  Code for video recorder  ######

	int codec = CV_FOURCC('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
	double fps = 3.0;

	cv::VideoWriter plane_video("/home/edgar-ii/planeSegmentation_3.avi",
               codec,
               fps,
               cv::Size(W, H),
               true);

	cv::VideoWriter rgb_video("/home/edgar-ii/originalVideo_3.avi",
               codec,
               fps,
               cv::Size(640, 480),
               true);
	*/

	// *** Parametros de RANSAC *** //
	attemps = 50;		// Numero de iteraciones para RANSAC
	threshold = 0.03;	// Distancia al plano en metros

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

		planeBGR = croppedBRG.clone();
		objectsBGR = croppedBRG.clone();
		//boost::this_thread::sleep( boost::posix_time::milliseconds(100) );


		bestPlane = findPlaneConsensus(croppedDepth, threshold, attemps);
		//std::cout << "main---->  planeComp:  " << bestPlane.GetPlaneComp() << std::endl;
		objectsDepth = obj_extractor(bestPlane, croppedDepth);

		///* Code for coloring the plane
		if(bestPlane.GetNormal() != cv::Point3f(1.0, 1.0, 1.0) )
		{
			for(int j = 0; j < planeBGR.rows; j++)
				for (int i = 0; i < planeBGR.cols; i++)
				{
					// Calculamos la distancia de cada uno de los puntos al plano
					px = croppedDepth.at<cv::Point3f>(j, i);
					// Camparamos si la distancia está dentro de la tolerancia
					if (bestPlane.DistanceToPoint(px, false) < threshold)
					{
						planeBGR.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 255, 0);
						z_number++;
						z_plane = (z_plane + px.z);
					}
				}
		}
		else
			std::cout << "I can't found the plane....   :( " << std::endl;
		// */

		for(int j = 0; j < objectsBGR.rows; j++)
				for (int i = 0; i < objectsBGR.cols; i++)
				{
					// Calculamos la distancia de cada uno de los puntos al plano
					px = objectsDepth.at<cv::Point3f>(j, i);
					// Camparamos si la distancia está dentro de la tolerancia
					if ( px != cv::Point3f(0.0, 0.0, 0.0) )
						objectsBGR.at<cv::Vec3b>(j, i) = cv::Vec3b(170, 10, 0);
				}

		//std::cout << "   Z_prom:  " << z_plane/z_number << std::endl;
		std::cout << "--------------------------------------" << std::endl;

		cv::rectangle(imgBGR, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));
		//cv::rectangle(imgDepth, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));

		//cv::imshow("Original RGB", imgBGR);
		//cv::imshow("Original Depth", imgDepth);
		//cv::imshow("Cropped Depth", croppedDepth);
		//cv::imshow("Cropped RGB", croppedBRG);
		cv::imshow("plane 3D", planeBGR);
		cv::imshow("Objects Point Cloud", objectsDepth);
		cv::imshow("objects", objectsBGR);


		/* ######  Code for video recorder  ######
		plane_video.write(croppedBRG);
		rgb_video.write(imgBGR);
		*/
	}


	return 0;
}
