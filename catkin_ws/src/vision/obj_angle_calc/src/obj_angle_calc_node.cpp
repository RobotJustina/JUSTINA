#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>
#include "justina_tools/JustinaTools.h"
#include "ros/ros.h"

#include "plane3D.hpp"
#include "objExtract.hpp"
#include "findPlaneRansac.hpp"
#include "vision_msgs/DetectObjects.h"

ros::ServiceClient cltRgbdRobot;
point_cloud_manager::GetRgbd srv;



bool callbackPCAobject(vision_msgs::DetectObjects::Request &req,
					vision_msgs::DetectObjects::Response &resp)
{
	std::cout << "Calling service to calculate PCA....." << std::endl;

	vision_msgs::VisionObject objectDetected;

	std::vector<float> centroid_coord;
	std::vector<cv::Point3f> principal_axis_calculated;

	int xmin, ymin, H, W;
	int x_min, x_max;
	int y_min, y_max;
	int attemps;
	int points_obj;
	float x_obj, y_obj, z_obj;
	float threshold;
	float h_table;

	cv::Mat imgBGR;
	cv::Mat imgDepth;
	cv::Mat planeBGR;
	cv::Mat objectsBGR;
	cv::Mat objectsDepth;
	cv::Mat croppedDepth;
	cv::Mat croppedBRG;

	cv::Vec4f planeComp;
	cv::Point3f px;

	plane3D bestPlane;


	// *** Parametros de RANSAC *** //
	attemps = 70;		// Numero de iteraciones para RANSAC
	threshold = 0.02;	// Distancia al plano en metros

	x_min = 1000;
	y_min = 1000;

	x_max = 0;
	y_max = 0;

	xmin = 190;
	ymin = 120;

	x_obj = 0.0;
	y_obj = 0.0;
	z_obj = 0.0;
	h_table = 0.0;

	points_obj = 0;

	W = 260;
	H = 320;

	centroid_coord.push_back(0.0);
	centroid_coord.push_back(0.0);
	centroid_coord.push_back(0.0);



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


	// ##### Find best fit model to point cloud
	// Return plane with Normal = (1, 1, 1) is didn't find plane
	bestPlane = FindPlaneRANSAC(croppedDepth, threshold, attemps);

	// /* Code for coloring the plane
	if(bestPlane.GetNormal() != cv::Point3f(1.0, 1.0, 1.0) )
	{
		for(int j = 0; j < planeBGR.rows; j++)
			for (int i = 0; i < planeBGR.cols; i++)
			{
				// Calculamos la distancia de cada uno de los puntos al plano
				px = croppedDepth.at<cv::Point3f>(j, i);
				// Camparamos si la distancia está dentro de la tolerancia
				if (bestPlane.DistanceToPoint(px, false) < threshold)
					planeBGR.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 255, 0);
			}

		// ##### Return the point cloud of objects cropped
		objectsDepth = ExtractObj(bestPlane, croppedDepth);
		h_table = CalculateZPlane(bestPlane, croppedDepth);
	}
	else
	{
		std::cout << "I can't found the plane....   :( " << std::endl;
		objectsDepth = cv::Mat(50, 50, CV_8UC3);
	}
	// */

	/* // This aply when objectsDepth size == objectsBGR size
	//  Code for coloring objects
	for(int j = 0; j < objectsDepth.rows; j++)
		for (int i = 0; i < objectsDepth.cols; i++)
			if ( objectsDepth.at<cv::Point3f>(j, i) != cv::Point3f(0.0, 0.0, 0.0) )
				objectsBGR.at<cv::Vec3b>(j, i) = cv::Vec3b(150, 0, 0);
	*/

	/*
	for(int i = 0; i < objectsDepth.rows; i++)
		for(int j = 0; j < objectsDepth.cols; j++)
		{
			if( objectsDepth.at<cv::Point3f>(i, j) != cv::Point3f(0.0, 0.0, 0.0) )
			{
				y_min = (i < y_min) ? i : y_min;
				x_min = (j < x_min) ? j : x_min;
				y_max = (i > y_max) ? i : y_max;
				x_max = (j > x_max) ? j : x_max;
			}
		}
	*/

	// Search the centroid of object PointCloud
	if(objectsDepth.size() != cv::Size(50, 50) )
	{
		centroid_coord = CalculateCentroid(objectsDepth, h_table);

		//This is for response
		resp.recog_objects.push_back(objectDetected);
		resp.recog_objects[0].pose.position.x = centroid_coord[0];
		resp.recog_objects[0].pose.position.y = centroid_coord[1];
		resp.recog_objects[0].pose.position.z = centroid_coord[2];

		std::cout << "   Z_prom:  " << h_table  << std::endl;
		principal_axis_calculated = CalculatePCA(objectsDepth, centroid_coord);
		//std::cout << "   axis[0]:  " << principal_axis_calculated[0] << "  -  norm:  " << cv::norm(principal_axis_calculated[0]) << std::endl;
		//std::cout << "   axis[1]:  " << principal_axis_calculated[1] << "  -  norm:  " << cv::norm(principal_axis_calculated[1]) << std::endl;
		//std::cout << "   axis[2]:  " << principal_axis_calculated[2] << "  -  norm:  " << cv::norm(principal_axis_calculated[2]) << std::endl;


		geometry_msgs::Vector3 q1;
		resp.recog_objects[0].principal_axis.push_back(q1);
		resp.recog_objects[0].principal_axis.push_back(q1);
		resp.recog_objects[0].principal_axis.push_back(q1);

		//This is the bigger axis
		resp.recog_objects[0].principal_axis[0].x = float(principal_axis_calculated[0].x);
		resp.recog_objects[0].principal_axis[0].y = float(principal_axis_calculated[0].y);
		resp.recog_objects[0].principal_axis[0].z = float(principal_axis_calculated[0].z);

		resp.recog_objects[0].principal_axis[1].x = float(principal_axis_calculated[1].x);
		resp.recog_objects[0].principal_axis[1].y = float(principal_axis_calculated[1].y);
		resp.recog_objects[0].principal_axis[1].z = float(principal_axis_calculated[1].z);

		resp.recog_objects[0].principal_axis[2].x = float(principal_axis_calculated[2].x);
		resp.recog_objects[0].principal_axis[2].y = float(principal_axis_calculated[2].y);
		resp.recog_objects[0].principal_axis[2].z = float(principal_axis_calculated[2].z);
	}
	else
		std::cout << "    I can't find a object on the table..... :(" << std::endl;


	std::cout << "    x_obj: " << centroid_coord[0] << " - y_obj: " << centroid_coord[1] << " - z_obj: " << centroid_coord[2] << std::endl;
	std::cout << "--------------------------------------" << std::endl;

	cv::rectangle(imgBGR, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));
	cv::rectangle(imgDepth, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));
	//cv::rectangle(objectsBGR, cv::Point(x_min, y_min), cv::Point(x_max, y_max), cv::Scalar(0, 255, 0));
	//cv::rectangle(objectsDepth, cv::Point(x_min, y_min), cv::Point(x_max, y_max), cv::Scalar(0, 255, 0));

	objectsDepth.convertTo(objectsDepth, CV_8UC3);
	cv::imshow("Original RGB", imgBGR);
	//cv::imshow("Original Depth", imgDepth);
	//cv::imshow("Cropped Depth", croppedDepth);
	//cv::imshow("Cropped RGB", croppedBRG);
	cv::imshow("plane 3D", planeBGR);
	cv::imshow("Objects Point Cloud", objectsDepth*255.0 );
	//cv::imshow("objects", objectsBGR);


	/*
	//######  Code for video recorder  ######
	plane_video.write(planeBGR);
	depth_video.write(objectsDepth*255.0);
	object_video.write(objectsBGR);
	*/

	return true;
}


int main(int argc, char** argv)
{
	std::cout << "INITIALIZING OBJECT ANGLE CALCULATOR BY  EDGAR-II" << std::endl;


	// Initializing ROS node
	ros::init(argc, argv, "obj_angle_calc");
	ros::NodeHandle n;
	ros::ServiceServer srvPCAobject;
	ros::Publisher marker_pub;

	srvPCAobject = n.advertiseService("detect_object/PCA_calculator", callbackPCAobject);
	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");


	ros::Rate loop(10);

	while( ros::ok() && cv::waitKey(15) != 27)
	{
		// ROS
		ros::spinOnce();
		loop.sleep();

		if( cv::waitKey(5) == 'q' )
			break;
	}
	cv::destroyAllWindows();
	return 0;
}
