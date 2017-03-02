#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

#include "justina_tools/JustinaTools.h"

#include "plane3D.hpp"
#include "findPlaneRansac.hpp"
#include "objExtract.hpp"


int main(int argc, char** argv)
{
	std::cout << "INITIALIZING OBJECT ANGLE CALCULATOR BY  EDGAR-II" << std::endl;

	std::vector<float> centroid_coord;
	std::vector<cv::Point3f> principal_axis;

	int xmin, ymin, H, W;
	int x_min, x_max;
	int y_min, y_max;
	int attemps;
	int points_obj;
	float threshold;
	float x_obj;
	float y_obj;
	float z_obj;
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

	xmin = 190;
	ymin = 120;

	x_min = 1000;
	y_min = 1000;
	x_max = 0;
	y_max = 0;

	x_obj = 0.0;
	y_obj = 0.0;
	z_obj = 0.0;
	h_table = 0.0;

	points_obj = 0;

	W = 260;
	H = 320;

	// Initializing ROS node
	ros::init(argc, argv, "obj_angle_calc");
	ros::NodeHandle n;
	ros::ServiceClient cltRgbdRobot;
	ros::Publisher marker_pub;

	geometry_msgs::Point p;

	point_cloud_manager::GetRgbd srv;
	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	/*
	// ######  Code for video recorder  ######
	int codec = CV_FOURCC('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
	double fps = 4.0;

	cv::VideoWriter plane_video("/home/edgar-ii/planeSegmentation_1.avi",
               codec,
               fps,
               cv::Size(W, H),
               true);

	cv::VideoWriter depth_video("/home/edgar-ii/originalVideo_1.avi",
               codec,
               fps,
               cv::Size(W, H),
               true);

		cv::VideoWriter object_video("/home/edgar-ii/object_1.avi",
               codec,
               fps,
               cv::Size(W, H),
               true);
	*/


	// *** Parametros de RANSAC *** //
	attemps = 70;		// Numero de iteraciones para RANSAC
	threshold = 0.03;	// Distancia al plano en metros

	while( ros::ok() && cv::waitKey(15) != 27)
	{
		x_min = 1000;
		y_min = 1000;

		x_max = 0;
		y_max = 0;

		x_obj = 0.0;
		y_obj = 0.0;
		z_obj = 0.0;

		points_obj = 0;


		if(!cltRgbdRobot.call(srv))
		{
			std::cout << "Angle_Calc.-> Cannot get point cloud.... :(  " << std::endl;
			return -1;
		}

		JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imgBGR, imgDepth);
		visualization_msgs::Marker centroid, axis_list;

		centroid.header.frame_id = "map";
		axis_list.header.frame_id = "map";
		centroid.header.stamp = ros::Time::now();
		axis_list.header.stamp = ros::Time::now();
		centroid.ns = "centroid";
		axis_list.ns = "principal axis";
		centroid.pose.orientation.w = 1.0;
		axis_list.pose.orientation.w = 1.0;

		centroid.id = 0;
		axis_list.id = 1;

		centroid.type = visualization_msgs::Marker::SPHERE;
		axis_list.type = visualization_msgs::Marker::LINE_LIST;

		// POINTS markers use x and y scale for width/height respectively
		centroid.scale.x = 0.035;
		centroid.scale.y = 0.035;
		centroid.scale.z = 0.035;

		axis_list.scale.x = 0.03;
		axis_list.scale.y = 0.03;
		axis_list.scale.z = 0.03;

		centroid.color.b = 1.0f;
		centroid.color.a = 1.0;

		axis_list.color.r = 1.0f;
		axis_list.color.a = 1.0;


		cv::Rect myROI(xmin, ymin, W, H);
		croppedDepth = imgDepth(myROI);
		croppedBRG = imgBGR(myROI);

		planeBGR = croppedBRG.clone();
		objectsBGR = croppedBRG.clone();
		//boost::this_thread::sleep( boost::posix_time::milliseconds(100) );

		// ##### Find best fit model to point cloud
		// Return plane with Normal = (1, 1, 1) is didn't find plane
		bestPlane = findPlaneConsensus(croppedDepth, threshold, attemps);

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
			objectsDepth = obj_extractor(bestPlane, croppedDepth);
			h_table = z_plane(bestPlane, croppedDepth);
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
			centroid_coord = calculate_centroid(objectsDepth, h_table);
			centroid.pose.position.x = p.x = centroid_coord[0];
			centroid.pose.position.y = p.y = centroid_coord[1];
			centroid.pose.position.z = p.z = centroid_coord[2];

			std::cout << "   Z_prom:  " << z_plane(bestPlane, croppedDepth)  << std::endl;
			principal_axis = PCA(objectsDepth, centroid_coord);
			std::cout << "   axis[0]:  " << principal_axis[0] << "  -  norm:  " << cv::norm(principal_axis[0]) << std::endl;
			std::cout << "   axis[1]:  " << principal_axis[1] << "  -  norm:  " << cv::norm(principal_axis[1]) << std::endl;
			std::cout << "   axis[2]:  " << principal_axis[2] << "  -  norm:  " << cv::norm(principal_axis[2]) << std::endl;

			axis_list.points.push_back(p);
			//This is the bigger axis
			p.x = p.x + principal_axis[0].x;
			p.y = p.y + principal_axis[0].y;
			p.z = p.z + principal_axis[0].z;
			axis_list.points.push_back(p);

			p.x = centroid_coord[0];
			p.y = centroid_coord[1];
			p.z = centroid_coord[2];
			axis_list.points.push_back(p);
			p.x = p.x + principal_axis[1].x;
			p.y = p.y + principal_axis[1].y;
			p.z = p.z + principal_axis[1].z;
			axis_list.points.push_back(p);

			p.x = centroid_coord[0];
			p.y = centroid_coord[1];
			p.z = centroid_coord[2];
			axis_list.points.push_back(p);
			p.x = p.x + principal_axis[2].x;
			p.y = p.y + principal_axis[2].y;
			p.z = p.z + principal_axis[2].z;
			axis_list.points.push_back(p);
		}

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
		cv::imshow("objects", objectsBGR);

		marker_pub.publish(centroid);
		marker_pub.publish(axis_list);
		/*
		//######  Code for video recorder  ######
		plane_video.write(planeBGR);
		depth_video.write(objectsDepth*255.0);
		object_video.write(objectsBGR);
		*/
	}


	return 0;
}
