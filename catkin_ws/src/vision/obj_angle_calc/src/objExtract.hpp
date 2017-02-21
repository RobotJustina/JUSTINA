#pragma once
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "plane3D.hpp"

cv::Mat obj_extractor(plane3D plane, cv::Mat points);



cv::Mat obj_extractor(plane3D plane, cv::Mat points)
{
	int z_numbers;
	int x_min, x_max;
	int y_min, y_max;
	float z_plane;
	float threshold;

	cv::Mat objectsPC;
	cv::Point3f px;


	z_numbers = 0;
	x_min = 1000;
	x_max = 0;
	y_min = 1000;
	y_max = 0;
	z_plane = 0.0;
	threshold = 0.03;

	objectsPC = points.clone();

	//std::cout << "obj_extr--->  planeComp:  " << plane.GetPlaneComp() << std::endl;
	//std::cout << "obj_extr--->  points_size: " << points.size() << std::endl;

	// Delete the points on the plane ///
	for(int i = 0; i < points.rows; i++)
		for(int j = 0; j < points.cols; j++)
		{
			px = points.at<cv::Point3f>(i, j);
			if( plane.DistanceToPoint(px, false) < threshold)
			{
				z_numbers++;
				z_plane += px.z;
				objectsPC.at<cv::Point3f>(i, j) = cv::Point3f(0.0, 0.0, 0.0);
			}
		}

	z_plane = z_plane / z_numbers;
	std::cout << "   Z_prom:  " << z_plane  << std::endl;


	for(int i = 0; i < points.rows; i++)
		for(int j = 0; j < points.cols; j++)
		{
			px = points.at<cv::Point3f>(i, j);
			if( px.z < (z_plane - 0.02) || px.x < 0.30 || px.x > 0.80)
				objectsPC.at<cv::Point3f>(i, j) = cv::Point3f(0.0, 0.0, 0.0);
		}

		// Cropp the object image
	for(int i = 0; i < objectsPC.rows; i++)
		for(int j = 0; j < objectsPC.cols; j++)
		{
			if( objectsPC.at<cv::Point3f>(i, j) != cv::Point3f(0.0, 0.0, 0.0) )
			{
				y_min = (i < y_min) ? i : y_min;
				x_min = (j < x_min) ? j : x_min;
				y_max = (i > y_max) ? i : y_max;
				x_max = (j > x_max) ? j : x_max;
			}
		}

	x_min -= 5;
	y_min -= 5;
	x_max += 5;
	y_max += 5;
	cv::rectangle(objectsPC, cv::Point(x_min, y_min), cv::Point(x_max, y_max), cv::Scalar(0, 255, 0));

	//cv::Rect myCrop(x_min, y_min, x_max -x_min, y_max - y_min);
	//objectsPC = objectsPC(myCrop);
	return objectsPC;
}

