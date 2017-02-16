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
	float z_plane;
	float threshold;

	cv::Mat objectsPC;
	cv::Point3f px;

	z_plane = 0.0;
	z_numbers = 0;
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
			if( px.z < (z_plane - 0.02) || px.x < 0.30 || px.x > 0.50)
				objectsPC.at<cv::Point3f>(i, j) = cv::Point3f(0.0, 0.0, 0.0);
		}


	return objectsPC;
}

