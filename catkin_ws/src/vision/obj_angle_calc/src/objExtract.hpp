#pragma once
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "plane3D.hpp"


float z_plane(plane3D plane, cv::Mat points);

cv::Mat obj_extractor(plane3D plane, cv::Mat points);

std::vector<float> calculate_centroid(cv::Mat objectsDepth);

std::vector<cv::Point3f> PCA(cv::Mat object, std::vector<float> centroid);



float z_plane(plane3D plane, cv::Mat points)
{
	int z_numbers;
	float z;

	cv::Point3f px;

	z_numbers = 0;
	z = 0.0;

	// Delete the points on the plane ///
	for(int i = 0; i < points.rows; i++)
		for(int j = 0; j < points.cols; j++)
		{
			px = points.at<cv::Point3f>(i, j);
			if( plane.DistanceToPoint(px, false) < 0.2)
			{
				z_numbers++;
				z += px.z;
			}
		}

	return z / z_numbers;;
}



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

	// Delete points under plane
	for(int i = 0; i < points.rows; i++)
		for(int j = 0; j < points.cols; j++)
		{
			px = points.at<cv::Point3f>(i, j);
			if( px.z < (z_plane - 0.01) || px.x < 0.40 || px.x > 0.80)
				objectsPC.at<cv::Point3f>(i, j) = cv::Point3f(0.0, 0.0, 0.0);
		}


	//  ### Code for crop objects from image

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

	cv::rectangle(objectsPC, cv::Point(x_min, y_min), cv::Point(x_max, y_max), cv::Scalar(0, 255, 0));

	if(y_min == 1000 && x_min == 1000 && x_max == 0 && y_max == 0)
		objectsPC = cv::Mat(50, 50, CV_8UC3);
	else
	{
		cv::Rect myCrop(x_min, y_min, x_max -x_min, y_max - y_min);
		objectsPC = objectsPC(myCrop);
	}

	return objectsPC;
}


std::vector<float> calculate_centroid(cv::Mat objectsDepth, float z_plane)
{
	std::vector<float> centroid;
	cv::Point3f px;

	int points_obj;

	float x_obj;
	float y_obj;
	float z_obj;

	x_obj = 0.0;
	y_obj = 0.0;
	z_obj = 0.0;

	points_obj = 0;

	// Search the centroid of object PointCloud
	for(int j = 0; j < objectsDepth.rows; j++)
		for (int i = 0; i < objectsDepth.cols; i++)
		{
			px = objectsDepth.at<cv::Point3f>(j,i);
			if ( px != cv::Point3f(0.0, 0.0, 0.0) && px != cv::Point3f(0, 255, 0))
			{
				x_obj += px.x;
				y_obj += px.y;
				z_obj += px.z;
				points_obj++;
				//std::cout << "x_obj: " << px.x << " - y_obj: " << px.y << " - z_obj: " << px.z << std::endl;
			}
		}

	x_obj = x_obj/points_obj;
	centroid.push_back(x_obj);

	y_obj = y_obj/points_obj;
	centroid.push_back(y_obj);

	z_obj = (z_obj/points_obj + z_plane + 0.01) / 2 ;
	centroid.push_back(z_obj);

	return centroid;
}


std::vector<cv::Point3f> PCA(cv::Mat object, std::vector<float> centroid)
{
	int n;
	float var_x;
	float var_y;
	float var_z;
	float cov_xy;
	float cov_xz;
	float cov_yz;
	std::vector<cv::Point3f> principal_comp;

	cv::Point3f px;
	cv::Point3f axis_1, axis_2, axis_3;

	Eigen::Matrix3f cov_matrix(3,3);
	//Eigen::Vector3f eivals;

	n = 0;
	var_x = 0.0;
	var_y = 0.0;
	var_z = 0.0;
	cov_xy = 0.0;
	cov_xz = 0.0;
	cov_yz = 0.0;


	for(int j = 0; j < object.rows; j++)
		for (int i = 0; i < object.cols; i++)
		{
			px = object.at<cv::Point3f>(j,i);
			if ( px != cv::Point3f(0.0, 0.0, 0.0) && px != cv::Point3f(0, 255, 0))
			{
				var_x += pow( (px.x - centroid[0]), 2 );
				var_y += pow( (px.y - centroid[1]), 2 );
				var_z += pow( (px.z - centroid[2]), 2 );
				cov_xy += ( px.x - centroid[0] )*( px.y - centroid[1] );
				cov_xz += ( px.x - centroid[0] )*( px.z - centroid[2] );
				cov_yz += ( px.y - centroid[1] )*( px.z - centroid[2] );
				n++;
			}
		}

	var_x /= n;
	var_y /= n;
	var_z /= n;

	cov_xy /= n;
	cov_xz /= n;
	cov_yz /= n;

	cov_matrix << var_x,  cov_xy, cov_xz,
				 cov_xy,  var_y, cov_yz,
				 cov_xz, cov_yz,  var_z;

	//std::cout << "    cov_matrix: " << std::endl << cov_matrix << std::endl;

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov_matrix);

	//std::cout << "    eigenvectors: " << std::endl << eig.eigenvectors().col(0) <<
	//			std::endl << eig.eigenvectors().col(1) << std::endl <<
	//			std::endl << eig.eigenvectors().col(2) << std::endl;

	//std::cout << "    eigenvalues: " << std::endl << eig.eigenvalues().transpose() << std::endl;

	axis_1 = cv::Point3f(eig.eigenvectors()(0,2), eig.eigenvectors()(1,2), eig.eigenvectors()(2,2));
	axis_2 = cv::Point3f(eig.eigenvectors()(0,1), eig.eigenvectors()(1,1), eig.eigenvectors()(2,1));
	axis_3 = cv::Point3f(eig.eigenvectors()(0,0), eig.eigenvectors()(1,0), eig.eigenvectors()(2,0));


	principal_comp.push_back(axis_3*eig.eigenvalues()(0) * 70);
	principal_comp.push_back(axis_2*eig.eigenvalues()(1) * 70);
	principal_comp.push_back(axis_1*eig.eigenvalues()(2) * 70);

	return principal_comp;
}