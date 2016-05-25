#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Plane3D.hpp"

class PlanarHorizontalSegment{

public: 
	PlanarHorizontalSegment( std::vector<cv::Point3f> PointsXYZ, Plane3D plane , cv::PCA pca, std::vector<cv::Point2f> convexHull2D); 
	PlanarHorizontalSegment(); 
	std::vector< cv::Point3f > Get_PointsXYZ() const; 
	Plane3D Get_Plane() const; 
	std::vector< cv::Point2f > Get_ConvexHull2D() const;
	float DistanceToPlane(cv::Point3f p); 

	double IsInside( cv::Point3f p); 

private: 
	std::vector< cv::Point3f > pointsXYZ; 
	Plane3D plane; 
	cv::PCA pca; 
	std::vector< cv::Point2f > convexHull2D;
}; 