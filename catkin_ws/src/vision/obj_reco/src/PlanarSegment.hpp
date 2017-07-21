#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Plane3D.hpp"

class PlanarSegment
{
	public: 
		PlanarSegment(Plane3D plane, std::vector<cv::Point3f> PointsXYZ, std::vector< cv::Point2i > indexes, std::vector<cv::Point2f> convexHull2D); 
		PlanarSegment(); 
        PlanarSegment(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Mat mask); // INCOMPLETE !! Only for detect spot 
		
		std::vector< cv::Point3f > Get_PointsXYZ() const; 
		std::vector< cv::Point2i > Get_Indexes() const; 
		Plane3D Get_Plane() const; 

		std::vector< cv::Point2f > Get_ConvexHull2D() const;

		float DistanceToPlane(cv::Point3f p); 
		double IsInside( cv::Point3f p); 

        std::vector<cv::Vec3f> DetectSpot( PlanarSegment planeSpot); 
        std::vector<cv::Point3f> DetectSpot(PlanarSegment planeSpoti, cv::Mat& spotMask, std::vector<cv::Point>& centerPixels); 
        
        // INCOMPLETE, Only for detect spot !!!
        cv::Mat imaBGR; 
        cv::Mat imaXYZ; 
        cv::Mat mask; 
        cv::Vec3i meanHSV; 
        cv::Vec3i stdDevHSV; 

private: 
		std::vector< cv::Point3f > pointsXYZ; 
		std::vector< cv::Point2i > indexes; 
		Plane3D plane; 
		std::vector< cv::Point2f > convexHull2D;


}; 

