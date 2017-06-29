#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "PlanarSegment.hpp"

class DetectedObject{

public:
	std::vector< cv::Point2i > indexes; 
	cv::Mat xyzPoints; 
	cv::Mat oriMask; 
	PlanarSegment planarSeg; 

	std::vector< cv::Point3f > pointCloud; 
	std::vector< cv::Point2f > xyPoints2D; 
	cv::Point3f centroid; 

	cv::Rect boundBox; 
	double height; 
    cv::Mat image; 

	cv::RotatedRect shadowOriBoundBoxt2D; 
	std::vector< cv::Point2f > shadowCHull; 
	std::vector< cv::Point2f > shadowContour2D; 

	DetectedObject( std::vector< cv::Point2i > indexes, std::vector< cv::Point3f > points3D, std::vector<cv::Point2f> points2D, float height, cv::Point3f centroid, cv::Mat oriMask ); 
	DetectedObject( cv::Mat bgrIma, cv::Mat xyzIma, cv::Mat mask); 


    cv::Mat GetImageWithMask();

    static bool CompareByEuclidean(DetectedObject o1, DetectedObject o2); 
private: 

};

