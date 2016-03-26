#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>

class LineSegment{

public:
	LineSegment( cv::Point2f p1, cv::Point2f ); 
	
	cv::Point2f GetP1() const; 
	cv::Point2f GetP2() const; 

	bool Intersects( LineSegment ls ); 
	static bool Intersection(	float p0_x, float p0_y, float p1_x, float p1_y, 
								float p2_x, float p2_y, float p3_x, float p3_y  ); 

private: 
	cv::Point2f p1; 
	cv::Point2f p2; 
	

}; 