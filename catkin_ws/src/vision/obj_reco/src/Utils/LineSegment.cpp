#include "LineSegment.h"

LineSegment::LineSegment( cv::Point2f p1, cv::Point2f ){
	this->p1 = cv::Point2f(p1); 
	this->p2 = cv::Point2f(p2); 
}

cv::Point2f LineSegment::GetP1() const{
	return this->p1; 
}

cv::Point2f LineSegment::GetP2() const{
	return this->p2; 
}

bool LineSegment::Intersects( LineSegment ls ){

	float p0_x = this->p1.x; 
	float p0_y = this->p1.y;
	
	float p1_x = this->p2.x;  
	float p1_y = this->p2.y;
	
	float p2_x = ls.p1.x; 
	float p2_y = ls.p1.y; 
	
	float p3_x = ls.p2.x;
	float p3_y = ls.p2.y; 

	// Algorithm from StackOverflow XD
	double s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;		s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    double s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)    
        return true;
	else
		return false; // No collision
}

bool LineSegment::Intersection(float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y)
{
    double s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;		s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    double s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)    
        return true;
	else
		return false; // No collision
}