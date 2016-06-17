#include "DetectedObject.hpp"

DetectedObject::DetectedObject( std::vector< cv::Point2i > indexes, std::vector< cv::Point3f > points3D, std::vector<cv::Point2f> points2D, float height, cv::Point3f centroid, cv::Mat oriMask ) 
{
	this->indexes = std::vector< cv::Point2i >( indexes ); 
	this->pointCloud = std::vector< cv::Point3f >( points3D ); 
	this->xyPoints2D = std::vector<cv::Point2f>( points2D ) ; 
	this->height = height; 
	this->centroid = cv::Point3f( centroid ); 
	this->oriMask = oriMask; 

	this->boundBox = cv::boundingRect( this->indexes ); 
	this->shadowOriBoundBoxt2D = cv::minAreaRect( this-> xyPoints2D ); 
	cv::convexHull ( this->xyPoints2D, this->shadowCHull ); 
	cv::approxPolyDP( this->shadowCHull, this->shadowContour2D, 0.0001, true ); 
}
