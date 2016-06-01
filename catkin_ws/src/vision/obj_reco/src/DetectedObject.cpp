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
/*
DetectedObject::DetectedObject( std::vector< cv::Point2i > indexes, cv::Mat xyzPoints,  PlanarSegment planarSeg )
{
	this->centroid = cv::Point3f(0.0, 0.0, 0.0); 

	this->indexes = indexes; 
	this->xyzPoints = xyzPoints; 
	this->planarSeg = planarSeg;
	this->height = 0; 
	for( int i=0; i<indexes.size(); i++){

		cv::Point3f pXYZ = xyzPoints.at<cv::Vec3f>( indexes[i] ); 

		this->pointCloud.push_back( pXYZ ); 
		this->xyPoints2D.push_back( cv::Point2f(pXYZ.x, pXYZ.y) );

		double distToPlane = this->planarSeg.DistanceToPlane( pXYZ ); 
		if( distToPlane > this->height )
			this->height = distToPlane; 

		this->centroid = this->centroid + pXYZ; 
	}

	this->centroid = this->centroid * ( 1.0 / (float)indexes.size() ); 

	this->boundBox = cv::boundingRect( this->indexes ); 

	this->shadowOriBoundBoxt2D = cv::minAreaRect( xyPoints2D );
	cv::convexHull( xyPoints2D, this->shadowCHull ); 
	cv::approxPolyDP( this->shadowCHull, this->shadowContour2D, 0.00001, true); 
}
*/
