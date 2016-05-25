#include "PlanarHorizontalSegment.hpp"

PlanarHorizontalSegment::PlanarHorizontalSegment( std::vector<cv::Point3f> PointsXYZ, Plane3D plane , cv::PCA pca, std::vector<cv::Point2f> convexHull2D){
	this->pointsXYZ = std::vector< cv::Point3f >( pointsXYZ ); 
	this->plane = Plane3D( plane ); 
	this->pca = cv::PCA(pca); 
	this->convexHull2D = std::vector<cv::Point2f> (convexHull2D); 
}

PlanarHorizontalSegment::PlanarHorizontalSegment(){}

std::vector< cv::Point3f > PlanarHorizontalSegment::Get_PointsXYZ() const{
	return this->pointsXYZ; 
}

Plane3D PlanarHorizontalSegment::Get_Plane() const{
	return this->plane; 
}

std::vector<cv::Point2f> PlanarHorizontalSegment::Get_ConvexHull2D() const{
	return this->convexHull2D; 
}

double PlanarHorizontalSegment::IsInside( cv::Point3f p ){
	return cv::pointPolygonTest( this->convexHull2D, cv::Point2f( p.x, p.y ), true); 
}
float PlanarHorizontalSegment::DistanceToPlane(cv::Point3f p){
	return this->plane.DistanceToPoint( p ); 
}
