#include "PlanarSegment.hpp"

PlanarSegment::PlanarSegment()
{}

PlanarSegment::PlanarSegment(Plane3D plane, std::vector<cv::Point3f> PointsXYZ, std::vector< cv::Point2i > indexes, std::vector<cv::Point2f> convexHull2D) 
{
	this->plane = Plane3D( plane ); 
	this->pointsXYZ = std::vector< cv::Point3f >( pointsXYZ ); 
	this->indexes = std::vector< cv::Point2i >( indexes ); 
	this->convexHull2D = std::vector<cv::Point2f>( convexHull2D ); 
}

std::vector< cv::Point3f > PlanarSegment::Get_PointsXYZ() const{
	return this->pointsXYZ; 
}

Plane3D PlanarSegment::Get_Plane() const{
	return this->plane; 
}

std::vector< cv::Point2i > PlanarSegment::Get_Indexes() const{
	return this->indexes; 
}

std::vector<cv::Point2f> PlanarSegment::Get_ConvexHull2D() const{
	return this->convexHull2D; 
}

double PlanarSegment::IsInside(cv::Point3f p){
	return cv::pointPolygonTest( this->convexHull2D, cv::Point2f( p.x, p.y ), true); 
}
