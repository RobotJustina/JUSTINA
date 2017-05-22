#include "Plane3D.hpp"

Plane3D::Plane3D()
{
	cv::Point3f p1(0.0 , 0.0, 0.0); 
	cv::Point3f normal(0.0, 0.0, 1.0); 

	this-> a = normal.x; 
	this-> b = normal.y; 
	this-> c = normal.z; 
	this-> d = -(normal.x*p1.x +  normal.y*p1.y + normal.z*p1.z );
}

Plane3D::Plane3D(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3)
{
	cv::Point3f p12 = p2 - p1; 
	cv::Point3f p13 = p3 - p1; 

	cv::Point3f normal = p12.cross( p13 ); 

	if( normal == cv::Point3f(0,0,0) ) 
		throw "Cant create Plane3D, normal is 0,0,0"; 

	normal *= 1 / cv::norm( normal ); 

	this-> a = normal.x; 
	this-> b = normal.y; 
	this-> c = normal.z; 
	this-> d = - ( normal.x*p1.x + normal.y*p1.y + normal.z*p1.z );
}

Plane3D::Plane3D(cv::Point3f normal, cv::Point3f p1)
{
	if( normal == cv::Point3f(0,0,0) ) 
		throw "Cant create Plane3D, normal is 0,0,0"; 

	normal *= 1 / cv::norm( normal ); 

	this-> a = normal.x; 
	this-> b = normal.y; 
	this-> c = normal.z; 
	this-> d = - ( normal.x*p1.x + normal.y*p1.y + normal.z*p1.z );
}

double Plane3D::GetA()
{	
	return this->a; 
}

double Plane3D::GetB()
{	
	return this->b; 
}

double Plane3D::GetC()
{	
	return this->c; 
}

double Plane3D::GetD()
{
	return this->d; 
}

cv::Point3f Plane3D::GetNormal()
{
	return cv::Point3f(this->a, this->b, this->c); 
}

double Plane3D::DistanceToPoint(cv::Point3f p, bool signedDistance)
{
	double num = a*p.x + b*p.y + c*p.z + d ; 
	double den = std::sqrt( a*a + b*b + c*c ); 

	if(!signedDistance) 
		num = std::abs(num); 

	return num/den;
}

bool Plane3D::AreValidPointsForPlane(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3)
{
	cv::Point3f p12 = p2 - p1; 
	cv::Point3f p13 = p3 - p1; 

	cv::Point3f normal = p12.cross( p13 ); 

	if( normal == cv::Point3f(0,0,0) )
		return false; 
	else
		return true; 
}

