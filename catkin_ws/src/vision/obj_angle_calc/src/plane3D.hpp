#pragma once
#include <cmath>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

class plane3D{

	public:
		plane3D();
		plane3D(cv::Point3d p1, cv::Point3d p2, cv::Point3d p3);
		plane3D(cv::Point3d normal, cv::Point3d point);

		double GetA();
		double GetB();
		double GetC();
		double GetD();
		cv::Vec4d GetPlaneComp();
		cv::Point3d GetNormal();

		double DistanceToPoint(cv::Point3d p, bool signedDistance=false);

	private:
		double a;
		double b;
		double c;
		double d;
		cv::Vec4d planeComp;
};


plane3D::plane3D()
{
	cv::Point3d p1(0.0 , 0.0, 0.0);
	cv::Point3d normal(0.0, 0.0, 1.0);

	this-> a = normal.x;
	this-> b = normal.y;
	this-> c = normal.z;
	this-> d = -(normal.x*p1.x +  normal.y*p1.y + normal.z*p1.z );
}

// Definicion de un plano por tres puntos
plane3D::plane3D(cv::Point3d p1, cv::Point3d p2, cv::Point3d p3)
{
	cv::Point3d p12 = p2 - p1;
	cv::Point3d p13 = p3 - p1;
	cv::Point3d normal;

	//std::cout << "norm_p12:  " << cv::norm(p12) << std::endl;
	//std::cout << "norm_p13:  " << cv::norm(p13) << std::endl;

	normal = p12.cross( p13 );

	if( normal == cv::Point3d(0.0, 0.0, 0.0) )
		throw "Cant create Plane3D, normal is 0,0,0";


	// Se normaliza el vector
	//normal *= 1 / cv::norm( normal );

	this-> a = normal.x;
	this-> b = normal.y;
	this-> c = normal.z;
	this-> d = - ( normal.x*p1.x + normal.y*p1.y + normal.z*p1.z );
}

// Definicion de un plano por un vector normal y un ponto
plane3D::plane3D(cv::Point3d normal, cv::Point3d p1)
{
	if( normal == cv::Point3d(0,0,0) )
		throw "Cant create Plane3D, normal is 0,0,0";

	// Se normaliza el vector
	normal *= 1 / cv::norm( normal );

	this-> a = normal.x;
	this-> b = normal.y;
	this-> c = normal.z;
	this-> d = - ( normal.x*p1.x + normal.y*p1.y + normal.z*p1.z );
}

double plane3D::GetA()
{
	return this->a;
}

double plane3D::GetB()
{
	return this->b;
}

double plane3D::GetC()
{
	return this->c;
}

double plane3D::GetD()
{
	return this->d;
}

cv::Point3d plane3D::GetNormal()
{
	return cv::Point3d(this->a, this->b, this->c);
}

cv::Vec4d plane3D::GetPlaneComp()
{
	return cv::Vec4d(this->a, this->b, this->c, this->d);
}

double plane3D::DistanceToPoint(cv::Point3d p, bool signedDistance)
{
	double a = this->a;
	double b = this->b;
	double c = this->c;
	double d = this->d;
	double num = a*p.x + b*p.y + c*p.z + d ;
	double den = std::sqrt( a*a + b*b + c*c );

	if(!signedDistance)
		num = std::abs(num);

	return num/den;
}
