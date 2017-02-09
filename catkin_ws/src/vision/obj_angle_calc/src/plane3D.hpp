#pragma once
#include <cmath>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

class plane3D{

	public:
		plane3D();
		plane3D(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3);
		plane3D(cv::Point3f normal, cv::Point3f point);

		double GetA();
		double GetB();
		double GetC();
		double GetD();
		cv::Vec4d GetPlaneComp();
		cv::Point3f GetNormal();

		double DistanceToPoint(cv::Point3f p, bool signedDistance=false);

	private:
		double a;
		double b;
		double c;
		double d;
		cv::Vec4d planeComp;
};

// Definicion de un plano por un punto y el vector normal al plano
plane3D::plane3D()
{
	cv::Point3f p1(0.0 , 0.0, 0.0);
	cv::Point3f normal(0.0, 0.0, 1.0);

	this-> a = normal.x;
	this-> b = normal.y;
	this-> c = normal.z;
	this-> d = -(normal.x*p1.x +  normal.y*p1.y + normal.z*p1.z );
}

// Definicion de un plano por tres puntos
plane3D::plane3D(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3)
{
	cv::Point3f p12 = p2 - p1;
	cv::Point3f p13 = p3 - p1;

	// Producto cruz
	cv::Point3f normal = p12.cross( p13 );

	if( normal == cv::Point3f(0,0,0) )
		throw "Cant create Plane3D, normal is 0,0,0";

	// Se normaliza el vector
	normal *= 1 / cv::norm( normal );

	this-> a = normal.x;
	this-> b = normal.y;
	this-> c = normal.z;
	this-> d = - ( normal.x*p1.x + normal.y*p1.y + normal.z*p1.z );
}

// Definicion de un plano por un vector normal y un ponto
plane3D::plane3D(cv::Point3f normal, cv::Point3f p1)
{
	if( normal == cv::Point3f(0,0,0) )
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

cv::Point3f plane3D::GetNormal()
{
	return cv::Point3f(this->a, this->b, this->c);
}

cv::Vec4d plane3D::GetPlaneComp()
{
	return cv::Vec4d(this->a, this->b, this->c, this->d);
}

double plane3D::DistanceToPoint(cv::Point3f p, bool signedDistance)
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
