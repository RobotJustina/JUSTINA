#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class Plane3D{

public:
	double GetA();
	double GetB();
	double GetC();
	double GetD();
	cv::Point3f GetNormal(); 

	Plane3D(); 
	Plane3D( cv::Point3f p1, cv::Point3f p2, cv::Point3f p3 );  
	Plane3D( cv::Point3f normalVector , cv::Point3f point ); 

	double DistanceToPoint( cv::Point3f p, bool signedDistance = false  );  
	cv::Point3f ProjectPointIntoPlane( cv::Point3f p ); 

	static bool AreValidPointsForPlane( cv::Point3f p1, cv::Point3f p2, cv::Point3f p3 ); 

private: 
	double a; 
	double b; 
	double c; 
	double d;
};