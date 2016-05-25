#pragma once
#include <cmath>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

class Plane3D{

	public:
		Plane3D(); 
		Plane3D(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3);  
		Plane3D(cv::Point3f normal, cv::Point3f point); 

		double GetA();
		double GetB();
		double GetC();
		double GetD();
		cv::Point3f GetNormal(); 

		double DistanceToPoint(cv::Point3f p, bool signedDistance=false);  

		static bool AreValidPointsForPlane( cv::Point3f p1, cv::Point3f p2, cv::Point3f p3 ); 

	private: 
		double a; 
		double b; 
		double c; 
		double d;
};
