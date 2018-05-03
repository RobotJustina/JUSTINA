#pragma once
#include <iostream>
#include <queue>
#include <ros/package.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/flann/flann.hpp"
#include "opencv2/tracking/tracking.hpp"
#include "justina_tools/JustinaTools.h"
#include "boost/filesystem.hpp"
#include "Plane3D.hpp"
#include "PlanarSegment.hpp"
#include "DetectedObject.hpp"

class ObjExtractor
{
	public: 
		static bool DebugMode;
		static bool UseBetterPlanes;
		static int  H , S, V ;
		static int  Hth, Sth, Vth;	



		static cv::Mat CalculateNormals(cv::Mat pointCloud, cv::Mat mask=cv::Mat());
		static std::vector<DetectedObject> GetObjectsInHorizontalPlanes(cv::Mat pointCloud);
		static cv::Vec3f GetGrippers(cv::Mat imageBGR, cv::Mat pointCloud);
		static bool TrainGripper(cv::Mat imageBGR);
		static void LoadValueGripper();
		static std::vector<PlanarSegment> ExtractHorizontalPlanesRANSAC(cv::Mat pointCloud, double maxDistPointToPlane, int maxIterations, int minPointsForPlane, cv::Mat mask); 
		static std::vector<PlanarSegment> ExtractHorizontalPlanesRANSAC_2(cv::Mat pointCloud, double maxDistPointToPlane, int maxIterations, int minPointsForPlane, cv::Mat mask); 
		static std::vector< std::vector< int > >  SegmentByDistance( std::vector< cv::Point3f > xyzPoints, double distThreshold );
		static cv::Vec3f RandomFloatColor(); 

		static cv::Vec4i GetLine(cv::Mat pointCloud);
		//ObjExtractor::H = 130, ObjExtractor::S = 127, ObjExtractor::V = 127;
		//ObjExtractor:: Hth = 10, ObjExtractor::Sth = 80, ObjExtrac tor::Vth = 80;
		static std::vector<PlanarSegment>  GetHorizontalPlanes(cv::Mat pointCloud);

        static DetectedObject GetObjectInBox(cv::Mat& imaBGR, cv::Mat& imaXYZ); 
        static bool extractObjectsFromHorizontalPlanes(cv::Mat& imaXYZ, cv::Mat& objExtr);
	static bool extractObjectsIncludingPlanes(cv::Mat& imaXYZ, cv::Mat& objExtr);
        static cv::Scalar frontLeftTop;
        static cv::Scalar backRightTop; 

	private:
}; 



