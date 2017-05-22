#pragma once
#include <iostream>
#include <queue>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/flann/flann.hpp"
#include "justina_tools/JustinaTools.h"
#include "Plane3D.hpp"
#include "PlanarSegment.hpp"
#include "DetectedObject.hpp"

class ObjExtractor
{
	public: 
		static bool DebugMode;
		static bool UseBetterPlanes; 

		static cv::Mat CalculateNormals(cv::Mat pointCloud, cv::Mat mask=cv::Mat());
		static std::vector<DetectedObject> GetObjectsInHorizontalPlanes(cv::Mat pointCloud);
		static std::vector<PlanarSegment> ExtractHorizontalPlanesRANSAC(cv::Mat pointCloud, double maxDistPointToPlane, int maxIterations, int minPointsForPlane, cv::Mat mask); 
		static std::vector<PlanarSegment> ExtractHorizontalPlanesRANSAC_2(cv::Mat pointCloud, double maxDistPointToPlane, int maxIterations, int minPointsForPlane, cv::Mat mask); 
		static std::vector< std::vector< int > >  SegmentByDistance( std::vector< cv::Point3f > xyzPoints, double distThreshold );
		static cv::Vec3f RandomFloatColor(); 

		static cv::Vec4i GetLine(cv::Mat pointCloud);
		static std::vector<PlanarSegment>  GetHorizontalPlanes(cv::Mat pointCloud);

	private:
}; 
