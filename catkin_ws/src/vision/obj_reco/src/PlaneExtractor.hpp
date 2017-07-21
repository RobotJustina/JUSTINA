#pragma once
#include <iostream>
#include <queue>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/flann/flann.hpp>

#include "Plane3D.hpp"
#include "PlanarSegment.hpp"
#include "DetectedObject.hpp"

class PlaneExtractor
{
    public:
        bool debug;
        
        PlaneExtractor();
        std::vector<PlanarSegment> GetHorizontalPlanesRANSAC(cv::Mat& imaBGR, cv::Mat& imaXYZ);
        cv::Mat CalculateFastNormalsImage(cv::Mat imaXYZ, cv::Mat mask = cv::Mat()); 
        std::vector< std::vector< int > > SegmentByDistance(std::vector< cv::Point3f > xyzPointsList, double distThreshold); 
        std::vector< std::vector< int > > SegmentByDistancePlanes(std::vector< cv::Point3f > xyzPointsList, double distThreshold, double maxDistZ=0.02); 
        void DrawHistogram(cv::Mat ima, cv::Mat mask,std::string histoName);


    private:
};
