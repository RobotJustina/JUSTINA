#pragma once

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/flann/flann.hpp"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"

#include "boost/filesystem.hpp"

class RoiTracker
{
    public:
        bool Debug;         

	//static bool faceSort(vision_msgs::VisionFaceObject &i, vision_msgs::VisionFaceObject &j);
	RoiTracker(); 
        bool LoadParams( std::string configFile ); 
        bool InitTracking(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Rect roiToTrack);
        bool InitTracking(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Rect roiToTrack, cv::Mat mask);
        bool InitFront(cv::Mat imaBGR, cv::Mat imaXYZ);
        bool Update(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Rect& nextRoi, double& confidence); 
        
    private:
	
        cv::Mat CalculateHistogram(cv::Mat bgrIma, cv::Mat mask);
        cv::Mat CalculateHistogram(cv::Mat bgrIma);

        std::vector< cv::Rect > GetSearchRois( cv::Rect centerRoi, cv::Mat bgrIma );
        std::vector< cv::Rect > GetSearchRoisMultiscale( cv::Rect centerRoi, cv::Mat bgrIma );
        
        int noBins;
        bool init; 
        cv::Mat histoToTrack; 
        cv::Mat maskToTrack; 
        cv::Rect roiToTrack;

        cv::Scalar frontLeftBot; 
        cv::Scalar backRightTop;

        double overPercWidth;
        double overPercHeight;
        int overNoRectsWidth;
        int overNoRectsHeight; 
        
        double scaleFactor; 
        cv::Size scaleMax; 
        cv::Size scaleMin; 
        int scaleSteps; 

        double matchThreshold;
};
