#pragma once
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include "DetectedObject.hpp"
#include <queue>

class PersonExtractor
{

public: 
	PersonExtractor(); 
	
	cv::Size minFaceSize; 
	cv::Size maxFaceSize; 

	double maxDistanceBackgroud; 

	double bodyFaceHeightRatio; 
	double bodyFaceWidthRatio; 

	double maxDistanceBodyPixel; 
	
	std::vector< cv::Point3f > DetectFaces( cv::Mat bgrImage, cv::Mat pointCloud, std::vector< cv::Rect > outFaceBoxes ); 
	std::vector< cv::Mat > SegmentBody( cv::Mat bgImage, cv::Mat pointCloud, std::vector< cv::Rect > &outBodyRects ); 
	void SegmentDistanceNeightbours( cv::Mat& pointCloud, cv::Point2i seed, double maxDist, cv::Mat mask, cv::Mat& outSegmentationMask); 

	bool debugMode; 
	
private: 
	std::string trainingDir; 
	
	std::string faceTrainingFile;
	std::string upperBodyTrainingFile;
	std::string lowerBodyTrainingFile;
	std::string fullBodyTrainingFile;
	std::string smileTrainingFile;

	cv::CascadeClassifier faceClassifier; 
	cv::CascadeClassifier upperBodyClassifier; 
	cv::CascadeClassifier lowerBodyClassifier; 
	cv::CascadeClassifier fullBodyClassifier;
	cv::CascadeClassifier smileClassifier;
};