#ifndef __OBJECTRECOGNIZER_HPP__
#define __OBJECTRECOGNIZER_HPP__

#include <iostream>
//#include <direct.h>
#include <boost/filesystem.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/ocl.hpp"
#include "opencv2/ocl/ocl.hpp"

#include "LineSegment.h"
//#include "UtilitiesVSN.hpp"

class ObjectRecognizer{

private:

	std::string name; 

	//std::string trainingDir;		// Directory were the images to train are //Now this value is passed by param to the function TrainingFromDirectory
	std::string trainingFile;	// File with names of the images to train 
	std::string detectorType;	
	std::string descriptorType;	

	cv::Ptr< cv::FeatureDetector > featDetector; 
	cv::Ptr< cv::DescriptorExtractor > featDescriptor;
	cv::Ptr< cv::DescriptorMatcher > featMatcher;
	
	cv::ocl::SURF_OCL oclSurf; 
	cv::SURF surf; 

	std::vector< cv::Mat > trainingImages;
	std::vector< std::string > trainingNames; 

	std::vector< std::vector< cv::KeyPoint > > trainingFeatures;
	std::vector< cv::Mat > trainingDescriptors;
	std::vector< cv::Ptr< cv::DescriptorMatcher > > trainingMatchers; 

public:
	ObjectRecognizer();
	ObjectRecognizer(std::string detectorType, std::string descriptorType, std::string trainingFile="ObjRecoFile"); 
	
	bool debugMode; 
	int maxBestMatches; 

	void TrainingFromDirectory(std::string trainingDir);

	std::string RecognizeObject(cv::Mat& originalImage, cv::Mat& outObjectInScene); 
	bool ValideteHomography( std::vector< cv::Point2f > trainCorners, std::vector< cv::Point2f > trainCornersTrans, cv::Mat homography ); 

	std::vector< std::string > GetFilesFromDir(std::string directory); 

	~ObjectRecognizer()
	{
	}
}; 

#endif