#pragma once
#include <iostream>
#include <queue>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/flann/flann.hpp"
#include "boost/filesystem.hpp"
#include "DetectedObject.hpp"
#include <ros/package.h>

class ObjRecognizer
{
public: 
    ObjRecognizer( int binNo ); 
    ObjRecognizer(); 

    int binNo; 
    std::string TrainingDir;
		
    double heightErrorThres; 
    double shapeErrorThres; 
    double colorErrorThres;  


    bool TrainObject(DetectedObject detObj,cv::Mat bgrImage, std::string name); 
    std::string RecognizeObject(DetectedObject detObj, cv::Mat bgrImage); 
    bool LoadTrainingDir();
    bool LoadTrainingDir(std::string trainingFolder);

private:

    std::vector< std::string > trainingNames; 
    std::vector<int> trainingIds; 
    std::vector<float> trainingHeights; 
    std::vector<cv::Mat> trainingHistos; 
    std::vector< std::vector< cv::Point2f > > trainingCont2D; 

    cv::Mat CalculateHistogram( cv::Mat bgrImage, cv::Mat mask ); 
};
