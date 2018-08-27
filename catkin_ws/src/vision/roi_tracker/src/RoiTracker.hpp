#pragma once

#include <iostream>


#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"


#include "opencv2/flann/flann.hpp"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"

#include "boost/filesystem.hpp"
#include <vector>
#include <cmath>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv/cv_image.h>
#include <dlib/dir_nav.h>


using namespace dlib;


using namespace cv;
using namespace std;

class RoiTracker
{
    public:
        bool Debug;
        correlation_tracker tracker;

	//static bool faceSort(vision_msgs::VisionFaceObject &i, vision_msgs::VisionFaceObject &j);
	RoiTracker();
        bool LoadParams( std::string configFile );
        bool InitTracking(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Rect roiToTrack);
        bool InitTracking(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Rect roiToTrack, cv::Mat mask);
        bool InitFront(cv::Mat imaBGR, cv::Mat imaXYZ);
        bool Update(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Rect& nextRoi, double& confidence);
        bool UpdateROI(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Rect& nextRoi, double& confidence);
        bool Train(cv::Mat imaBGR, cv::Mat imaXYZ, bool& enableTrain,bool& enableTracker );
        double CompareHist(cv::Mat &Histo);
        cv::Rect dlibRectangleToOpenCV(dlib::rectangle r);
        dlib::rectangle openCVRectToDlib(cv::Rect r);

        bool IfPerson(cv::Mat imaBGR);
        bool Experiences();
        cv::Point3f centroidLast;

    private:

    	CascadeClassifier face_cascade;
    	/**VARIABLES VECTOR**/
	    std::vector<Rect> faces; // Vector para las caras detectadas

    	/**VARIABLES MAT*/
        Mat gray;

        cv::Mat CalculateHistogram(cv::Mat bgrIma, cv::Mat mask);
        cv::Mat CalculateHistogram(cv::Mat bgrIma);

        std::vector< cv::Rect > GetSearchRois( cv::Rect centerRoi, cv::Mat bgrIma );
        std::vector< cv::Rect > GetSearchRoisMultiscale( cv::Rect centerRoi, cv::Mat bgrIma );
        std::vector< cv::Rect > GetTrainRoisMultiscale( cv::Rect centerRoi, cv::Mat bgrIma );

        int noBins;
        int noExperiences;
        int Exper;
        bool init;
        bool success;
        cv::Mat histoToTrack;
        cv::Mat maskToTrack;
        cv::Rect roiToTrack;


        cv::Scalar frontLeftBot;
        cv::Scalar backRightTop;

        double overPercWidth;
        double overPercHeight;
        int overNoRectsWidth;
        int overNoRectsHeight;

        double scaleFactorIncrement;
        double scaleFactorDecrement;
        cv::Size scaleMax;
        cv::Size scaleMin;
        int scaleSteps;

        double matchThreshold;
};
