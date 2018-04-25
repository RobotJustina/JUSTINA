#pragma once
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"

using namespace std;
using namespace cv;
using namespace cv::face;

class faceobj
{
public:

	typedef enum { female, male, unknown } Gender;
	
	Mat faceRGB;
	Mat facePC;
	Rect boundingbox;
	Point3f pos3D;
	string id;
	double confidence;
	Gender gender;
	bool smile;
    int ages;

	faceobj();
	~faceobj();
	
};

