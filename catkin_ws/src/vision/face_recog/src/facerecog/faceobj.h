#pragma once
#include "opencv2/opencv.hpp"

using namespace cv;

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

	faceobj();
	~faceobj();
	
};

