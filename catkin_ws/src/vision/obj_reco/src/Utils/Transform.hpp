#pragma once
#include <iostream>
#include <math.h>
#include <cmath>
#include <opencv2/core/core.hpp>

class Transform
{
public:
	static void Kinect2Head(cv::Mat& mat, float pan, float tilt);
	static void Kinect2Robot(cv::Mat& mat, float pan, float tilt, float headZ);
	static void Kinect2World(cv::Mat& mat, float pan, float tilt, float roll, float headZ, float robotX, float robotY, float theta);
	//static void Kinect2World_(cv::Mat& mat, float pan, float tilt, float headZ, float robotX, float robotY, float theta);
};
