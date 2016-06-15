///LINE FITTING USING RANSAC
#ifndef LINERANSAC_H
#define LINERANSAC_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// Standard includes
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>

#define ERROR_APROX_PIX 3.0 
#define ERROR_APROX_METRIC 0.01

//Random sampling n numbers out maxn
cv::Mat randomSample(int n, cv::Mat points);

//Get the inliers in a point set over a 2D line defined by two points
std::vector<int> find2DConsensus(cv::Mat points, cv::Mat sample);

//Get the inliers in a point set over a 3D line defined by two points
std::vector<int> find3DConsensus(cv::Mat points, cv::Mat sample);

//Get the best line that fits a point set using RANSAC
std::vector<int> lineRANSAC(cv::Mat points);

#endif
