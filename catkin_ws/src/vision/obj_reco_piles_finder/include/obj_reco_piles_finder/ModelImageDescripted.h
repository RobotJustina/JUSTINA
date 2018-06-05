/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2017  <copyright holder> <email>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 */

#ifndef IMAGEDESCRIPTED_H
#define IMAGEDESCRIPTED_H

#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace std;

class ModelImageDescripted
{
  private:
    vector<KeyPoint> keypoints;
    Mat frame;
    Mat descriptors;
	string name;
  public:
    
    ModelImageDescripted(Mat frame, vector<KeyPoint> keypoints, Mat descriptors, string name);
	
	vector<Point2f>  scene_corners;
	int min_x;
	int max_x;
	float average_y;
	
    
    vector<KeyPoint> getKeypoints();
    Mat getFrame();
    Mat getDescriptors();
	string getName();
    
};

#endif // IMAGEDESCRIPTED_H
