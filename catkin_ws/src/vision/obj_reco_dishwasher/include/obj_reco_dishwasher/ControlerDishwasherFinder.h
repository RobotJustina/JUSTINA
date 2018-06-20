/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2018  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef CONTROLEROBJECTSNOTEXTURENORD_H
#define CONTROLEROBJECTSNOTEXTURENORD_H

#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Vector3.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "justina_tools/JustinaTools.h"

#include "obj_reco_dishwasher/ModelDishwasher.h"

#include "vision_msgs/MSG_VisionDishwasher.h"

#include "vision_msgs/SRV_FindDishwasher.h"


class ControlerDishwasherFinder
{

public:
	
	bool debug;
	int minAceptableArea;
	int maxAceptableArea;
	float threshold_avgPixelsAreaOnImage;
		
	
	ControlerDishwasherFinder( ros::NodeHandle &, bool = false);
	
	void cb_searchObjectsOnTopic(const sensor_msgs::PointCloud2ConstPtr&);
	
	vision_msgs::MSG_VisionDishwasher cb_searchObjects(cv::Mat imgSrc, cv::Mat xyzCloud);
	
	bool cb_srv_FindPlaneObjects(vision_msgs::SRV_FindDishwasher::Request &req, vision_msgs::SRV_FindDishwasher::Response &resp);
	
private:
	
	Mat segmentColor(Mat (* segmetationFunction)(Mat,Mat, Scalar, Scalar),cv::Mat,Mat , Scalar minHSV, Scalar maxHSV);
	void markFound(vector<Point>contour, Rect bounding_rect,string nameRect,cv::Mat& output);	
	vector<Point> getLargestContourArea(Mat mask);
	vector<Point> getLargestContourArea(vector<vector<Point> > contours);
	vector<vector<Point> > getContours(Mat mask);
	Mat updateSearchingArea(cv::Mat wholeMask,cv::Mat subMask, Rect bounding_rect );
	Mat blindSpotOnInput(cv::Mat bgrInput,cv::Mat maskSpot, Rect bounding_rect );
	Mat getMaskLargestCountour(Mat mask, vector<Point>contour);
	vector<cv::Point3f> getMasked3DPOints(Mat XYZimg,Mat mask);
	vector<cv::Point3f> getMasked3DPOintsValidatedByDistancesDishWasher(vector<cv::Point3f> input3DPoints);
	vector<cv::Point3f> getMasked3DPOintsValidatedByDistancesDishShrink(vector<cv::Point3f> input3DPoints);
	
	
	vector<Point2f> adjustMinRect(Mat mask ,Point2f  rect_points[]);
	
	void getOrientation(vector<cv::Point3f> validPoints, cv::Point3f& cntr, vector<cv::Point3f>& eigen_vecs, vector<float>& eigen_val);
	cv::Point3f calculateSize(vector<cv::Point3f> validPoints );
	cv::Point3f calculateNearest3Dpoint(vector<cv::Point3f> validPoints);
	cv::Point3f calculateCenter3Dpoint( vector<cv::Point3f> validPoints );
	cv::Point3f calculateTallestPoint( vector<cv::Point3f> validPoints );
	//vision_msgs::VisionFlattenedObject createVisionFlattenedObject(cv::Mat xyzCloud, cv::Mat mask,cv::Rect bounding_rect, string name);
	
	//vector<vision_msgs::VisionFlattenedObject> insertObjectMessaje(vector<vision_msgs::VisionFlattenedObject> object_list, vision_msgs::VisionFlattenedObject object_msg);
	
	ros::NodeHandle		nodeHandle;
	ros::ServiceServer	srv_FindDishwasher;
	ros::ServiceClient  clt_RgbdRobot;
	ModelDishwasher models;
	
	
	
};

#endif // CONTROLEROBJECTSNOTEXTURENORD_H
