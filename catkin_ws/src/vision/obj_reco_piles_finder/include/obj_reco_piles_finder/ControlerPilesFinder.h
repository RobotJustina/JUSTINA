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


#include <math.h>

#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <cv_bridge/cv_bridge.h>


#include "justina_tools/JustinaTools.h"
#include "obj_reco_piles_finder/ModelPile.h"
#include "vision_msgs/MSG_ObjectsPile.h"
#include "vision_msgs/SRV_FindObjectsPiles.h"


class ControlerPilesFinder
{

public:
	
	bool debug;
	
	ControlerPilesFinder( ros::NodeHandle &, bool = false);
	
	
	vector <vision_msgs::MSG_ObjectsPile> cb_searchObjects(cv::Mat imgSrc, Mat &img_matches);
	
	bool cb_srv_FindPiles(vision_msgs::SRV_FindObjectsPiles::Request &req, vision_msgs::SRV_FindObjectsPiles::Response &resp);
	
private:
	vector < vector <ModelImageDescripted> > analizeScene(vector<ModelImageDescripted> scene_objects);
	void analizeObject (vector<Point2f>  scene_object, int &min_x, int &max_x, float &average_y );
	void setLabel(cv::Mat& im, const std::string label, const cv::Point &point);
	void loadROSParametersValues();
	void loadKnowledgeBase(string path_file);
		
	ros::NodeHandle		nodeHandle;
	ros::ServiceServer	srv_FindPilesObjects;
	ros::ServiceClient  clt_RgbChest;
	
	vector<ModelImageDescripted> models_knowledge_base;
	string knowlede_base_file;
	
	
	
};

#endif // CONTROLEROBJECTSNOTEXTURENORD_H
