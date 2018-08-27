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

#include "obj_reco_piles_finder/ControlerPilesFinder.h"
#include <image_transport/image_transport.h>

ros::NodeHandle* node;

cv::Mat ImageRGB;
bool ReadyRGB=false;


/* IMAGE RGB*/
void cameraRGBCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Point3f p;
    cv::Mat m = cv_bridge::toCvShare(msg, "bgr8")->image;
    if( m.data==NULL){
      std::cout << "<<<<<<< NO DATA >>>>>>>" << std::endl;
    }
    else{
      cvtColor(m, m, cv::COLOR_RGB2GRAY);
      m.copyTo( ImageRGB );
      ReadyRGB=true;
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


/*TODO cambiar el tipo de mensajes para usar imagenes comprimidas al envío*/
bool ControlerPilesFinder::cb_srv_FindPiles(vision_msgs::SRV_FindObjectsPiles::Request &req, vision_msgs::SRV_FindObjectsPiles::Response &resp)
{
	Mat bgrImg;
	Mat img_matches;


	vector <vision_msgs::MSG_ObjectsPile> piles_msg;

	image_transport::ImageTransport it(*node);
	ros::Subscriber Camera_RGB  = node->subscribe ("/camera/rgb"  , 1,  cameraRGBCallback);
	ros::Rate loop(20);

		while( node->ok() )
    {
      loop.sleep();

      if(ReadyRGB==true)
      {
				ImageRGB.copyTo(bgrImg);
        ReadyRGB=false;

				break;
      }

      ros::spinOnce();
    }
		Camera_RGB.shutdown();



	resp.piles = cb_searchObjects( bgrImg, img_matches);
	//resp.output = img_matches;


	return true;
}


ControlerPilesFinder::ControlerPilesFinder(ros::NodeHandle &nodeHandle_, bool debug):nodeHandle(nodeHandle_) {

	ROS_INFO_STREAM("Starting: Piles finder NODE");

	loadROSParametersValues();


	loadKnowledgeBase(this->knowlede_base_file);

	node=&nodeHandle;
	this->srv_FindPilesObjects = nodeHandle.advertiseService("/vision/obj_reco/piles_finder", &ControlerPilesFinder::cb_srv_FindPiles, this) ;

	ROS_INFO_STREAM("Ready: Piles finder NODE");

	ros::spin();
}






vector <vision_msgs::MSG_ObjectsPile> ControlerPilesFinder::cb_searchObjects(cv::Mat imgSrc, Mat &img_matches){

	vector < vision_msgs::MSG_ObjectsPile> piles;

	Ptr<Feature2D> f2d;
	cv::Mat bgr;

	ROS_INFO_STREAM("Searching:  Piles Object finder");

	bgr = imgSrc.clone();


	f2d = xfeatures2d::SIFT::create();

	vector<ModelImageDescripted> scene_objects;
	vector<KeyPoint> keypoints;
	Mat descriptors;


	f2d->detect( bgr, keypoints );
	f2d->compute( bgr, keypoints, descriptors);

	img_matches = bgr.clone();
	scene_objects.clear();

	for(int i=0 ; i < this->models_knowledge_base.size(); i ++){
		ModelImageDescripted pattern = this->models_knowledge_base[i];

		FlannBasedMatcher matcher= FlannBasedMatcher();
		vector< DMatch > matches;
		matcher.match	( pattern.getDescriptors(), descriptors, matches );

		//-- Filter matches
		const double kDistanceCoef = 3.0;
		std::sort(matches.begin(), matches.end());
		while (matches.front().distance * kDistanceCoef < matches.back().distance) {
			matches.pop_back();
		}


		if(matches.size()>4){
			//-- Localize the object
			vector<Point2f> obj;
			vector<Point2f> scene;
			Mat mask;
			Mat H;

			for( size_t i = 0; i < matches.size(); i++ ){
				//-- Get the keypoints from the good matches
				obj.push_back( pattern.getKeypoints()[ matches[i].queryIdx ].pt );
				scene.push_back( keypoints[ matches[i].trainIdx ].pt );
			}

			H = findHomography( obj, scene, RANSAC,3, mask);

			int num_inliers=0;
			vector< DMatch > good_matches;
			for( int i = 0; i < matches.size(); i++ )
			{
				if((unsigned int)mask.at<uchar>(i)==1){
					num_inliers++;
					good_matches.push_back(matches[i]);
				}
			}

			//-- Get the corners from the image_1 ( the object to be "detected" )
			if(num_inliers>10){
				vector<Point2f> scene_corners(4);
				vector<Point2f> obj_corners(4);
				
				obj_corners[0] = cvPoint(0,0); 
				obj_corners[1] = cvPoint( pattern.getFrame().cols, 0 );
				obj_corners[2] = cvPoint( pattern.getFrame().cols, pattern.getFrame().rows ); 
				obj_corners[3] = cvPoint( 0, pattern.getFrame().rows );
				
				perspectiveTransform( obj_corners, scene_corners, H);

				if(contourArea(scene_corners)>500){
					//-- Draw lines between the corners (the mapped object in the scene - image_2 )
					line( img_matches, scene_corners[0], scene_corners[1], CV_RGB(255,0, 0), 2 );
					line( img_matches, scene_corners[1], scene_corners[2], CV_RGB(255,0, 0), 2 );
					line( img_matches, scene_corners[2] , scene_corners[3], CV_RGB(255,0, 0), 2 );
					line( img_matches, scene_corners[3] , scene_corners[0], CV_RGB(255,0, 0), 2 );
					setLabel(img_matches, pattern.getName(),scene_corners[0]);

					int min_x, max_x;
					float average_y;
					analizeObject (scene_corners,min_x, max_x,average_y );
					pattern.average_y = average_y;
					pattern.min_x = min_x;
					pattern.max_x = max_x;
					pattern.scene_corners = scene_corners;

					scene_objects.push_back(pattern);


				}
			}
		}

	}

// 	// Show detected matches
// 	namedWindow("Points matched", cv::WINDOW_AUTOSIZE);
// 	imshow("Points matched", img_matches);
// 	waitKey(30);


	vector <vector <ModelImageDescripted> > objects_arranged;

	objects_arranged = analizeScene(scene_objects);

	for(int n_pile = 0; n_pile < objects_arranged.size(); n_pile++){
		vision_msgs::MSG_ObjectsPile pile_msg;
		pile_msg.header.frame_id = "base_link";
		pile_msg.header.stamp    = ros::Time();

		for(int n_obj= 0 ; n_obj <objects_arranged[n_pile].size();n_obj++ ){
			std_msgs::String name;
			name.data= objects_arranged[n_pile][n_obj].getName();
			pile_msg.names.push_back( name );

		}
		piles.push_back(pile_msg);
	}





	ROS_INFO_STREAM("Finished searching: Piles Object finder");

	if( debug ){
		imshow("Input", bgr);
		waitKey(100);
		// Show detected matches
		namedWindow("Points matched", cv::WINDOW_AUTOSIZE);
		imshow("Points matched", img_matches);
		waitKey(30);

	}

	return piles;
}


vector < vector <ModelImageDescripted> > ControlerPilesFinder::analizeScene(vector<ModelImageDescripted> scene_objects){
	vector < vector <ModelImageDescripted> > objects_arranged;

	if (scene_objects.size()==0){
		return objects_arranged;
	}


	ModelImageDescripted initial_object = scene_objects.back();
	scene_objects.pop_back();

	vector <ModelImageDescripted> inititial_pile;
	inititial_pile.push_back(initial_object);
	objects_arranged.push_back(inititial_pile);

	while (scene_objects.size() != 0)
	{

		ModelImageDescripted unordered_object = scene_objects.back();
		scene_objects.pop_back();

		bool addedToPile = false;

		for(int pile_num = 0; pile_num < objects_arranged.size(); pile_num++ ){


			int min_x_inpile =  INT_MAX;
			int max_x_inpile =  INT_MIN;

			for(int object_on_pile = 0 ; object_on_pile < objects_arranged[pile_num].size(); object_on_pile++ ){
				if ( objects_arranged[pile_num][object_on_pile].max_x   >   max_x_inpile ){
					max_x_inpile = objects_arranged[pile_num][object_on_pile].max_x;
				}
				if( objects_arranged[pile_num][object_on_pile].min_x  <   min_x_inpile ){
					min_x_inpile = objects_arranged[pile_num][object_on_pile].min_x;
				}

			}

			if( (unordered_object.min_x >= min_x_inpile && unordered_object.min_x <= max_x_inpile) ||
				(unordered_object.max_x >= min_x_inpile && unordered_object.max_x <= max_x_inpile  ) ||
				(unordered_object.min_x < min_x_inpile && unordered_object.max_x > max_x_inpile)	){
				for (vector <ModelImageDescripted>::iterator it = objects_arranged[pile_num].begin() ; it != objects_arranged[pile_num].end(); ++it){
					if(unordered_object.average_y > (*it).average_y ){
						objects_arranged[pile_num].insert(it, unordered_object);
						addedToPile = true;
            break;
					}
				}
				if (addedToPile ==false){
					objects_arranged[pile_num].push_back(unordered_object);
					addedToPile = true;
				}
				}
		}
		if (addedToPile ==false){
			//its a new pile of objects
			vector <ModelImageDescripted> new_pile;
			new_pile.push_back(unordered_object);
			objects_arranged.push_back(new_pile);

		}
	}

	return objects_arranged;


}


void ControlerPilesFinder::analizeObject (vector<Point2f>  scene_object, int &min_x, int &max_x, float &average_y ){
	min_x = INT_MAX;
	max_x = INT_MIN;
	average_y = 0;

	for(int i = 0; i < scene_object.size(); i++){
		average_y += ( scene_object[i].y / scene_object.size() );
		if (scene_object[i].x > max_x ){
			max_x = scene_object[i].x;
		}
		if(scene_object[i].x < min_x ){
			min_x = scene_object[i].x;
		}
	}
}

void ControlerPilesFinder::setLabel(cv::Mat& im, const std::string label, const cv::Point &point)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::rectangle(im, point + cv::Point(0, baseline), point + cv::Point(text.width, -text.height), CV_RGB(255,0, 0), CV_FILLED);
	cv::putText(im, label, point, fontface, scale, CV_RGB(255,255,255), thickness, 8);
}


void ControlerPilesFinder::loadROSParametersValues() {
	//default values

	this->debug = true;
	this->knowlede_base_file = "/home/dougbel/git_repositories/JUSTINA/catkin_ws/src/vision/obj_reco_piles_finder/config/Keypoints.yml";



	if (!nodeHandle.getParam("knowlede_base_file", knowlede_base_file)) {
		ROS_ERROR("Could not find knowledge base file parameter from parameter server!");

	}

	if (!nodeHandle.getParam("debug_knowlede_base_file", debug)) {
		ROS_ERROR("Could not find debug_knowlede_base_file parameter from parameter server!");

	}

	ROS_INFO_STREAM("Knowledge  base file: "<< knowlede_base_file);

}

void ControlerPilesFinder::loadKnowledgeBase(string path_file){
	FileStorage fs(path_file, FileStorage::READ);
	FileNode n = fs["strings"];                         // Read string sequence - Get node

	if (n.type() != FileNode::SEQ)
	{
		cout << "strings is not a sequence! FAIL" << endl;
		return ;
	}

	models_knowledge_base.clear();

	FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
	for (; it != it_end; ++it){
		string name = (string)*it;

		vector<KeyPoint> read_keypoint;
		Mat read_descriptors;
		Mat read_frame;

		fs["keypoints_"+name] >> read_keypoint;
		fs["descriptors_"+name] >> read_descriptors;
		fs["image_"+name] >> read_frame;

		ModelImageDescripted pattern = ModelImageDescripted(read_frame, read_keypoint,read_descriptors,name);

		models_knowledge_base.push_back(pattern);
	}
}
