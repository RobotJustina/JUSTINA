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

#include "obj_reco_plastic_tray/ControlerPlasticTrayFinder.h"



/*TODO cambiar el tipo de mensajes para usar imagenes comprimidas al envío*/
bool ControlerPlasticTrayFinder::cb_srv_FindPlaneObjects(vision_msgs::SRV_DetectPlasticTrayZones::Request &req, vision_msgs::SRV_DetectPlasticTrayZones::Response &resp)
{
	cv::Mat bgrImg;
	cv::Mat xyzCloud;
	
	vision_msgs::MSG_VisionPlasticTray response_msg;
	
	point_cloud_manager::GetRgbd srv;
	
	while(!clt_RgbdRobot.call(srv))
	{
		ROS_INFO_STREAM("Cannot get point cloud");
	}
	
	JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, bgrImg, xyzCloud);

	// imshow("pcl",  xyzCloud);
	// waitKey(10);
	
	response_msg = cb_searchObjects( bgrImg, xyzCloud);
	
	resp.plastic_tray_zones = response_msg;
	
	return true;
}


ControlerPlasticTrayFinder::ControlerPlasticTrayFinder(ros::NodeHandle &nodeHandle_, bool debug):nodeHandle(nodeHandle_) {
	
	ROS_INFO_STREAM("Starting: Plastic tray finder NODE");
	
	this->debug = debug;
	this->minAceptableArea = 10000;
	this->maxAceptableArea = 6000000;
	this->threshold_avgPixelsAreaOnImage = .3; //it takes 30% more or less to determine that is the same object
	
	this->models.loadKnowledgeBase();
	
	this->srv_FindPlaneObjects = nodeHandle.advertiseService("/vision/obj_reco/plastic_tray", &ControlerPlasticTrayFinder::cb_srv_FindPlaneObjects, this) ;
	this->clt_RgbdRobot        = nodeHandle.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
	
	
	//ros::Subscriber subscriber = nodeHandle.subscribe("/hardware/point_cloud_man/rgbd_wrt_robot",30, &ControlerPlasticTrayFinder::cb_searchObjectsOnTopic, this);

	ROS_INFO_STREAM("Ready: Plastic tray finder NODE");
	
	ros::spin();
}


void
ControlerPlasticTrayFinder::cb_searchObjectsOnTopic(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	
	cv::Mat imgSrc;
	cv::Mat xyzCloud;
	
	JustinaTools::PointCloud2Msg_ToCvMat(cloud_msg, imgSrc, xyzCloud);
	
	cb_searchObjects(imgSrc, xyzCloud);
	
}



vision_msgs::MSG_VisionPlasticTray  
ControlerPlasticTrayFinder::cb_searchObjects(cv::Mat imgSrc, cv::Mat xyzCloud){
	
	cv::Mat bgr;
	//cv::Mat founds;
	cv::Mat element;
	cv::Mat partialMaskColor, wholeMaskColor, ouputMask;
	Scalar minScalar;
	Scalar maxScalar;
	float averageAreaPixels;
	
	vector<Point> largest_contour;
	cv::Rect bounding_rect;
	string name;
	
	char str[400];
	int  largest_area;
	
	ROS_INFO_STREAM("Searching: Plastic tray finder");
	
	
	
	cv::Mat (* fptr) (cv::Mat,cv::Mat, Scalar, Scalar);
	
	element = getStructuringElement( MORPH_ELLIPSE, Size(11,11), Point( 5,5));
	
	
	bgr = imgSrc.clone();
	//bgr.copyTo(founds);
	
	vision_msgs::MSG_VisionPlasticTray response_msg;
	
	
	response_msg.header.frame_id = "base_link";
	response_msg.header.stamp    = ros::Time();
	
	ouputMask = Mat::zeros(bgr.rows, bgr.cols, CV_8UC1);
	
	
	
	for(int i = 0; i < models.size() ; i++){
		vector<vector<Point> > contours;
		
		name             = models.names[i];
		minScalar        = models.min_maxScalarsSegmentation[i][0];
		maxScalar        = models.min_maxScalarsSegmentation[i][1];
		fptr             = models.methodsSegmentation[i];
		averageAreaPixels= models.averagesAreasPixels[i];
		
		wholeMaskColor        = segmentColor(fptr, bgr, element, minScalar, maxScalar);		
		partialMaskColor      = wholeMaskColor.clone();
		
		
			
		partialMaskColor = wholeMaskColor;
		contours         = getContours(partialMaskColor);
		largest_contour  = getLargestContourArea(partialMaskColor);
		if(largest_contour.size()==0) break;
		largest_area     = contourArea(largest_contour,false);
		bounding_rect    = boundingRect(largest_contour);
		
		
		
		
		partialMaskColor = getMaskLargestCountour(partialMaskColor, largest_contour);
		
		if( largest_area > minAceptableArea ){
			
			partialMaskColor = getMaskLargestCountour(partialMaskColor, largest_contour);				
			wholeMaskColor   = updateSearchingArea(wholeMaskColor, partialMaskColor,bounding_rect);
			
			if ( averageAreaPixels == -1 ||
				( largest_area < averageAreaPixels+averageAreaPixels*threshold_avgPixelsAreaOnImage 
				&& largest_area > averageAreaPixels-averageAreaPixels*threshold_avgPixelsAreaOnImage)){ //si el valor es -1 quiere decir que es el único objeto con ese color y no vale la pena saber tamaños
					

					bgr = blindSpotOnInput( bgr, partialMaskColor, bounding_rect );
					ouputMask = ouputMask | partialMaskColor;					
					
					
					///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					RotatedRect minRect;
					Point2f     rect_points[4]; 
					vector<Point2f> rect_points_adjusted;
					
					minRect = minAreaRect(  Mat( largest_contour ) );
					minRect.points( rect_points );
					
					
					rect_points_adjusted = adjustMinRect(partialMaskColor,rect_points);
				
					/*TODO verificar que el orden de los puntos siempre es izquierda abajo, izquierda arriba, derecha arriba, derecha abajo*/
				
					//Finding the highest point to avoid crashes
// 					cv::Point3f  tallestPoint;
// 					vector <cv::Point3f> valid_masked_points;
// 					valid_masked_points = getMasked3DPOints(xyzCloud, partialMaskColor);				
// 					tallestPoint = calculateTallestPoint(valid_masked_points);


					//findind homography
					std::vector<Point2f> obj;
					obj.push_back(Point2f(0,100));
					obj.push_back(Point2f(0,0));
					obj.push_back(Point2f(100,0));
					obj.push_back(Point2f(100,100));
					
					Mat H = findHomography( obj, rect_points_adjusted, 0 );
					
					
					std::vector<Point2f> zone_dish;
					zone_dish.push_back(Point2f(0,100));
					zone_dish.push_back(Point2f(0,0));
					zone_dish.push_back(Point2f(50,0));
					zone_dish.push_back(Point2f(50,100));
					std::vector<Point2f> zone_glass;
					zone_glass.push_back(Point2f(50,100));
					zone_glass.push_back(Point2f(50,0));
					zone_glass.push_back(Point2f(100,0));
					zone_glass.push_back(Point2f(100,100));
					std::vector<Point2f> zone_clutery;
					zone_clutery.push_back(Point2f(85,60));
					zone_clutery.push_back(Point2f(85,0));
					zone_clutery.push_back(Point2f(100,0));
					zone_clutery.push_back(Point2f(100,60));
					
					
					std::vector<Point2f> zone_scene(4);
					vector<vector<Point> > contours;
					vector<Point>          esquinas;
					vector <cv::Point3f>   zonevalid_masked_points;
					vector<cv::Point3f>    eigen_vecs(3);
					vector<float>          eigen_val(3);
					cv::Point3f            cntr;
					cv::Point3f            size;
					cv::Point3f            nearestPoint;
					
					
					zone_scene.clear();
					contours.clear();
					esquinas.clear();
					perspectiveTransform( zone_dish, zone_scene, H);
					esquinas.push_back(Point(zone_scene[0].x,zone_scene[0].y));
					esquinas.push_back(Point(zone_scene[1].x,zone_scene[1].y));
					esquinas.push_back(Point(zone_scene[2].x,zone_scene[2].y));
					esquinas.push_back(Point(zone_scene[3].x,zone_scene[3].y));
					contours.push_back(esquinas);
					drawContours( bgr, contours, 0, cv::Scalar(255,128,128), cv::FILLED);
					
					Mat zoneMask_dish = Mat::zeros(partialMaskColor.rows, partialMaskColor.cols, partialMaskColor.type());					
					cv::drawContours( zoneMask_dish, contours, 0, cv::Scalar(255,255,255), cv::FILLED);
					
					
					zone_scene.clear();
					contours.clear();
					esquinas.clear();
					perspectiveTransform( zone_glass, zone_scene, H);
					esquinas.push_back(Point(zone_scene[0].x,zone_scene[0].y));
					esquinas.push_back(Point(zone_scene[1].x,zone_scene[1].y));
					esquinas.push_back(Point(zone_scene[2].x,zone_scene[2].y));
					esquinas.push_back(Point(zone_scene[3].x,zone_scene[3].y));
					contours.push_back(esquinas);
					drawContours( bgr, contours, 0, cv::Scalar(128,255,128), cv::FILLED);
					
					Mat zoneMask_glass = Mat::zeros(partialMaskColor.rows, partialMaskColor.cols, partialMaskColor.type());					
					cv::drawContours( zoneMask_glass, contours, 0, cv::Scalar(255,255,255), cv::FILLED);
					
					
					zone_scene.clear();
					contours.clear();
					esquinas.clear();
					perspectiveTransform( zone_clutery, zone_scene, H);
					esquinas.push_back(Point(zone_scene[0].x,zone_scene[0].y));
					esquinas.push_back(Point(zone_scene[1].x,zone_scene[1].y));
					esquinas.push_back(Point(zone_scene[2].x,zone_scene[2].y));
					esquinas.push_back(Point(zone_scene[3].x,zone_scene[3].y));
					contours.push_back(esquinas);
					drawContours( bgr, contours, 0, cv::Scalar(128,128,255), cv::FILLED);
					
					Mat zoneMask_clutery = Mat::zeros(partialMaskColor.rows, partialMaskColor.cols, partialMaskColor.type());					
					cv::drawContours( zoneMask_clutery, contours, 0, cv::Scalar(255,255,255), cv::FILLED);
					
					//ajustment for glasses
					zoneMask_glass = zoneMask_glass & ~zoneMask_clutery;
					erode(zoneMask_glass,zoneMask_glass, element);
					dilate(zoneMask_glass,zoneMask_glass, element);
					
					geometry_msgs::Vector3 eigenVector1, eigenVector2, eigenVector3;
					
					
					zonevalid_masked_points = getMasked3DPOints(xyzCloud, zoneMask_dish);
					getOrientation( zonevalid_masked_points ,cntr, eigen_vecs, eigen_val);
					size = calculateSize(zonevalid_masked_points);
					nearestPoint  = calculateNearest3Dpoint( zonevalid_masked_points );
					response_msg.center_point_zone_dish.x = cntr.x;
					response_msg.center_point_zone_dish.y = cntr.y;
					response_msg.center_point_zone_dish.z = cntr.z;
					response_msg.eigen_values_zone_dish.x = eigen_val[0];
					response_msg.eigen_values_zone_dish.y = eigen_val[1];
					response_msg.eigen_values_zone_dish.z = eigen_val[2];
					eigenVector1.x    = eigen_vecs[0].x;
					eigenVector1.y    = eigen_vecs[0].y;
					eigenVector1.z    = eigen_vecs[0].z;
					eigenVector2.x    = eigen_vecs[1].x;
					eigenVector2.y    = eigen_vecs[1].y;
					eigenVector2.z    = eigen_vecs[1].z;
					eigenVector3.x    = eigen_vecs[2].x;
					eigenVector3.y    = eigen_vecs[2].y;
					eigenVector3.z    = eigen_vecs[2].z;
					response_msg.eigen_vectors_zone_dish.push_back(eigenVector1);
					response_msg.eigen_vectors_zone_dish.push_back(eigenVector2);
					response_msg.eigen_vectors_zone_dish.push_back(eigenVector3);
					response_msg.nearest_point_zone_dish.x = nearestPoint.x;
					response_msg.nearest_point_zone_dish.y = nearestPoint.y;
					response_msg.nearest_point_zone_dish.z = nearestPoint.z;
					response_msg.size_zone_dish.x = size.x;
					response_msg.size_zone_dish.y = size.y;
					response_msg.size_zone_dish.z = size.z;
					
					
					zonevalid_masked_points = getMasked3DPOints(xyzCloud, zoneMask_glass);
					getOrientation( zonevalid_masked_points ,cntr, eigen_vecs, eigen_val);
					size = calculateSize(zonevalid_masked_points);
					nearestPoint  = calculateNearest3Dpoint( zonevalid_masked_points );
					response_msg.center_point_zone_glass.x = cntr.x;
					response_msg.center_point_zone_glass.y = cntr.y;
					response_msg.center_point_zone_glass.z = cntr.z;
					response_msg.eigen_values_zone_glass.x = eigen_val[0];
					response_msg.eigen_values_zone_glass.y = eigen_val[1];
					response_msg.eigen_values_zone_glass.z = eigen_val[2];
					eigenVector1.x    = eigen_vecs[0].x;
					eigenVector1.y    = eigen_vecs[0].y;
					eigenVector1.z    = eigen_vecs[0].z;
					eigenVector2.x    = eigen_vecs[1].x;
					eigenVector2.y    = eigen_vecs[1].y;
					eigenVector2.z    = eigen_vecs[1].z;
					eigenVector3.x    = eigen_vecs[2].x;
					eigenVector3.y    = eigen_vecs[2].y;
					eigenVector3.z    = eigen_vecs[2].z;
					response_msg.eigen_vectors_zone_glass.push_back(eigenVector1);
					response_msg.eigen_vectors_zone_glass.push_back(eigenVector2);
					response_msg.eigen_vectors_zone_glass.push_back(eigenVector3);
					response_msg.nearest_point_zone_glass.x = nearestPoint.x;
					response_msg.nearest_point_zone_glass.y = nearestPoint.y;
					response_msg.nearest_point_zone_glass.z = nearestPoint.z;
					response_msg.size_zone_glass.x = size.x;
					response_msg.size_zone_glass.y = size.y;
					response_msg.size_zone_glass.z = size.z;
					
					
					
					
					zonevalid_masked_points = getMasked3DPOints(xyzCloud, zoneMask_clutery);
					getOrientation( zonevalid_masked_points ,cntr, eigen_vecs, eigen_val);
					size = calculateSize(zonevalid_masked_points);
					nearestPoint  = calculateNearest3Dpoint( zonevalid_masked_points );
					response_msg.center_point_zone_cluttery.x = cntr.x;
					response_msg.center_point_zone_cluttery.y = cntr.y;
					response_msg.center_point_zone_cluttery.z = cntr.z;
					response_msg.eigen_values_zone_cluttery.x = eigen_val[0];
					response_msg.eigen_values_zone_cluttery.y = eigen_val[1];
					response_msg.eigen_values_zone_cluttery.z = eigen_val[2];
					eigenVector1.x    = eigen_vecs[0].x;
					eigenVector1.y    = eigen_vecs[0].y;
					eigenVector1.z    = eigen_vecs[0].z;
					eigenVector2.x    = eigen_vecs[1].x;
					eigenVector2.y    = eigen_vecs[1].y;
					eigenVector2.z    = eigen_vecs[1].z;
					eigenVector3.x    = eigen_vecs[2].x;
					eigenVector3.y    = eigen_vecs[2].y;
					eigenVector3.z    = eigen_vecs[2].z;
					response_msg.eigen_vectors_zone_cluttery.push_back(eigenVector1);
					response_msg.eigen_vectors_zone_cluttery.push_back(eigenVector2);
					response_msg.eigen_vectors_zone_cluttery.push_back(eigenVector3);
					response_msg.nearest_point_zone_cluttery.x = nearestPoint.x;
					response_msg.nearest_point_zone_cluttery.y = nearestPoint.y;
					response_msg.nearest_point_zone_cluttery.z = nearestPoint.z;
					response_msg.size_zone_cluttery.x = size.x;
					response_msg.size_zone_cluttery.y = size.y;
					response_msg.size_zone_cluttery.z = size.z;
					
					
					
					sprintf(str,"%s: %i",name.c_str(), largest_area);
					markFound(largest_contour,bounding_rect, str, bgr);
					sensor_msgs::Image container;
					cv_bridge::CvImage cvi_ouputMask;
					cvi_ouputMask.encoding = sensor_msgs::image_encodings::BGR8;
					cvi_ouputMask.image = bgr;
					cvi_ouputMask.toImageMsg(container);
					response_msg.imgOutput= container;
					
// 					for( int j = 0; j < 4; j++ )
// 						line( bgr, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 2, 8 );
					
// 					imshow("dishes", zoneMask_dish);
// 					imshow("glasses", zoneMask_glass);
// 					imshow("clutery", zoneMask_clutery);
// 					waitKey(15);
					
					
					
					
					break;
					
				}
				
		}
		if(largest_area > maxAceptableArea){
			ROS_ERROR_STREAM("Bad Color Segmentation");
			break;
		}
			
		
	}
	

// 	stringstream ss;
// 	ss << response_msg.objectList.size()<<" objectos detectados"<<endl;
// 	ROS_INFO_STREAM(ss.str());
	ROS_INFO_STREAM("Finished searching: Plastic tray finder");
	
	if( debug ){
		imshow("Blinded", bgr);
		waitKey(100);
		
	}
	
	return response_msg;
}



cv::Point3f ControlerPlasticTrayFinder::calculateTallestPoint( vector<cv::Point3f> validPoints ){

	
	cv::Point3f  tallest(0,0,0);
	
		
	for( int i = 0; i < validPoints.size(); i++ ) {
		
		cv::Point3f point = validPoints[i];
		if( tallest.z <  point.z ){
			tallest  = point;
		}
		
		
	}
	
	return tallest;
	
}



vector<Point2f> ControlerPlasticTrayFinder::adjustMinRect(Mat mask ,Point2f  rect_points[]){
	std::vector<Point2f> rect_points_adjusted;
	rect_points_adjusted.push_back(Point2f());
	rect_points_adjusted.push_back(Point2f());
	rect_points_adjusted.push_back(Point2f());
	rect_points_adjusted.push_back(Point2f());
	for( int j = 0; j < 4; j++ ){
		int minDistance = std::numeric_limits<int>::max();;
		for(int row= 0; row< mask.rows; row++){
			for(int col= 0; col< mask.cols; col++){
				int value = mask.at<uchar>(row,col);
				if(mask.at<uchar>(row,col) > 0 )
				{
					int distance = pow(rect_points[j].x - col,2)+ pow(rect_points[j].y - row,2);
					
					if(distance < minDistance){
						rect_points_adjusted[j] = Point2f(col,row);
						minDistance = distance;
					}
					
				}
				
			}
		}
	}
	return rect_points_adjusted;
}

cv::Point3f ControlerPlasticTrayFinder::calculateSize( vector<cv::Point3f> validPoints ){
	
	float min_x = std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float min_z = std::numeric_limits<float>::max();

	float max_x = std::numeric_limits<float>::min();
	float max_y = std::numeric_limits<float>::min();
	float max_z = std::numeric_limits<float>::min();
	
	cv::Point3f  size;
	
	for( int i = 0; i < validPoints.size(); i++ ) {
		
		cv::Point3f point = validPoints[i];
			
		if( min_x > point.x)  min_x = point.x;
		if( min_y > point.y)  min_y = point.y;
		if( min_z > point.z)  min_z = point.z;
		if( max_x < point.x)  max_x = point.x;
		if( max_y < point.y)  max_y = point.y;
		if( max_z < point.z)  max_z = point.z;
		
	}
	size.x = max_x - min_x;
	size.y = max_y - min_y;
	size.z = max_z - min_z;
	
	return size;
	
}


cv::Point3f ControlerPlasticTrayFinder::calculateNearest3Dpoint( vector<cv::Point3f> validPoints ){
	
	float min_distance = std::numeric_limits<float>::max();
	
	cv::Point3f nearestPoint;
	
	for( int i = 0; i < validPoints.size(); i++ ) {
		
		cv::Point3f point = validPoints[i];
		float distance = cv::norm( point );
		
		if ( min_distance >  distance ){
			min_distance = distance;
			nearestPoint = point;
		}		
	}
	return nearestPoint;
}



vector<cv::Point3f> ControlerPlasticTrayFinder::getMasked3DPOints(cv::Mat XYZimg, cv::Mat mask){
	vector <cv::Point3f> validPoints;
	
	
	for( int y = 0; y < XYZimg.rows; y++ ) {
		for( int x = 0; x < XYZimg.cols; x++ ) {
			if ( mask.at<uchar>(y,x) != 0 ) {
				Point3f point3D= XYZimg.at<cv::Vec3f>(y,x);
				if( !(point3D.x==0 && point3D.y==0 && point3D.z==0) ){
					validPoints.push_back(point3D);
				}
			}
		}
	}
	return validPoints;
}


void ControlerPlasticTrayFinder::getOrientation( vector<cv::Point3f> validPoints, cv::Point3f& cntr, vector<cv::Point3f>& eigen_vecs, vector<float>& eigen_val)
{
	
	if(validPoints.size()==0){
		return;
	}
	
	//Construct a buffer used by the pca analysis
	int sz = validPoints.size();
	cv::Mat data_pts = cv::Mat(sz, 3, CV_32FC1);
	
	for (int i = 0; i < data_pts.rows; ++i)
	{
		data_pts.at<float>(i, 0) = validPoints[i].x;
		data_pts.at<float>(i, 1) = validPoints[i].y;
		data_pts.at<float>(i, 2) = validPoints[i].z;
	}
	//Perform PCA analysis
	PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
	//Store the center of the object
	cntr = cv::Point3f(pca_analysis.mean.at<float>(0, 0),
				   pca_analysis.mean.at<float>(0, 1),
				   pca_analysis.mean.at<float>(0, 2));
	
	for (int i = 0; i < 3; ++i)
	{
		eigen_vecs[i] = cv::Point3f(pca_analysis.eigenvectors.row(i));
		eigen_val[i] = pca_analysis.eigenvalues.at<float>(0, i);
	}
	
	return ;
}


vector<Point> ControlerPlasticTrayFinder::getLargestContourArea(cv::Mat mask){
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	
	int largest_area=0;
	vector<Point> largest_contour;
	int largest_contour_index=0;
	cv::Rect bounding_rect;
	for( int i = 0; i< contours.size(); i++ )
	{
		//  Find the area of contour
		double a=contourArea( contours[i],false);
		if(a>largest_area){
			largest_area=a;
			// Store the index of largest contour
			largest_contour_index=i;
			// store largest contour 
			largest_contour = contours[i];
		}
	}
	return largest_contour;
}



vector<vector<Point> > ControlerPlasticTrayFinder::getContours(cv::Mat mask){
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	
	return contours;
}


cv::Mat ControlerPlasticTrayFinder::updateSearchingArea( cv::Mat wholeMask, cv::Mat subMask, cv::Rect bounding_rect ){
	cv::Mat tmp = wholeMask.clone();
	cv::Mat ROI=tmp(bounding_rect);
	cv::Mat ROI2=subMask(bounding_rect);
	ROI = ROI & ~ROI2;
	
	return tmp;
}

cv::Mat ControlerPlasticTrayFinder::blindSpotOnInput( cv::Mat bgrInput, cv::Mat maskSpot, cv::Rect bounding_rect ){
	
	cv::Mat tmp = bgrInput.clone();
	
	cvtColor(maskSpot, maskSpot, CV_GRAY2BGR);
	
	cv::Mat ROI=tmp(bounding_rect);
	cv::Mat ROI2=maskSpot(bounding_rect);
	ROI = ROI & ~ROI2;
	
	return tmp;
	
}

cv::Mat ControlerPlasticTrayFinder::getMaskLargestCountour(cv::Mat mask, vector<Point>contour){
	
	cv::Mat maskUpdate=cv::Mat::zeros(mask.rows,mask.cols,mask.type());
	
	vector<vector<Point> > contours;
	contours.push_back(contour);
	vector<vector<Point> >hull(1);
	convexHull( cv::Mat(contour), hull[0], false );
	cv::drawContours( maskUpdate, hull, 0, cv::Scalar(255,255,255), cv::FILLED);
	
	//cv::drawContours( maskUpdate, contours, 0, cv::Scalar(255,255,255), cv::FILLED);
	
	return maskUpdate;
}

cv::Mat ControlerPlasticTrayFinder::segmentColor(cv::Mat (* segmetationFunction)(cv::Mat,cv::Mat, Scalar, Scalar),cv::Mat bgr,cv::Mat element, Scalar minHSV, Scalar maxHSV){
	
	return (* segmetationFunction)(bgr, element, minHSV, maxHSV);
}

void ControlerPlasticTrayFinder::markFound(vector<Point>contour, cv::Rect bounding_rect,string name, cv::Mat& output){

    rectangle(output, bounding_rect,  Scalar(0,255,0),2, 8,0);
    putText(output, name, Point2f(bounding_rect.x,bounding_rect.y), FONT_HERSHEY_PLAIN, 0.9,  Scalar(017,070,244,255));
    

    return;
}
     
