#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <map>
#include <stdlib.h>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <ros/package.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "justina_tools/JustinaTools.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "vision_msgs/Cube.h"
#include "vision_msgs/CubesSegmented.h"
#include "vision_msgs/GetCubes.h"
#include "vision_msgs/FindPlane.h"
#include "vision_msgs/DetectObjects.h"

#include "visualization_msgs/MarkerArray.h"

using namespace std;
using namespace cv;

ros::NodeHandle* node;

ros::ServiceServer srvCubesSeg;
ros::ServiceServer srvCutlerySeg;
ros::ServiceClient cltRgbdRobot;
ros::ServiceClient cltFindPlane;
ros::ServiceClient cltExtObj;
ros::ServiceClient cltExtCut;
ros::Subscriber subCalibColor;
tf::TransformListener * transformListener;


visualization_msgs::MarkerArray cubesMarker;
std::map<std::string, visualization_msgs::Marker> cubesMapMarker;

int Hmin=0, Smin=0, Vmin=0, Hmax=0, Smax=0, Vmax=0;

float minX = 0.10, maxX = 1.0;
float minY = -0.5, maxY = 0.5;
float minZ = 0.65, maxZ = 2.0;

string colour;

std::stringstream Huemin, Huemax, Satmin, Satmax, Valmin, Valmax;

bool cropping = false;
bool getRoi = false;
bool getPointColor = false;
int xmin, ymin, xmax, ymax;

void pcaAnalysis(cv::Mat &mask, cv::Mat &xyzCloud, float &roll, float &pitch, float &yaw){
    std::vector<cv::Vec3f> points;
    cv::Mat data_pts;
    for (int row = 0; row < mask.rows; ++row)
        for (int col = 0; col < mask.cols; ++col)
            if (mask.at<uchar>(row,col)>0){
                points.push_back(xyzCloud.at<cv::Vec3f>(row, col));
            }
    int sz = static_cast<int>(points.size());
    data_pts = cv::Mat(sz, 3, CV_64FC1);
    for(int i = 0; i < data_pts.rows; i++){
        cv::Vec3f p = points[i];
        data_pts.at<double>(i, 0) = static_cast<double>(p[0]);
        data_pts.at<double>(i, 1) = static_cast<double>(p[1]);
        data_pts.at<double>(i, 2) = static_cast<double>(p[2]);
    }
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
    cv::Vec3d cntr = cv::Point3d(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1), pca_analysis.mean.at<double>(0, 2));

    std::vector<cv::Vec3d> eigen_vecs(3);
    std::vector<cv::Vec3d> eigen_vecsn(3);
    std::vector<double> eigen_val(3);
    for (int i = 0; i < 3; i++) {
        eigen_vecs[i] = cv::Vec3d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1), pca_analysis.eigenvectors.at<double>(i, 2));
        eigen_vecsn[i] = cv::normalize(eigen_vecs[i]);
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    std::cout << "Eigen values: 0 " << eigen_val[0] << std::endl;
    std::cout << "Eigen values: 1 " << eigen_val[1] << std::endl;
    std::cout << "Eigen values: 2 " << eigen_val[2] << std::endl;

    /*
    if(eigen_val[1] / eigen_val[0] > 0.6 ){
        std::cout << "cubes_segmentation_node.->The main axis and the next axis of closest size is almost equal." << std::endl;
        roll = 0.0;
        pitch = 0.0;
        yaw = 1.5708;
    }
    else{
        std::cout << "cubes_segmentation_node.->The main axis is biger that the others axes" << std::endl;
        double angle = atan2(eigen_vecs[0][1], eigen_vecs[0][0]);
        roll = (float) angle;
        pitch = 0.0;
        yaw = 0.0;
    }*/
    if(eigen_val[1] / eigen_val[0] > 0.6 ){
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
    }
    else{
        double angle = -atan2(eigen_vecs[0][1], eigen_vecs[0][0]);
        /*if(angle >= M_PI)
          angle = M_PI - angle;*/
        roll = (float) angle;
        pitch = 0.0;
        yaw = 0.0;
    }
    std::cout << "cubes_segmentation_node.->Orientation of gripper, roll:" << roll << ", pitch:" << pitch << ", yaw:" << yaw  << std::endl;
}

void on_trackbar(int, void*) 
{
	/*if (bloques <= 1) {
	 bloques = 3;
	 } else {
	 if (bloques % 2 != 1) {
	 bloques += 1;
	 }
	 }*/
}

void on_mouse(int event, int x, int y, int flags, void* param) 
{
	if (event == CV_EVENT_LBUTTONDOWN) 
	{
		xmin = x;
		xmax = x;
		ymin = y;
		ymax = y;
		cropping = true;
	}
	else if (event == CV_EVENT_MOUSEMOVE && cropping) 
	{
		xmax = x;
		ymax = y;
	} 
	else if (event == CV_EVENT_LBUTTONUP) 
	{
		xmax = x;
		ymax = y;
		cropping = false;
        if(xmin >= xmax || ymin >= ymax){
		    getRoi = false;
            getPointColor = true;
        }
        else{
		    getRoi = true;
            getPointColor = false;
        }
	}
}

void loadValuesFromFile(string color, bool test)
{
	std::string configDir = ros::package::getPath("cubes_segmentation") + "/ConfigDir";
	std::string configFile;
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 
	if (test)
		configFile = configDir + "/Cubes_config.xml";
	else
		configFile = configDir + "/Cutlery_config.xml";
	cv::FileStorage fs;


	Huemin << "H_min" << color;
    Huemax << "H_max" << color;
    Satmin << "S_min" << color;
    Satmax << "S_max" << color;
    Valmin << "V_min" << color;
    Valmax << "V_max" << color;

	fs.open(configFile, fs.READ );

	Hmin = (int)fs[Huemin.str()]; 
	Smin = (int)fs[Satmin.str()]; 
	Vmin = (int)fs[Valmin.str()]; 
	Hmax = (int)fs[Huemax.str()]; 
	Smax = (int)fs[Satmax.str()]; 
	Vmax = (int)fs[Valmax.str()]; 
		
	fs.release();

	Huemin.str(std::string());
	Huemax.str(std::string());
	Satmin.str(std::string());
	Satmax.str(std::string());
	Valmin.str(std::string());
	Valmax.str(std::string());

}


bool GetImagesFromJustina( cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "CubesSegmentation.->Cannot get point cloud" << std::endl;
        return false;
    }
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}

void callbackCalibrateCutlery(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "CubesSegmentation.->Calibrate colour cutlery" << std::endl;
    colour = msg->data;
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat frameWork;
    cv::Mat frameHSV;
    cv::Mat maskRange;
    cv::Mat mask;

    Huemin << "H_min" << colour;
    Huemax << "H_max" << colour;
    Satmin << "S_min" << colour;
    Satmax << "S_max" << colour;
    Valmin << "V_min" << colour;
    Valmax << "V_max" << colour;

    loadValuesFromFile(colour, false);
    
    std::string configDir = ros::package::getPath("cubes_segmentation") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 

	std::string configFile =configDir + "/Cutlery_config.xml";
	cv::FileStorage fs;

	if(!boost::filesystem::exists(configFile))
	{
		fs.open(configFile, fs.WRITE);
		fs.release();	
	}
	
    ros::Rate loop(30);

    while(ros::ok() && cv::waitKey(1) != 'q')
	{
    	GetImagesFromJustina(bgrImg,xyzCloud);
    	bgrImg.copyTo(frameWork);
        blur( frameWork, frameWork, Size(4, 4) , Point(-1,-1) );

    	if(!cropping && !getRoi)
			imshow("Original", bgrImg);
		else if (cropping && !getRoi) 
		{
			cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
			imshow("Original", bgrImg);
		} 
		else if (!cropping && getRoi) 
		{
			cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
			imshow("Original", bgrImg);
		}

		cv::createTrackbar("HMIN", "Original", &Hmin, 255, on_trackbar);
		cv::createTrackbar("HMAX", "Original", &Hmax, 255, on_trackbar);
		cv::createTrackbar("SMIN", "Original", &Smin, 255, on_trackbar);
		cv::createTrackbar("SMAX", "Original", &Smax, 255, on_trackbar);
		cv::createTrackbar("VMIN", "Original", &Vmin, 255, on_trackbar);
		cv::createTrackbar("VMAX", "Original", &Vmax, 255, on_trackbar);
		setMouseCallback("Original", on_mouse, &bgrImg);

		if (getRoi) 
		{
			cv::Rect rect(xmin, ymin, xmax - xmin, ymax - ymin);
			cv::Mat roi = frameWork(rect);
			cv::Mat roiHSV;
			cv::cvtColor(roi, roiHSV, CV_BGR2HSV);
			std::vector<cv::Mat> channels;
			cv::split(roiHSV, channels);
			double minVal, maxVal;
			cv::Point minPos, maxPos;
			cv::minMaxLoc(channels[0], &minVal, &maxVal, &minPos, &maxPos);
			Hmin = minVal;
			Hmax = maxVal;
			cv::minMaxLoc(channels[1], &minVal, &maxVal, &minPos, &maxPos);
			Smin = minVal;
			Smax = maxVal;
			cv::minMaxLoc(channels[2], &minVal, &maxVal, &minPos, &maxPos);
			Vmin = minVal;
			Vmax = maxVal;
			cvSetTrackbarPos("HMIN", "Original", Hmin);
			cvSetTrackbarPos("HMAX", "Original", Hmax);
			cvSetTrackbarPos("SMIN", "Original", Smin);
			cvSetTrackbarPos("SMAX", "Original", Smax);
			cvSetTrackbarPos("VMIN", "Original", Vmin);
			cvSetTrackbarPos("VMAX", "Original", Vmax);
			cv::imshow("Roi", roi);
			cv::imshow("RoiHSV", roiHSV);
			getRoi = false;
		}
        if(getPointColor){
			cv::Rect rect(xmin, ymin, xmax - xmin, ymax - ymin);
			cv::Mat roi = frameWork(rect);
			cv::Mat roiHSV;
			cv::cvtColor(roi, roiHSV, CV_BGR2HSV);
			std::vector<cv::Mat> channels;
			cv::split(roiHSV, channels);
			double minVal, maxVal;
			cv::Point minPos, maxPos;
			cv::minMaxLoc(channels[0], &minVal, &maxVal, &minPos, &maxPos);
			Hmin = minVal;
			Hmax = maxVal;
			cv::minMaxLoc(channels[1], &minVal, &maxVal, &minPos, &maxPos);
			Smin = minVal;
			Smax = maxVal;
			cv::minMaxLoc(channels[2], &minVal, &maxVal, &minPos, &maxPos);
			Vmin = minVal;
			Vmax = maxVal;
			cvSetTrackbarPos("HMIN", "Original", Hmin);
			cvSetTrackbarPos("HMAX", "Original", Hmax);
			cvSetTrackbarPos("SMIN", "Original", Smin);
			cvSetTrackbarPos("SMAX", "Original", Smax);
			cvSetTrackbarPos("VMIN", "Original", Vmin);
			cvSetTrackbarPos("VMAX", "Original", Vmax);
			getPointColor = false;
        }
        
		cv::cvtColor(frameWork, frameHSV, CV_BGR2HSV);
		cv::inRange(frameHSV, cv::Scalar(Hmin, Smin, Vmin),cv::Scalar(Hmax, Smax, Vmax), maskRange);
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1.5, 1.5));
		cv::morphologyEx(maskRange, mask, cv::MORPH_ERODE, kernel, cv::Point(-1, -1), 1);
		cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel, cv::Point(-1, -1), 7);

		cv::Mat maskedImage;
		frameWork.copyTo(maskedImage, mask);
		// Compute the centorid mask
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat canny_output;
		mask.copyTo(canny_output);
		cv::imshow("Canny", canny_output);
		cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		if (contours.size() > 0) 
		{
			double maxArea = -1;
			int indexMaxArea = 0;
			for (unsigned int i = 0; i < contours.size(); i++) 
			{
				float area = cv::contourArea(contours[i]);
				if (area > maxArea) 
				{
					maxArea = area;
					indexMaxArea = i;
				}
			}
			std::vector<cv::Point> contour_poly;
			cv::approxPolyDP(cv::Mat(contours[indexMaxArea]), contour_poly, 3,true);
			cv::boundingRect(contour_poly);
			cv::rectangle(maskedImage, cv::boundingRect(contour_poly).tl(),cv::boundingRect(contour_poly).br(), CV_RGB(124, 40, 30), 2, 8, 0);
			cv::Moments centroide = moments(contours[indexMaxArea], false);
			cv::Point punto(centroide.m10 / centroide.m00, centroide.m01 / centroide.m00);
			cv::circle(maskedImage, punto, 4, CV_RGB(124, 40, 30), -1, 8, 0);
			std::cout << cv::Mat(punto) << std::endl;

		}

		imshow("Color mask", mask);
		cv::imshow("Image with mask", maskedImage);

		if(cv::waitKey(1)=='s')
		{
			fs.open(configFile, fs.APPEND);

			fs<< Huemin.str() << Hmin;
			fs<< Huemax.str() << Hmax;
			fs<< Satmin.str() << Smin;
			fs<< Satmax.str() << Smax;
			fs<< Valmin.str() << Vmin;
			fs<< Valmax.str() << Vmax;
			
			fs.release();

			std::cout << colour << " Calibration Completed..." << std::endl;

			Huemin.str(std::string());
			Huemax.str(std::string());
			Satmin.str(std::string());
			Satmax.str(std::string());
			Valmin.str(std::string());
			Valmax.str(std::string());
			
		}
		
		ros::spinOnce();
        loop.sleep();
    }
}


void callbackCalibrateV2(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "CubesSegmentation.->Calibrate colour V2" << std::endl;
    colour = msg->data;
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat frameWork;
    cv::Mat frameHSV;
    cv::Mat maskRange;
    cv::Mat mask;

    Huemin << "H_min" << colour;
    Huemax << "H_max" << colour;
    Satmin << "S_min" << colour;
    Satmax << "S_max" << colour;
    Valmin << "V_min" << colour;
    Valmax << "V_max" << colour;

    
    
    std::string configDir = ros::package::getPath("cubes_segmentation") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 

	std::string configFile =configDir + "/Cubes_config.xml";
	cv::FileStorage fs;

	if(!boost::filesystem::exists(configFile))
	{
		fs.open(configFile, fs.WRITE);
		fs.release();	
	}
	
    ros::Rate loop(30);

    while(ros::ok() && cv::waitKey(1) != 'q')
	{
    	GetImagesFromJustina(bgrImg,xyzCloud);
    	bgrImg.copyTo(frameWork);

    	if(!cropping && !getRoi)
			imshow("Original", bgrImg);
		else if (cropping && !getRoi) 
		{
			cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
			imshow("Original", bgrImg);
		} 
		else if (!cropping && getRoi) 
		{
			cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
			imshow("Original", bgrImg);
		}

		cv::createTrackbar("HMIN", "Original", &Hmin, 255, on_trackbar);
		cv::createTrackbar("HMAX", "Original", &Hmax, 255, on_trackbar);
		cv::createTrackbar("SMIN", "Original", &Smin, 255, on_trackbar);
		cv::createTrackbar("SMAX", "Original", &Smax, 255, on_trackbar);
		cv::createTrackbar("VMIN", "Original", &Vmin, 255, on_trackbar);
		cv::createTrackbar("VMAX", "Original", &Vmax, 255, on_trackbar);
		setMouseCallback("Original", on_mouse, &bgrImg);

		if (getRoi) 
		{
			cv::Rect rect(xmin, ymin, xmax - xmin, ymax - ymin);
			cv::Mat roi = frameWork(rect);
			cv::Mat roiHSV;
			cv::cvtColor(roi, roiHSV, CV_BGR2HSV);
			std::vector<cv::Mat> channels;
			cv::split(roiHSV, channels);
			double minVal, maxVal;
			cv::Point minPos, maxPos;
			cv::minMaxLoc(channels[0], &minVal, &maxVal, &minPos, &maxPos);
			Hmin = minVal;
			Hmax = maxVal;
			cv::minMaxLoc(channels[1], &minVal, &maxVal, &minPos, &maxPos);
			Smin = minVal;
			Smax = maxVal;
			cv::minMaxLoc(channels[2], &minVal, &maxVal, &minPos, &maxPos);
			Vmin = minVal;
			Vmax = maxVal;
			cvSetTrackbarPos("HMIN", "Original", Hmin);
			cvSetTrackbarPos("HMAX", "Original", Hmax);
			cvSetTrackbarPos("SMIN", "Original", Smin);
			cvSetTrackbarPos("SMAX", "Original", Smax);
			cvSetTrackbarPos("VMIN", "Original", Vmin);
			cvSetTrackbarPos("VMAX", "Original", Vmax);
			cv::imshow("Roi", roi);
			cv::imshow("RoiHSV", roiHSV);
			getRoi = false;
		}

		cv::cvtColor(frameWork, frameHSV, CV_BGR2HSV);
		cv::inRange(frameHSV, cv::Scalar(Hmin, Smin, Vmin),cv::Scalar(Hmax, Smax, Vmax), maskRange);
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1.5, 1.5));
		cv::morphologyEx(maskRange, mask, cv::MORPH_ERODE, kernel, cv::Point(-1, -1), 1);
		cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel, cv::Point(-1, -1), 7);

		cv::Mat maskedImage;
		frameWork.copyTo(maskedImage, mask);
		// Compute the centorid mask
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat canny_output;
		mask.copyTo(canny_output);
		cv::imshow("Canny", canny_output);
		cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		if (contours.size() > 0) 
		{
			double maxArea = -1;
			int indexMaxArea = 0;
			for (unsigned int i = 0; i < contours.size(); i++) 
			{
				float area = cv::contourArea(contours[i]);
				if (area > maxArea) 
				{
					maxArea = area;
					indexMaxArea = i;
				}
			}
			std::vector<cv::Point> contour_poly;
			cv::approxPolyDP(cv::Mat(contours[indexMaxArea]), contour_poly, 3,true);
			cv::boundingRect(contour_poly);
			cv::rectangle(maskedImage, cv::boundingRect(contour_poly).tl(),cv::boundingRect(contour_poly).br(), CV_RGB(124, 40, 30), 2, 8, 0);
			cv::Moments centroide = moments(contours[indexMaxArea], false);
			cv::Point punto(centroide.m10 / centroide.m00, centroide.m01 / centroide.m00);
			cv::circle(maskedImage, punto, 4, CV_RGB(124, 40, 30), -1, 8, 0);
			std::cout << cv::Mat(punto) << std::endl;

		}

		imshow("Color mask", mask);
		cv::imshow("Image with mask", maskedImage);

		if(cv::waitKey(1)=='s')
		{
			fs.open(configFile, fs.APPEND);

			fs<< Huemin.str() << Hmin;
			fs<< Huemax.str() << Hmax;
			fs<< Satmin.str() << Smin;
			fs<< Satmax.str() << Smax;
			fs<< Valmin.str() << Vmin;
			fs<< Valmax.str() << Vmax;
			
			fs.release();

			std::cout << colour << " Calibration Completed..." << std::endl;

			Huemin.str(std::string());
			Huemax.str(std::string());
			Satmin.str(std::string());
			Satmax.str(std::string());
			Valmin.str(std::string());
			Valmax.str(std::string());
			
		}
		
		ros::spinOnce();
        loop.sleep();
    }
}

void callbackStartCalibrate(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "CubesSegmentation.->Calibrate colour" << std::endl;
    colour = msg->data;
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat imageHSV;
    cv::Mat imageSegmentada;
    cv::Mat maskRange;

    Huemin << "H_min" << colour;
    Huemax << "H_max" << colour;
    Satmin << "S_min" << colour;
    Satmax << "S_max" << colour;
    Valmin << "V_min" << colour;
    Valmax << "V_max" << colour;

    
    
    std::string configDir = ros::package::getPath("cubes_segmentation") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 

	std::string configFile =configDir + "/Cubes_config.xml";
	cv::FileStorage fs;

	if(!boost::filesystem::exists(configFile))
	{
		fs.open(configFile, fs.WRITE);
		fs.release();	
	}
	
    ros::Rate loop(30);

	while(ros::ok() && cv::waitKey(1) != 'q')
	{
    	GetImagesFromJustina(bgrImg,xyzCloud);
    	imshow("calibrate", bgrImg);
    	createTrackbar("HMIN", "calibrate", &Hmin, 255, on_trackbar);
		createTrackbar("HMAX", "calibrate", &Hmax, 255, on_trackbar);
		createTrackbar("SMIN", "calibrate", &Smin, 255, on_trackbar);
		createTrackbar("SMAX", "calibrate", &Smax, 255, on_trackbar);
		createTrackbar("VMIN", "calibrate", &Vmin, 255, on_trackbar);
		createTrackbar("VMAX", "calibrate", &Vmax, 255, on_trackbar);


		cvtColor(bgrImg, imageHSV, CV_BGR2HSV);
		inRange(imageHSV, Scalar(Hmin, Smin, Vmin),Scalar(Hmax, Smax, Vmax), imageSegmentada);
		erode(imageSegmentada, maskRange, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(maskRange, maskRange,getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		
		imshow("Color mask", maskRange);
		cv::Mat maskedImage;
		bgrImg.copyTo(maskedImage, maskRange);
		imshow("Image with mask", maskedImage);

		if(cv::waitKey(1)=='s')
		{
			fs.open(configFile, fs.APPEND );

			fs<< Huemin.str() << Hmin;
			fs<< Huemax.str() << Hmax;
			fs<< Satmin.str() << Smin;
			fs<< Satmax.str() << Smax;
			fs<< Valmin.str() << Vmin;
			fs<< Valmax.str() << Vmax;
			
			fs.release();

			std::cout << colour << " Calibration Completed..." << std::endl;

			Huemin.str(std::string());
			Huemax.str(std::string());
			Satmin.str(std::string());
			Satmax.str(std::string());
			Valmin.str(std::string());
			Valmax.str(std::string());
			
		}

		ros::spinOnce();
        loop.sleep();
	}
}

bool setDeepthWindow()
{
	std::cout << "CubesSegmentation.->Trying to find a plane" << std::endl;

	point_cloud_manager::GetRgbd srv;
    vision_msgs::FindPlane fp;
    fp.request.name = "";

    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "CubesSegmentation.->Cannot get point cloud :'(" << std::endl;
        return false;
    }

    fp.request.point_cloud = srv.response.point_cloud;

    if(!cltFindPlane.call(fp))
    {
        std::cout << "CubesSegmentation.->Cannot find a plane" << std::endl;
        return false;
    }
    std::cout << "CubesSegmentation.->Find a plane" << std::endl;

    minX = fp.response.nearestPoint.x;
    maxX = minX + 1.0;
    minY = fp.response.nearestPoint.y - 0.5;
    maxY = fp.response.nearestPoint.y + 0.5;
    minZ = fp.response.nearestPoint.z + 0.0005;
    maxZ = fp.response.nearestPoint.z + 1.0;

    std::cout << "minX: " << minX << std::endl;
    std::cout << "minY: " << minY << std::endl;
    std::cout << "minZ: " << minZ << std::endl;
    return true;
} 


bool callback_srvCubeSeg(vision_msgs::GetCubes::Request &req, vision_msgs::GetCubes::Response &resp)
{

	cv::Vec3f aux (0.0, 0.0, 0.0);
	cv::Vec3f centroid (0.0, 0.0, 0.0); 

	cv::Mat bgrImg;
    cv::Mat xyzCloud;
    
    cv::Mat imageHSV;

    GetImagesFromJustina(bgrImg,xyzCloud);

    cv::cvtColor(bgrImg,imageHSV,CV_BGR2HSV);
    cv::Mat globalmask = cv::Mat::zeros(imageHSV.size(),CV_8U);
    cv::bitwise_not(globalmask,globalmask);

    vision_msgs::CubesSegmented cubes = req.cubes_input;
        
    //inRange(imageHSV,Scalar(0,70,50), Scalar(0,255,255),maskHSV);

    vector <cv::Point> centroidList;
    std::vector<std::vector<cv::Point> > contoursRec;
    std::vector<cv::Scalar> colors;
    geometry_msgs::Point minP, maxP;

    vision_msgs::DetectObjects srv;
    if(!cltExtObj.call(srv))
    {
        std::cout << "cubes_segmentation_node.-> Cannot extract a object above planes" << std::endl;
        return false;
    }
    sensor_msgs::ImageConstPtr objExtrMaskConsPtr( new sensor_msgs::Image( srv.response.image ) );
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(objExtrMaskConsPtr, sensor_msgs::image_encodings::TYPE_8UC1);
    cv::Mat objExtrMask = cv_ptr->image;

    for(int i = 0; i < cubes.recog_cubes.size(); i++)
    {

    	minP.x=10.0;
    	minP.y=10.0;
    	minP.z=10.0;
    	maxP.x=0.3;
    	maxP.y=0.3;
    	maxP.z=0.3;

    	cv::Mat maskHSV;
    	vision_msgs::Cube cube = cubes.recog_cubes[i];

    	loadValuesFromFile(cube.color, true);
    	
    	inRange(imageHSV,Scalar(Hmin, Smin, Vmin), Scalar(Hmax,Smax,Vmax),maskHSV);//color rojo
    	cv::Mat maskXYZ;
		cv::inRange(xyzCloud,cv::Scalar(minX, minY,minZ),cv::Scalar(maxX,maxY,maxZ),maskXYZ);
        cv::imshow("In range image", maskXYZ);

		cv::Mat mask;
		maskXYZ.copyTo(mask,maskHSV);
		cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(1.5, 1.5));
		cv::morphologyEx(mask,mask,cv::MORPH_ERODE,kernel,cv::Point(-1,-1),1);
		cv::morphologyEx(mask,mask,cv::MORPH_DILATE,kernel,cv::Point(-1,-1),7);

        cv::bitwise_and(mask, objExtrMask , mask);

		// Compute the centorid mask
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat canny_output;
		mask.copyTo(canny_output);
		cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		if (contours.size() == 0){
			cube.detected_cube  = false;
			cube.cube_centroid.x = 0.0;
			cube.cube_centroid.y = 0.0;
			cube.cube_centroid.z = 0.0;
            resp.cubes_output.recog_cubes.push_back(cube);	
            continue;
        }

        double maxArea = -1;
        int indexMaxArea = 0;
        for (unsigned int contour = 0; contour < contours.size(); contour++) 
        {
            float area = cv::contourArea(contours[contour]);
            if (area > maxArea) 
            {
                maxArea = area;
                indexMaxArea = contour;
            }
        }
        std::vector<cv::Point> contour_poly;
        cv::approxPolyDP(cv::Mat(contours[indexMaxArea]), contour_poly, 3,true);
        //cv::boundingRect(contour_poly);
        cv::Mat boundingMask = cv::Mat::zeros(mask.size(), CV_8U);
        cv::fillConvexPoly(boundingMask, &contour_poly[0], (int)contour_poly.size(), 255, 8, 0);
        cv::bitwise_and(mask, boundingMask , mask);
        cv::imshow("Mask", mask);
        		
		cv::Point imgCentroid(0,0);
		int numPoints = 0;

		bool firstData = false;
        for (int row = 0; row < mask.rows; ++row)
		{
			for (int col = 0; col < mask.cols; ++col)
			{
				if (mask.at<uchar>(row,col)>0)
				{
					//centroid += xyzCloud.at<cv::Vec3f>(i,j);
					aux = xyzCloud.at<cv::Vec3f>(row,col);
					centroid += aux;
					imgCentroid += cv::Point(col,row);
					++numPoints;
    
                    if(!firstData){
                        firstData = true;
                        minP.x = aux.val[0]; 
                        maxP.x = aux.val[0]; 
                        minP.y = aux.val[1]; 
                        maxP.y = aux.val[1]; 
                        minP.z = aux.val[2]; 
                        maxP.z = aux.val[2]; 
                    }
                    else{
                        if(minP.x > aux.val[0])
                            minP.x = aux.val[0];

                        if(minP.y > aux.val[1])
                            minP.y = aux.val[1];

                        if(minP.z > aux.val[2])
                            minP.z = aux.val[2];

                        if(maxP.x < aux.val[0])
                            maxP.x = aux.val[0];

                        if(maxP.y < aux.val[1])
                            maxP.y = aux.val[1];

                        if(maxP.z < aux.val[2])
                            maxP.z = aux.val[2];
                    }
                }
			}
		}

		if (numPoints == 0)
		{
			std::cout << "CubesSegmentation.->Cannot get centroid " << std::endl;
			cube.detected_cube  = false;
			cube.cube_centroid.x = 0.0;
			cube.cube_centroid.y = 0.0;
			cube.cube_centroid.z = 0.0;
		}
		else
		{
			centroid /= numPoints;
			imgCentroid /= numPoints;
			centroidList.push_back(imgCentroid);
            contoursRec.push_back(contour_poly);
			std::cout << "CubesSegmentation.->Centroid:" << centroid << std::endl;
			std::cout << "CubesSegmentation.->MinP:[" << minP << "]" << std::endl;
			std::cout << "CubesSegmentation.->MaxP:[" << maxP << "]" << std::endl;
			cube.detected_cube = true;
			cube.cube_centroid.x = centroid[0];
			cube.cube_centroid.y = centroid[1];
			cube.cube_centroid.z = centroid[2];

			cube.minPoint = minP;
			cube.maxPoint = maxP;

            tf::StampedTransform transform;
            transformListener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
            transformListener->lookupTransform("map", "base_link", ros::Time(0), transform);

            tf::Vector3 cubePosWrtRobot((minP.x + maxP.x) / 2.0f, (minP.y + maxP.y) / 2.0f, (minP.z + maxP.z) / 2.0f);
            tf::Vector3 cubePosWrtWorld = transform * cubePosWrtRobot;

            cv::Mat colorBGR = cv::Mat(1, 1, CV_8UC3);
            cv::Mat colorHSV = cv::Mat(1, 1, CV_8UC3);
            colorHSV.at<cv::Vec3b>(0, 0)[0] = (Hmin + Hmax) / 2.0f;
            colorHSV.at<cv::Vec3b>(0, 0)[1] = (Smin + Smax) / 2.0f;
            colorHSV.at<cv::Vec3b>(0, 0)[2] = (Vmin + Vmax) / 2.0f;
            cv::cvtColor(colorHSV, colorBGR, CV_HSV2BGR);
            colors.push_back(cv::Scalar(colorBGR.at<cv::Vec3b>(0, 0)[0], colorBGR.at<cv::Vec3b>(0, 0)[1], colorBGR.at<cv::Vec3b>(0, 0)[2]));

            cube.colorRGB.x = colorBGR.at<cv::Vec3b>(0, 0)[2] / 255.0f;
            cube.colorRGB.y = colorBGR.at<cv::Vec3b>(0, 0)[1] / 255.0f;
            cube.colorRGB.z = colorBGR.at<cv::Vec3b>(0, 0)[0] / 255.0f;
 
            std::map<std::string, visualization_msgs::Marker>::iterator cubeIt = cubesMapMarker.find(cube.color);
            if(cubeIt == cubesMapMarker.end()){
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time();
                marker.ns = "cubes_marker";
                marker.id = i;
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = cubePosWrtWorld.x();
                marker.pose.position.y = cubePosWrtWorld.y();
                marker.pose.position.z = cubePosWrtWorld.z();
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = fabs(minP.x - maxP.x);
                marker.scale.y = fabs(minP.y - maxP.y);
                marker.scale.z = fabs(minP.z - maxP.z);
                marker.color.a = 0.8;
                marker.color.r = colorBGR.at<cv::Vec3b>(0, 0)[2] / 255.0f;
                marker.color.g = colorBGR.at<cv::Vec3b>(0, 0)[1] / 255.0f;
                marker.color.b = colorBGR.at<cv::Vec3b>(0, 0)[0] / 255.0f;
                cubesMapMarker[cube.color] = marker; 
            }
            else{
                visualization_msgs::Marker marker = cubeIt->second;
                marker.pose.position.x = cubePosWrtWorld.x();
                marker.pose.position.y = cubePosWrtWorld.y();
                marker.pose.position.z = cubePosWrtWorld.z();
                marker.scale.x = fabs(minP.x - maxP.x);
                marker.scale.y = fabs(minP.y - maxP.y);
                marker.scale.z = fabs(minP.z - maxP.z);
                cubesMapMarker[cube.color] = marker; 
            }
		}

		cv::bitwise_not(mask,mask);
		//imshow("mask", mask);
		
		resp.cubes_output.recog_cubes.push_back(cube);	
		mask.copyTo(globalmask, globalmask);
    }
    cv::bitwise_not(globalmask,globalmask);
	//imshow("globalmask", globalmask);
    cv::Mat maskedImage;
	bgrImg.copyTo(maskedImage,globalmask);
	for(int i=0; i<centroidList.size(); i++)
	{
		cv::circle(maskedImage, centroidList[i],5, colors[i], -1);
        cv::rectangle(maskedImage, cv::boundingRect(contoursRec[i]).tl(), cv::boundingRect(contoursRec[i]).br(), colors[i], 2, 8, 0);
	}
	imshow("global",maskedImage);
    return true;
}


bool callback_srvCutlerySeg(vision_msgs::GetCubes::Request &req, vision_msgs::GetCubes::Response &resp)
{

	cv::Vec3f aux (0.0, 0.0, 0.0);
	cv::Vec3f centroid (0.0, 0.0, 0.0); 

	cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat bgrImgCopy;
    cv::Mat imageHSV;

    GetImagesFromJustina(bgrImg,xyzCloud);

    bgrImg.copyTo(bgrImgCopy);
    blur(bgrImgCopy, bgrImgCopy, Size(4, 4) , Point(-1,-1) );
    cv::cvtColor(bgrImgCopy,imageHSV,CV_BGR2HSV);
    cv::Mat globalmask = cv::Mat::zeros(imageHSV.size(),CV_8U);
    cv::bitwise_not(globalmask,globalmask);

    vision_msgs::CubesSegmented cubes = req.cubes_input;
        
    //inRange(imageHSV,Scalar(0,70,50), Scalar(0,255,255),maskHSV);

    vector <cv::Point> centroidList;
    std::vector<std::vector<cv::Point> > contoursRec;
    std::vector<cv::Scalar> colors;
    geometry_msgs::Point minP, maxP;

    vision_msgs::DetectObjects srv;
    if(!cltExtCut.call(srv))
    {
        std::cout << "cutlery_segmentation_node.-> Cannot extract a object with planes" << std::endl;
        return false;
    }
    sensor_msgs::ImageConstPtr objExtrMaskConsPtr( new sensor_msgs::Image( srv.response.image ) );
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(objExtrMaskConsPtr, sensor_msgs::image_encodings::TYPE_8UC1);
    cv::Mat objExtrMask = cv_ptr->image;

    for(int i = 0; i < cubes.recog_cubes.size(); i++)
    {

    	minP.x=10.0;
    	minP.y=10.0;
    	minP.z=10.0;
    	maxP.x=0.3;
    	maxP.y=0.3;
    	maxP.z=0.3;

    	cv::Mat maskHSV;
    	vision_msgs::Cube cube = cubes.recog_cubes[i];

    	loadValuesFromFile(cube.color, false);
    	
    	inRange(imageHSV,Scalar(Hmin, Smin, Vmin), Scalar(Hmax,Smax,Vmax),maskHSV);//color rojo
    	cv::Mat maskXYZ;
		cv::inRange(xyzCloud,cv::Scalar(minX, minY,minZ),cv::Scalar(maxX,maxY,maxZ),maskXYZ);
        cv::imshow("In range image", maskXYZ);

		cv::Mat mask;
		maskXYZ.copyTo(mask,maskHSV);
		cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(1.5, 1.5));
		cv::morphologyEx(mask,mask,cv::MORPH_ERODE,kernel,cv::Point(-1,-1),1);
		cv::morphologyEx(mask,mask,cv::MORPH_DILATE,kernel,cv::Point(-1,-1),7);

        cv::bitwise_and(mask, objExtrMask , mask);

		// Compute the centorid mask
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat canny_output;
		mask.copyTo(canny_output);
		cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		if (contours.size() == 0){
			cube.detected_cube  = false;
			cube.cube_centroid.x = 0.0;
			cube.cube_centroid.y = 0.0;
			cube.cube_centroid.z = 0.0;
            resp.cubes_output.recog_cubes.push_back(cube);	
            continue;
        }

        double maxArea = -1;
        int indexMaxArea = 0;
        for (unsigned int contour = 0; contour < contours.size(); contour++) 
        {
            float area = cv::contourArea(contours[contour]);
            if (area > maxArea) 
            {
                maxArea = area;
                indexMaxArea = contour;
            }
        }
        std::vector<cv::Point> contour_poly;
        cv::approxPolyDP(cv::Mat(contours[indexMaxArea]), contour_poly, 3,true);
        //cv::boundingRect(contour_poly);
        cv::Mat boundingMask = cv::Mat::zeros(mask.size(), CV_8U);
        cv::fillConvexPoly(boundingMask, &contour_poly[0], (int)contour_poly.size(), 255, 8, 0);
        cv::bitwise_and(mask, boundingMask , mask);
        cv::imshow("Mask", mask);

        //cv::rectangle(bgrImg, cv::boundingRect(contour_poly).tl(),cv::boundingRect(contour_poly).br(), CV_RGB(124, 40, 30), 2, 8, 0);
        /*cv::Moments centroide = moments(contours[indexMaxArea], false);
        cv::Point punto(centroide.m10 / centroide.m00, centroide.m01 / centroide.m00);
        cv::circle(maskedImage, punto, 4, CV_RGB(124, 40, 30), -1, 8, 0);*/
        		
		cv::Point imgCentroid(0,0);
		int numPoints = 0;

		bool firstData = false;
        for (int row = 0; row < mask.rows; ++row)
		{
			for (int col = 0; col < mask.cols; ++col)
			{
				if (mask.at<uchar>(row,col)>0)
				{
					//centroid += xyzCloud.at<cv::Vec3f>(i,j);
					aux = xyzCloud.at<cv::Vec3f>(row,col);
					centroid += aux;
					imgCentroid += cv::Point(col,row);
					++numPoints;
    
                    if(!firstData){
                        firstData = true;
                        minP.x = aux.val[0]; 
                        maxP.x = aux.val[0]; 
                        minP.y = aux.val[1]; 
                        maxP.y = aux.val[1]; 
                        minP.z = aux.val[2]; 
                        maxP.z = aux.val[2]; 
                    }
                    else{
                        if(minP.x > aux.val[0])
                            minP.x = aux.val[0];

                        if(minP.y > aux.val[1])
                            minP.y = aux.val[1];

                        if(minP.z > aux.val[2])
                            minP.z = aux.val[2];

                        if(maxP.x < aux.val[0])
                            maxP.x = aux.val[0];

                        if(maxP.y < aux.val[1])
                            maxP.y = aux.val[1];

                        if(maxP.z < aux.val[2])
                            maxP.z = aux.val[2];
                    }
                }
			}
		}

		if (numPoints == 0)
		{
			std::cout << "CutlerySegmentation.->Cannot get centroid " << std::endl;
			cube.detected_cube  = false;
			cube.cube_centroid.x = 0.0;
			cube.cube_centroid.y = 0.0;
			cube.cube_centroid.z = 0.0;
		}
		else
		{
			centroid /= numPoints;
			imgCentroid /= numPoints;
			centroidList.push_back(imgCentroid);
            contoursRec.push_back(contour_poly);
			std::cout << "CutlerySegmentation.->Centroid:" << centroid << std::endl;
			std::cout << "CutlerySegmentation.->MinP:[" << minP << "]" << std::endl;
			std::cout << "CutlerySegmentation.->MaxP:[" << maxP << "]" << std::endl;
			cube.detected_cube = true;
			cube.cube_centroid.x = centroid[0];
			cube.cube_centroid.y = centroid[1];
			cube.cube_centroid.z = centroid[2];

			cube.minPoint = minP;
			cube.maxPoint = maxP;

            tf::StampedTransform transform;
            transformListener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
            transformListener->lookupTransform("map", "base_link", ros::Time(0), transform);

            tf::Vector3 cubePosWrtRobot((minP.x + maxP.x) / 2.0f, (minP.y + maxP.y) / 2.0f, (minP.z + maxP.z) / 2.0f);
            tf::Vector3 cubePosWrtWorld = transform * cubePosWrtRobot;

            cv::Mat colorBGR = cv::Mat(1, 1, CV_8UC3);
            cv::Mat colorHSV = cv::Mat(1, 1, CV_8UC3);
            colorHSV.at<cv::Vec3b>(0, 0)[0] = (Hmin + Hmax) / 2.0f;
            colorHSV.at<cv::Vec3b>(0, 0)[1] = (Smin + Smax) / 2.0f;
            colorHSV.at<cv::Vec3b>(0, 0)[2] = (Vmin + Vmax) / 2.0f;
            cv::cvtColor(colorHSV, colorBGR, CV_HSV2BGR);
            colors.push_back(cv::Scalar(colorBGR.at<cv::Vec3b>(0, 0)[0], colorBGR.at<cv::Vec3b>(0, 0)[1], colorBGR.at<cv::Vec3b>(0, 0)[2]));

            cube.colorRGB.x = colorBGR.at<cv::Vec3b>(0, 0)[2] / 255.0f;
            cube.colorRGB.y = colorBGR.at<cv::Vec3b>(0, 0)[1] / 255.0f;
            cube.colorRGB.z = colorBGR.at<cv::Vec3b>(0, 0)[0] / 255.0f;
            

            float roll, pitch, yaw;
            pcaAnalysis(mask, xyzCloud, roll, pitch, yaw);
            cube.roll = roll;
            cube.pitch = pitch;
            cube.yaw = yaw;

            cube.type_object = (roll==0.0 && pitch==0 && yaw==0.0) ? 1 : 0;
            /*if(roll==0.0 && pitch==0 && yaw==0.0)
            	cube.type_object = 0;
            else
            	cube.type_object = 1;*/

            std::map<std::string, visualization_msgs::Marker>::iterator cubeIt = cubesMapMarker.find(cube.color);
            if(cubeIt == cubesMapMarker.end()){
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time();
                marker.ns = "cubes_marker";
                marker.id = i;
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = cubePosWrtWorld.x();
                marker.pose.position.y = cubePosWrtWorld.y();
                marker.pose.position.z = cubePosWrtWorld.z();
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = fabs(minP.x - maxP.x);
                marker.scale.y = fabs(minP.y - maxP.y);
                marker.scale.z = fabs(minP.z - maxP.z);
                marker.color.a = 0.8;
                marker.color.r = colorBGR.at<cv::Vec3b>(0, 0)[2] / 255.0f;
                marker.color.g = colorBGR.at<cv::Vec3b>(0, 0)[1] / 255.0f;
                marker.color.b = colorBGR.at<cv::Vec3b>(0, 0)[0] / 255.0f;
                cubesMapMarker[cube.color] = marker; 
            }
            else{
                visualization_msgs::Marker marker = cubeIt->second;
                marker.pose.position.x = cubePosWrtWorld.x();
                marker.pose.position.y = cubePosWrtWorld.y();
                marker.pose.position.z = cubePosWrtWorld.z();
                marker.scale.x = fabs(minP.x - maxP.x);
                marker.scale.y = fabs(minP.y - maxP.y);
                marker.scale.z = fabs(minP.z - maxP.z);
                cubesMapMarker[cube.color] = marker; 
            }
		}

		cv::bitwise_not(mask,mask);
		//imshow("mask", mask);
		
		resp.cubes_output.recog_cubes.push_back(cube);	
		mask.copyTo(globalmask, globalmask);
    }
    cv::bitwise_not(globalmask,globalmask);
	//imshow("globalmask", globalmask);
    cv::Mat maskedImage;
	bgrImg.copyTo(maskedImage,globalmask);
	for(int i=0; i<centroidList.size(); i++)
	{
		cv::circle(maskedImage, centroidList[i],5, colors[i], -1);
        cv::rectangle(maskedImage, cv::boundingRect(contoursRec[i]).tl(), cv::boundingRect(contoursRec[i]).br(), colors[i], 2, 8, 0);
	}
	imshow("global",maskedImage);
    return true;
}



int main(int argc, char** argv)
{
	
    std::cout << "Initializing Cubes Segmentation by Hugo..." << std::endl;
    ros::init(argc, argv, "cubes_segmentation");
    ros::NodeHandle n;
    node = &n;
    
    srvCubesSeg = n.advertiseService("/vision/cubes_segmentation/cubes_seg", callback_srvCubeSeg);
    srvCutlerySeg = n.advertiseService("/vision/cubes_segmentation/cutlery_seg", callback_srvCutlerySeg);
    cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    cltFindPlane = n.serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/findPlane");
    cltExtObj = n.serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/ext_objects_above_planes");
    cltExtCut = n.serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/ext_objects_with_planes");
    ros::Subscriber subStartCalib = n.subscribe("/vision/cubes_segmentation/start_calib", 1, callbackStartCalibrate);
    ros::Subscriber subCalibV2 = n.subscribe("/vision/cubes_segmentation/calibv2", 1, callbackCalibrateV2);
    ros::Subscriber subCalibCutlery = n.subscribe("/vision/cubes_segmentation/calibCutlery", 1, callbackCalibrateCutlery);
    ros::Publisher pubCubesMarker = n.advertise<visualization_msgs::MarkerArray>("/vision/cubes_segmentation/cubes_markers", 1);

    ros::Rate loop(30);
    
    std::cout << "CubesSegmentation.->Running..." << std::endl;
    
    transformListener = new tf::TransformListener();
    
    while(ros::ok() && cv::waitKey(1) != 'q')
    {
        /*cubesMarker.markers.clear();
        for(std::map<std::string, visualization_msgs::Marker>::iterator it = cubesMapMarker.begin(); it != cubesMapMarker.end(); it++)
            cubesMarker.markers.push_back(it->second);
        pubCubesMarker.publish(cubesMarker);*/
        ros::spinOnce();
        loop.sleep();
    }
    
    delete transformListener;

    cv::destroyAllWindows();   
}
