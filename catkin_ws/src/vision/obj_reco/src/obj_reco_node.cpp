#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "pcl_conversions/pcl_conversions.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/DetectObjects.h"
#include "justina_tools/JustinaTools.h"
#include "ObjExtractor.hpp"

cv::VideoCapture kinect;
cv::Mat imageBGR;
cv::Mat pointCloud;

bool debugMode = true; 
bool useCVKinect = false; 

void GetParams(int argc, char** argv);
bool callback_RecognizeObjects(vision_msgs::RecognizeObjects::Request &req, vision_msgs::RecognizeObjects::Response &resp);
bool callback_DetectObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);

ros::NodeHandle* node;

int main(int argc, char** argv)
{
	std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS" << std::endl;

	// Checking Params
	GetParams(argc, argv);

	// Initializing kinect with opencv
	if(useCVKinect)
	{
		// Opening kienct as camera
		kinect.open(CV_CAP_OPENNI);
		// For matching bgr and pointscloud
		kinect.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION , CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON);
		// First images are always wrong 
		if( kinect.isOpened() )
		{
			kinect.grab();
			kinect.retrieve(imageBGR, CV_CAP_OPENNI_BGR_IMAGE);
			kinect.retrieve(pointCloud, CV_CAP_OPENNI_POINT_CLOUD_MAP);
		}
		else
		{
			std::cout << "Cant open kinect sensor!" << std::endl;
			kinect.release();
			return 0; 
		}
	}

	// Initializing ROS node
	ros::init(argc, argv, "obj_reco_node");
	ros::NodeHandle n;
	ros::ServiceServer srvSrvRecogObjs = n.advertiseService("recog_objects", callback_RecognizeObjects);
	ros::ServiceServer srvDetectObjs = n.advertiseService("det_objs", callback_DetectObjects);
	ros::Rate loop(10);

	// Principal loop
	char keyStroke = 0; 
	while(ros::ok())
	{
		// FOR DEBUG ONLYYYYYYYYY
		if(useCVKinect)
		{
			if( !kinect.grab() )
			{
				std::cout << "Cant grab images from kinect." << std::endl; 
			}
			else
			{
				// color image (CV_8UC3)
				kinect.retrieve(imageBGR, CV_CAP_OPENNI_BGR_IMAGE);			
				// XYZ in meters (CV_32FC3)
				kinect.retrieve(pointCloud, CV_CAP_OPENNI_POINT_CLOUD_MAP);	
				// showing images 
				cv::imshow("imageBGR", imageBGR);
				cv::imshow("pointCloud", pointCloud);				
				
				keyStroke = cv::waitKey(5); 
			}
		}    

		// ROS 
		ros::spinOnce();
		loop.sleep();
	}
	cv::destroyAllWindows(); 
	return 0;
}

void GetParams(int argc, char** argv)
{
	for( int i=0; i<argc; i++)
	{
		std::string params( argv[i] );

		if( params == "-d" )
		{
			debugMode = true;
			std::cout << "-> DebugMode ON" << std::endl; 
		}
		else if( params == "-k" )
		{
			useCVKinect = true; 
			std::cout << "-> Using CV Kinect" << std::endl; 
		}
	}
}

bool callback_DetectObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp)
{
	boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
	msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot", ros::Duration(1.0) ) ; 
	if( msg == NULL )
	{
	  std::cout << "det_objs TIMEOUT" << std::endl; 
	  return false;
	}

	sensor_msgs::PointCloud2 pc2 = * msg;

	cv::Mat ima; 
	cv::Mat pcl; 
	JustinaTools::PointCloud2Msg_ToCvMat(pc2, ima, pcl); 
	
	if( debugMode )
	{
		cv::imshow("ima", ima);
		cv::imshow("pcl", pcl);		
	}
	
	ObjExtractor::DebugMode = true; 
	ObjExtractor::GetObjectsInHorizontalPlanes(pcl); 

	cv::waitKey(-1); 
	cv::destroyAllWindows();
	std::cout << "Terminated DetectObjects" << std::endl; 
	return true; 
}

bool callback_RecognizeObjects(vision_msgs::RecognizeObjects::Request &req, vision_msgs::RecognizeObjects::Response &resp)
{
	//std::vector<std::pair< double, std::string> > recog_objects;
	//cv::Mat bgrImage;
	//cv::Mat pointCloud;

	//Transform from PointCloud2 (ros msg) to cv::Mat format
	
	vision_msgs::VisionObject obj1;
	obj1.id= "Milk";
	resp.recog_objects.push_back(obj1);

	vision_msgs::VisionObject obj2;
	obj2.id= "Frutastica";
	resp.recog_objects.push_back(obj2);

	//std::vector< DetectedObject > detObj = objExt.ExtractObjectsHorizantalPlanes(bgrImage, pointCloud, detectedObj); 
	std::cout << "HW" << std::endl; 
	return true; 
}
