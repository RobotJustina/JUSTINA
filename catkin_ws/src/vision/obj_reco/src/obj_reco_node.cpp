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
#include "vision_msgs/TrainObject.h"

#include "justina_tools/JustinaTools.h"

#include "ObjExtractor.hpp"
#include "DetectedObject.hpp"
#include "ObjRecognizer.hpp"

cv::VideoCapture kinect;
cv::Mat lastImaBGR;
cv::Mat lastImaPCL;

ObjRecognizer objReco; 

std::string outWinName = "Reco Obj - Output Window"; 
bool debugMode = true; 
bool useCVKinect = false; 

void GetParams(int argc, char** argv);

void callback_tpcPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg); 

bool callback_srvRecognizeObjects(vision_msgs::RecognizeObjects::Request &req, vision_msgs::RecognizeObjects::Response &resp);
bool callback_srvDetectObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);
bool callback_srvTrainObject(vision_msgs::TrainObject::Request &req, vision_msgs::TrainObject::Response &resp);

ros::NodeHandle* node;

int main(int argc, char** argv)
{
	std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS" << std::endl;



	// Initializing ROS node
	ros::init(argc, argv, "obj_reco_node");
	ros::NodeHandle n;

	ros::Subscriber tpcPointCloud = n.subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, callback_tpcPointCloud);

	ros::ServiceServer srvRecogObjs = n.advertiseService("recog_objects", callback_srvRecognizeObjects);
	ros::ServiceServer srvDetectObjs = n.advertiseService("det_objs", callback_srvDetectObjects);
	ros::ServiceServer srvTrainObject = n.advertiseService("trainObject", callback_srvTrainObject); 
	ros::Rate loop(10);

	//
 	objReco= ObjRecognizer(18); 
	objReco.LoadTrainingDir(); 

	// Principal loop
	char keyStroke = 0; 
	while(ros::ok())
	{
		// ROS 
		ros::spinOnce();
		loop.sleep();

		if( cv::waitKey(5) == 'q' )
			break; 
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

bool callback_srvTrainObject(vision_msgs::TrainObject::Request &req, vision_msgs::TrainObject::Response &resp)
{
	ObjExtractor::DebugMode = false; 
	std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(lastImaPCL); 

	if( detObjList.size() > 0 )
	{
		objReco.TrainObject( detObjList[0], lastImaBGR, req.name ); 
	}

	std::cout << "Training Object (name=" << req.name << std::endl; 
	return true; 
}


void callback_tpcPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat bgrImage;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImage, xyzCloud);

	bgrImage.copyTo(lastImaBGR); 
	xyzCloud.copyTo(lastImaPCL); 

	ObjExtractor::DebugMode = false; 
	std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(xyzCloud); 

	for( int i=0; i<detObjList.size(); i++)
	{
		if( i == 0 )
			cv::rectangle( bgrImage, detObjList[i].boundBox, cv::Scalar(255,0,0), 2); 
		else
			cv::rectangle( bgrImage, detObjList[i].boundBox, cv::Scalar(0,255,0), 2); 
	}

	cv::imshow("Detected Objects", bgrImage); 
}

bool callback_srvDetectObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp)
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
	
	ObjExtractor::DebugMode = false; 
	std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(pcl); 

	for( int i=0; i<detObjList.size(); i++)
	{
		cv::rectangle( ima, detObjList[i].boundBox, cv::Scalar(0,255,0), 2); 

		vision_msgs::VisionObject obj; 
		std::stringstream ss; 
		ss << "unknown_" << i ; 
		obj.id = ss.str(); 
		obj.pose.position.x = detObjList[i].centroid.x;
		obj.pose.position.y = detObjList[i].centroid.y;
		obj.pose.position.z = detObjList[i].centroid.z;

		resp.recog_objects.push_back(obj); 
 	}
 	
	cv::imshow( outWinName, ima ); 
	cv::waitKey(10); 
	//cv::destroyAllWindows();
	std::cout << "Terminated Detect Objects (TEST)" << std::endl; 
	return true; 
}

bool callback_srvRecognizeObjects(vision_msgs::RecognizeObjects::Request &req, vision_msgs::RecognizeObjects::Response &resp)
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
