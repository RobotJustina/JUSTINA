#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "pcl_conversions/pcl_conversions.h"

#include "std_msgs/Bool.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/DetectObjects.h"
#include "vision_msgs/TrainObject.h"
#include "vision_msgs/VisionObjectList.h"
#include "justina_tools/JustinaTools.h"

#include "ObjExtractor.hpp"
#include "DetectedObject.hpp"
#include "ObjRecognizer.hpp"

cv::VideoCapture kinect;
cv::Mat lastImaBGR;
cv::Mat lastImaPCL;

ObjRecognizer objReco; 

bool debugMode = false; 
bool useCVKinect = false; 

bool enableDetectWindow = false; 
bool enableRecognizeTopic = false; 
std::string dirToSaveFiles = ""; 

void GetParams(int argc, char** argv);

ros::Publisher pubRecognizedObjects; 

ros::Subscriber subPointCloud;
void callback_subPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg); 

ros::Subscriber subEnableDetectWindow; 
void callback_subEnableDetectWindow(const std_msgs::Bool::ConstPtr& msg);

ros::Subscriber subEnableRecognizeTopic; 
void callback_subEnableRecognizeTopic(const std_msgs::Bool::ConstPtr& msg); 

ros::ServiceServer srvDetectObjs; 
bool callback_srvDetectObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);

ros::ServiceServer srvTrainObject;
bool callback_srvTrainObject(vision_msgs::TrainObject::Request &req, vision_msgs::TrainObject::Response &resp);

ros::NodeHandle* node;

int main(int argc, char** argv)
{
	std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS" << std::endl;
	GetParams(argc, argv); 

	// Initializing ROS node
	ros::init(argc, argv, "obj_reco_node");
	ros::NodeHandle n;

	subPointCloud = n.subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, callback_subPointCloud);
	subEnableDetectWindow = n.subscribe("/vision/obj_reco/enableDetectWindow", 1, callback_subEnableDetectWindow);
	subEnableRecognizeTopic = n.subscribe("/vision/obj_reco/enableRecognizeTopic", 1, callback_subEnableRecognizeTopic); 

	pubRecognizedObjects = n.advertise<vision_msgs::VisionObjectList>("/vision/obj_reco/recognizedObjectes",1); 

	srvDetectObjs = n.advertiseService("/vision/obj_reco/det_objs", callback_srvDetectObjects);
	srvTrainObject = n.advertiseService("/vision/obj_reco/trainObject", callback_srvTrainObject); 

	ros::Rate loop(10);

	// Getting Objects to train
 	objReco = ObjRecognizer(18); 
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
		else if( params == "-f" )
		{
			dirToSaveFiles = argv[i+1];
			std::cout << "-> DirToSaveFiles: " << dirToSaveFiles << std::endl; 
		}
	}
}

bool callback_srvTrainObject(vision_msgs::TrainObject::Request &req, vision_msgs::TrainObject::Response &resp)
{
	if( req.name == "" )
	{
		std::cout << "WARNING !: objects must have a name to be trained" << std::cout; 
		return false; 
	}

	cv::Mat imaBGR = lastImaBGR.clone();  
	cv::Mat imaPCL = lastImaPCL.clone(); 

	ObjExtractor::DebugMode = debugMode; 
	std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(imaPCL); 

	if( detObjList.size() > 0 )
		objReco.TrainObject( detObjList[0], imaBGR, req.name ); 

	std::cout << "Training Success" << req.name << std::endl; 
	return true; 
}


void callback_subPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat bgrImage;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImage, xyzCloud);

	lastImaBGR = bgrImage.clone(); 
	lastImaPCL = xyzCloud.clone(); 

	if( enableDetectWindow || enableRecognizeTopic )
	{
		ObjExtractor::DebugMode = debugMode; 
		std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(xyzCloud); 

		vision_msgs::VisionObjectList objList; 
		for( int i=0; i<detObjList.size(); i++)
		{
			if( i == 0 )
				cv::rectangle( bgrImage, detObjList[i].boundBox, cv::Scalar(255,0,0), 2); 
			else
				cv::rectangle( bgrImage, detObjList[i].boundBox, cv::Scalar(0,0,255), 2); 

			if( enableRecognizeTopic )
			{
				std::string objName = objReco.RecognizeObject( detObjList[i], lastImaBGR ); 				
				vision_msgs::VisionObject obj; 

				obj.id = objName;
				obj.pose.position.x = detObjList[i].centroid.x;
				obj.pose.position.y = detObjList[i].centroid.y;
				obj.pose.position.z = detObjList[i].centroid.z;

				objList.ObjectList.push_back(obj);
			}
		}
		
		if( enableRecognizeTopic )
			pubRecognizedObjects.publish( objList ); 

		if( enableDetectWindow )
			cv::imshow("Detected Objects", bgrImage);
	}
}

bool callback_srvDetectObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp)
{
	cv::Mat imaBGR = lastImaBGR.clone();  
	cv::Mat imaPCL = lastImaPCL.clone(); 

	ObjExtractor::DebugMode = debugMode; 
	std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(imaPCL); 
	
	cv::Mat imaToShow = lastImaBGR.clone();
	for( int i=0; i<detObjList.size(); i++)
	{
		std::string objName = objReco.RecognizeObject( detObjList[i], imaBGR ); 

		cv::rectangle(imaToShow, detObjList[i].boundBox, cv::Scalar(0,0,255) ); 
		cv::putText(imaToShow, objName, detObjList[i].boundBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );

		if( objName == "" )
			continue; 

		if( dirToSaveFiles != "" )
		{
			cv::Mat imaToSave = imaBGR.clone(); 
			cv::rectangle(imaToSave, detObjList[i].boundBox, cv::Scalar(0,0,255) ); 
			cv::putText(imaToSave, objName, detObjList[i].boundBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );
			cv::imwrite( dirToSaveFiles + objName + ".png", imaToSave); 
		}

		vision_msgs::VisionObject obj; 
		obj.id = objName;
		obj.pose.position.x = detObjList[i].centroid.x;
		obj.pose.position.y = detObjList[i].centroid.y;
		obj.pose.position.z = detObjList[i].centroid.z;

		resp.recog_objects.push_back(obj); 
 	}
 	
	cv::imshow( "Recognized Objects", imaToShow ); 
	return true; 
}

bool callback_srvRecognizeObjects(vision_msgs::RecognizeObjects::Request &req, vision_msgs::RecognizeObjects::Response &resp)
{

	boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
	msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot", ros::Duration(1.0) ) ; 
	if( msg == NULL )
	{
	  std::cout << "det_objs TIMEOUT" << std::endl; 
	  return false;
	}

	sensor_msgs::PointCloud2 pc2 = * msg;

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

void callback_subEnableDetectWindow(const std_msgs::Bool::ConstPtr& msg)
{
	enableDetectWindow = msg->data;

	if( !enableDetectWindow )
		cv::destroyAllWindows(); 
}

void callback_subEnableRecognizeTopic(const std_msgs::Bool::ConstPtr& msg)
{
	enableRecognizeTopic = msg->data; 
}
