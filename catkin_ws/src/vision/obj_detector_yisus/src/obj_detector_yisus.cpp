#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "pcl_conversions/pcl_conversions.h"
#include "vision_msgs/recognize_objects.h"
#include "vision_msgs/recognize_object.h"
#include "Utils/ObjectsExtractor.hpp"
#include "Utils/HuObjectRecognizer.hpp"
#include "Utils/PersonExtractor.hpp"
#include "Utils/ColorObjectRecognizer.hpp"

ObjectsExtractor objExt = ObjectsExtractor(); 
ColorObjectRecognizer colorReco = ColorObjectRecognizer( "Objects", "colorObjects", 18 ); 
HuObjectRecognizer huReco = HuObjectRecognizer(); 

int TrainFromFile(ColorObjectRecognizer& colorReco, HuObjectRecognizer& shapeReco, std::string workingFolder)
{
    std::string nodeName = "obj"; 
	std::string trainingFileName = workingFolder + "TrainedObjects.xml";
	try
	{
		std::cout << "Trying to load from file: " << trainingFileName << std::endl;  

		int idx = 0; 
		cv::FileStorage fs; 	

		std::vector< std::string > names; 
		std::vector< float > heights; 
		std::vector< std::vector < cv::Point2f > > contours; 
		std::vector< cv::Mat > histos; 

		if( fs.open( trainingFileName , fs.READ ) )
		{
			cv::FileNode contoursNode = fs[ nodeName ]; 
			cv::FileNodeIterator it = contoursNode.begin(); 
			cv::FileNodeIterator it_end = contoursNode.end(); 

			for( ; it != it_end ; ++it, idx++ )
			{
				std::string oName = (*it)["name"]; 
				double oHeight = (double)(*it)["height"]; 
				
				std::vector < cv::Point2f > oCont; 
				(*it)["contour2d"] >> oCont; 

				cv::Mat oHist; 
				(*it)["histogram"] >> oHist; 

				std::string nameWO_ext = oName.substr(0, oName.find_first_of("_") ); 

				names.push_back( nameWO_ext ); 
				heights.push_back( oHeight ); 
				contours.push_back( oCont ); 
				histos.push_back( oHist ); 
			}
			fs.release(); 
		}
		else
		{
			std::cout << "ColorObjectRecognizer : Can't Load Training File (" << trainingFileName << ")" << std::endl; 
			return false;
		}
		std::cout << "ColorObjectRecognizer.-> Trained objects: " << std::endl;
		for( int i = 0; i<names.size(); i++ )
		{
			colorReco.trainingNames.push_back( names[i] );
			colorReco.trainingHistograms.push_back( histos[i] ); 

			shapeReco.trainNames.push_back( names[i] ); 
			shapeReco.trainHeights.push_back( heights[i] ); 
			shapeReco.trainShadowsCont2D.push_back( contours[i] ); 
			std::cout << "Object_" << i << ": " << names[i] << std::endl;
		}
		std::cout <<"ColorObjectRecognizer: Objects trained succesfully!!! :D" << std::endl;
		return true; 
	}
	catch(std::exception ex)
    {
		std::cout << "ColorObjectRecognizer: Can't LoadTraining File (" << trainingFileName << ")" << " ex:" << ex.what() <<std::endl;
		return false; 
	}
}

std::pair<double, std::string> RecognizeObject( std::vector< std::pair< double, std::string> > sizeError, std::vector< std::pair< double, std::string> > shapeError , std::vector< std::pair< double, std::string> > colorError , double& outErrorSize , double& outErrorShape , double& outErrorColor)
{
	double sizeThres = 0.02;
	double shapeThres = 0.051; 
	double colorThres = 0.6; 

	std::pair<double, std::string> bestSoFar(-1000.00, ""); 
	
	for( int i=0; i<sizeError.size(); i++)
	{
		bool sizePass = sizeThres > sizeError[i].first; 
		bool shapePass = shapeThres > shapeError[i].first; 
		bool colorPass = colorThres < colorError[i].first; 

		if( sizePass && shapePass && colorPass )
		{
			if(colorError[i].first > bestSoFar.first)
			{
				bestSoFar.first = colorError[i].first; 
				bestSoFar.second = colorError[i].second; 

				outErrorSize = sizeError[i].first; 
				outErrorShape = shapeError[i].first; 
				outErrorColor = colorError[i].first;
			}
		}
	}

	return bestSoFar; 	 
}

void PointCloud2msg_ToCvMat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{
	//std::cout << "ObjectDetectorNode.-> Transforming from PointCloud2 ros message to cv::Mat type" << std::endl;
	//std::cout << "ObjectDetectorNode.-> Width= " << pc_msg.width << "  height= " << pc_msg.height << std::endl;
	pcl::PointCloud<pcl::PointXYZRGBA> pc_pcl;
	pcl::fromROSMsg(pc_msg, pc_pcl);  //Transform from PointCloud2 msg to pointCloud (from pcl) type

	if(!pc_pcl.isOrganized())
	{
		std::cout << "ObjectDetectorNode.->Point cloud is not organized!! O M G!!!!" << std::endl;
		return;
	}
	//std::cout << "ObjectDetectorNode.-> Pcl_w= " << pc_pcl.width << "  pcl_h= " << pc_pcl.height << std::endl;
	bgr_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_8UC3);
	pc_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_32FC3);
	
	pcl::PointXYZRGBA p_ = pc_pcl.at(320, 240);
	//std::cout << "ObjectDetectorNode: Middle point: " << p_.x << "  " << p_.y << "  " << p_.z << "  " << p_.r << "  " << p_.g << "  " << p_.b << std::endl;
	for (int h=0; h<bgr_dest.rows; h++)
		for (int w=0; w<bgr_dest.cols; w++)
		{
			pcl::PointXYZRGBA p = pc_pcl.at(w, h);
			bgr_dest.data[h*bgr_dest.step + w*3] = (unsigned char)p.b;
			bgr_dest.data[h*bgr_dest.step + w*3 + 1] = (unsigned char)p.g;
			bgr_dest.data[h*bgr_dest.step + w*3 + 2] = (unsigned char)p.r;
			pc_dest.at<cv::Vec3f>(h,w)[0] = p.x;
			pc_dest.at<cv::Vec3f>(h,w)[1] = p.y;
			pc_dest.at<cv::Vec3f>(h,w)[2] = p.z;
		}

}

bool GetParams(int argc, char** argv, std::string& workingFolder)
{
	bool correctParams = false;
    workingFolder = "";
    for (int i = 0; i < argc; i++)
	{
		//std::cout << "Param: " << argv[i] << std::endl;
		std::string strParam(argv[i]);
		if (strParam.compare("-f") == 0)
		{
			workingFolder = argv[++i];
			correctParams = true;
		}
	}
	if (!correctParams)
	{
		std::cout << "TOO FEW PARAMETERS!!" << std::endl;
		std::cout << "Apparently you are not qualified to use the OBJECT_DETECTOR_NODE." << std::endl << "Usage:" << std::endl;
		std::cout << "-f your_path/workingFolder/" << std::endl;
		std::cout << "But please, keep trying.... " << std::endl;
		return false;
	}
	return correctParams;
}

bool callback_recog_objects(vision_msgs::recognize_objects::Request &req, vision_msgs::recognize_objects::Response &resp)
{
    std::vector<std::pair< double, std::string> > recog_objects;
    cv::Mat bgrImage;
	cv::Mat pointCloud;
    //Transform from PointCloud2 (ros msg) to cv::Mat format
    PointCloud2msg_ToCvMat(req.point_cloud, bgrImage, pointCloud);
    cv::Mat detectedObj;		
	std::vector< DetectedObject > detObj = objExt.ExtractObjectsHorizantalPlanes(bgrImage, pointCloud, detectedObj); 
}

bool callback_recog_object(vision_msgs::recognize_object::Request &req, vision_msgs::recognize_object::Response &resp)
{
}

int main(int argc, char** argv)
{
    std::string workingFolder = "";
	if(!GetParams(argc, argv, workingFolder))
		return EXIT_FAILURE;
    
    std::cout << "INITIALIZING OBJECT DECTECTOR BY MR YISUS... " << std::endl;
    ros::init(argc, argv, "object_detector_yisus");
    ros::NodeHandle n;
    ros::ServiceServer srvSrvRecogObjs = n.advertiseService("recog_objects", callback_recog_objects);
    ros::ServiceServer srvSrvRecogObj = n.advertiseService("recog_object", callback_recog_object);
    ros::Rate loop(10);

    colorReco.debugMode = false; 
	colorReco.minErrorForRecognition = 100000.00; 
	huReco.debugMode = false; 
	huReco.maxErrorForRecognition = 9999.0; 
	objExt.Debug = false; 

	TrainFromFile(colorReco, huReco, workingFolder); 

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
