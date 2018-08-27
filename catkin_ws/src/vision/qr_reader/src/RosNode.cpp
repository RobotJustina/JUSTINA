/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** RosNode.cpp
** Class implementation for a ROS node to be used with the QR code detector
** It creates the ROS node, configures the ROS envoronment, sets up publishers
** and subscribers, and manages the execution loop.
** 
** Author: Mauricio Matamoros
** -------------------------------------------------------------------------*/
#include "RosNode.h"
#include <boost/bind.hpp>
#include <std_msgs/String.h>
#include "justina_tools/JustinaTools.h"

using namespace qr_reader;

RosNode::RosNode(int argc, char** argv, const std::string& image_src, const std::string& recognized_text) :
		mainThread(NULL),
		it(nh){
	text_publisher = nh.advertise<std_msgs::String>(recognized_text, 1);
	this->image_src = image_src;
	subQRStart = nh.subscribe("/vision/qr/start_qr", 1, &RosNode::imageQRStartCallback ,this);
	//subPointCloud = nh.subscribe(image_src, 1, &RosNode::imageCallback ,this);
	JustinaTools::setNodeHandle(&nh);
}

void RosNode::addImageReceivedHandler(const cvImgFunctionType& handler){
	imageReceived.connect(handler);
}

void RosNode::publishRecognizedText(const std::string& text){
	std_msgs::String msg;
	msg.data = text;
	text_publisher.publish(msg);
}

void RosNode::imageCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	if(imageReceived.empty())
		return;
	cv_bridge::CvImageConstPtr imgPtr;
	cv_bridge::CvImage * img = new cv_bridge::CvImage();
	try{
		cv::Mat bgrImg;
    	cv::Mat xyzCloud;
    	JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
		img->header = msg->header;
		img->encoding = sensor_msgs::image_encodings::BGR8;
		img->image = bgrImg;
		imgPtr = cv_bridge::CvImageConstPtr(img);
		//imgPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		//imgMsgToCv(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("Error while converting ros image message: %s", e.what());
		return;
	}
	catch( ... ) {
		ROS_ERROR("Error while converting ros image message.");
		return;
	}
	imageReceived(imgPtr);
}

void RosNode::imageQRStartCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data){
		subPointCloud = nh.subscribe(image_src, 1, &RosNode::imageCallback ,this);
	}
	else{
		subPointCloud.shutdown();
	}
}

void RosNode::mainThreadTask(){
	ros::spin();
}

void RosNode::runAsyncSpin(){
	if(mainThread != NULL)
		return;
	mainThread = new boost::thread(&RosNode::mainThreadTask, this);
}

void RosNode::spin(){
	if(mainThread != NULL)
		return;
	ros::spin();
}

