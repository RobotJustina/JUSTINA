#include <cstdlib>
#include <string>
#include <vector>
#include <sstream>


#include <boost/bind.hpp>
#include "RosNode.h"
#include "QRDecoder.h"

// Topic where camera images came from.
#define IMAGE_SOURCE "/hardware/point_cloud_man/rgbd_wrt_robot"

// Topic for publishing recognized text in QR codes.
#define RECOGNIZED_TEXT "qr/recognized"

using namespace qr_reader;

// void decodeAndSend(cv_bridge::CvImageConstPtr& imgPtr);
// void imageCallback(const sensor_msgs::ImageConstPtr& msg);
// cv_bridge::CvImageConstPtr rosImgPtrToOpenCV(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv) {
	QRDecoder decoder;
	ros::init(argc, argv, "qr_reader");

	RosNode node(argc, argv, IMAGE_SOURCE, RECOGNIZED_TEXT);
	node.addImageReceivedHandler(
		boost::bind(&QRDecoder::beginRecognize, &decoder, _1));
	decoder.addTextRecognizedHandler(
		boost::bind(&RosNode::publishRecognizedText, &node, _1));

    std::cout << "ROS QR Reader using " << decoder.info() << std::endl;

	decoder.runAsync();
	node.spin();

	return 0;
}