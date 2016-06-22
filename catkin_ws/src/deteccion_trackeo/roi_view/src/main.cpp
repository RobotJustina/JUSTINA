#include "ros/ros.h"//Used for launch file parameter parsing
#include <string>//Used for rois message vector
#include <vector>	
#include <sstream>
#include "ros/console.h"
//Included for files
#include <iostream>
#include <fstream>
#include "stdio.h"
#include "dirent.h"

//Publish Messages
#include "roi_msgs/RoiRect.h"
#include "roi_msgs/Rois.h"
#include "std_msgs/String.h"

//Time Synchronizer
// NOTE: Time Synchronizer conflicts with QT includes may need to investigate
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

//Subscribe Messages
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// Image Transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// Used to display OPENCV images
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace sensor_msgs::image_encodings;
using namespace roi_msgs;
using namespace cv;

cv::Scalar darkBlue(130,0,0);
cv::Scalar white(255,255,255);

class roiViewerNode
{
private:
	
	ros::NodeHandle node_;

	message_filters::Subscriber<Image> sub_image_;
	message_filters::Subscriber<Rois> sub_rois_;

	
	typedef ApproximateTime<Image, Rois> ApproximatePolicy;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
	boost::shared_ptr<ApproximateSync> approximate_sync_;

	int label;
	bool show_confidence;

public:

	explicit roiViewerNode(const ros::NodeHandle& nh):
	node_(nh)
	{

		label = 0;
		show_confidence = false;

		//Read mode from launch file
		std::string mode="";
		node_.param(ros::this_node::getName() + "/mode", mode, std::string("none"));
		ROS_INFO("Selected mode: %s",mode.c_str());

		if(mode.compare("roi_display")==0){
			ROS_INFO("MODE: %s",mode.c_str());

			//Get the image width and height
			node_.param(ros::this_node::getName()+"/label",label,0);

			//Read parameter for showing roi confidence
			node_.param(ros::this_node::getName()+"/show_confidence",show_confidence,false);

			// Subscribe to Messages
			sub_image_.subscribe(node_,"input_image",20);
			sub_rois_.subscribe(node_,"input_rois",20);

			// Sync the Synchronizer
			approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(20),
					sub_image_,
					sub_rois_));

			approximate_sync_->registerCallback(boost::bind(&roiViewerNode::imageCb,
					this,
					_1,
					_2));
		}else{

			ROS_INFO("Unknown mode:%s  Please set to {roi_display} in roiViewer.launch",mode.c_str());
		}
		// Visualization
		cv::namedWindow("Detections", 0 ); // non-autosized
		cv::startWindowThread();

	}

	void imageCb(const ImageConstPtr& image_msg,
			const RoisConstPtr& rois_msg){

		std::string filename = image_msg->header.frame_id.c_str();
		std::string imgName = filename.substr(filename.find_last_of("/")+1);

		ROS_INFO("roiViewer Callback called for image: %s", imgName.c_str());

		cv_bridge::CvImagePtr cv_color = cv_bridge::toCvCopy(image_msg,
				sensor_msgs::image_encodings::BGR8);
		RoiRect roi;

		for(unsigned int i=0;i<rois_msg->rois.size();i++)
		{
			roi = rois_msg->rois[i];
			
			if(roi.label==label){
				
				Point ptUpperLeft = Point(roi.x,roi.y);
				Point ptLowerRight = Point(roi.x+roi.width,roi.y+roi.height);

				rectangle(cv_color->image,ptUpperLeft,ptLowerRight,Scalar(255,255,255));

				if (show_confidence)
				{
					// Draw roi confidence
					float confidenceToDisplay = float(int(roi.confidence*100))/100;
					std::stringstream conf_ss;
					conf_ss << confidenceToDisplay;
					cv::rectangle(cv_color->image, cv::Point(roi.x, roi.y-15),	cv::Point(roi.x +60, roi.y), darkBlue, CV_FILLED, 8);
					cv::putText(cv_color->image, conf_ss.str(), cv::Point(roi.x, roi.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, white, 1.7, CV_AA);
				}
			}	
		}

		// Display the cv image
		//cv::namedWindow("ROI Color Image",1);
		//ROS_INFO("Showing detections");
		cv::imshow("Detections",cv_color->image);
		cv::waitKey(3);
		
	}
	~roiViewerNode()
	{
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roiViewer");
	ros::NodeHandle n;
	roiViewerNode roiViewerNode(n);
	ros::spin();

	return 0;
}

