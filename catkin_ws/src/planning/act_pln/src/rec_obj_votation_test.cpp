#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/tracking.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTools.h"
#include "pcl_conversions/pcl_conversions.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

ros::ServiceClient cltDetectObjects;


int main(int argc, char** argv){
	ros::init(argc, argv, "votation_test");
	ros::NodeHandle n;
	ros::Rate rate(10);

	JustinaVision::setNodeHandle(&n);
    	//cltDetectObjects         = n.serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/vot_objs");
    	cltDetectObjects         = n.serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/tf_object");

	std::vector<vision_msgs::VisionObject> recognizedObjects;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	
	std::cout << "Find a object " << std::endl;

    	vision_msgs::DetectObjects srv;
	std::vector<vision_msgs::VisionObject> recoObjList;
	sensor_msgs::Image container;
	srv.request.saveFiles = false;
	srv.request.iterations = 10;
    	if(!cltDetectObjects.call(srv))
    	{
        	std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
        	return -1;
    	}
	container = srv.response.image;
	cv_bridge::CvImagePtr img;
	img = cv_bridge::toCvCopy(container, sensor_msgs::image_encodings::BGR8);
	cv::Mat mat_received = img->image;
	std::cout << "FILAS: " << mat_received.rows << std::endl;
	std::cout << "COLUMNAS: " << mat_received.cols << std::endl;
	std::cout << "mat type: " << mat_received.type() << std::endl;
	cv::Mat mat;
	mat_received.convertTo(mat, 16);
	//cv::imshow("rectangles", mat);
	//cv::waitKey(0);
    	recoObjList=srv.response.recog_objects;
   	if(recoObjList.size() < 1)
    	{
        	std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
        	return -1;
    	}
    	std::cout << "JustinaVision.->Detected " << int(recoObjList.size()) << " objects" << std::endl;
	std::stringstream ss;
	cv::Rect boundBox;
	
	for(std::vector<vision_msgs::VisionObject>::const_iterator it = recoObjList.begin(); it != recoObjList.end(); it++){
		std::cout << "Name: " << it->id << std::endl;
		std::cout << "confidence: " << it->confidence << std::endl;
		std::cout << "Bounding box: " << it->x << ", " << it->y << ", " << it->width << ", " << it->height << std::endl;
		boundBox.x = it->x;
		boundBox.y = it->y;
		boundBox.width = it->width;
		boundBox.height = it->height;
		ss << it->id << " "  << it->confidence;
		if(it->confidence > 0.5){
			cv::rectangle(mat, boundBox, cv::Scalar(0, 0, 255));
			cv::putText(mat, ss.str(), boundBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255));
		}

		ss.str("");
		
	}
	cv::imshow("rectangles", mat);
	cv::waitKey(0);

	while(ros::ok()){
	rate.sleep();
	ros::spinOnce();}
	return 0;	
}
