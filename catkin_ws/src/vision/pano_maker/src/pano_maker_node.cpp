#include <iostream>

#include "PanoMaker.hpp"

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "vision_msgs/GetPanoramic.h"

#include <justina_tools/JustinaTools.h>



ros::NodeHandle* node;

ros::ServiceClient cli_rgbdRobot;
ros::ServiceServer srv_getPanoramic;
ros::Subscriber sub_takeImage;
ros::Subscriber sub_clearImages;
ros::Subscriber sub_makePanoramic; 
ros::Publisher pub_panoramic; 

bool dbMode = true;
PanoMaker panoMaker; 
        
bool GetImagesFromJustina(cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cli_rgbdRobot.call(srv))
        return false;

    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}
 
bool cb_srv_getPanoramic(vision_msgs::GetPanoramic::Request &req, vision_msgs::GetPanoramic::Response &resp)
{
    float hdPanMin = req.head_pan_min;
    float hdPanMax = req.head_pan_max;
    float hdTiltMin = req.head_tilt_min;
    float hdTiltMax = req.head_tilt_max; 
    std::cout << "Info: PanoMaker (getPanoramic) - pan=[" << hdPanMin << "," << hdPanMax << "] tilt=[" << hdTiltMax << "," << hdTiltMin << "]" << std::endl; 
}

void cb_sub_takeImage(const std_msgs::Empty::ConstPtr& msg)
{
    cv::Mat imaBGR; 
    cv::Mat imaXYZ;
    if( !GetImagesFromJustina( imaBGR, imaXYZ) )
    {
        std::cout << "PanoMaker (cv_sub_takeImage) : Cant get images from Justina" << std::endl; 
        return; 
    }

    if( dbMode )
        cv::imshow("addImage", imaBGR); 

    panoMaker.AddImage( imaBGR , imaXYZ); 
    return; 
} 

void cb_sub_clearImages(const std_msgs::Empty::ConstPtr& msg)
{
    panoMaker.ClearImages();
}

void cb_sub_makePanoramic(const std_msgs::Empty::ConstPtr& msg)
{
    cv::Mat panoBGR; 
    cv::Mat panoXYZ; 
    if( !panoMaker.MakePanoramic(panoBGR, panoXYZ) )
        return; 

    if( dbMode )
        imshow( "panoBGR", panoBGR);

    //Converting to image with cv_bridge
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();

    cv_bridge::CvImage imgBridge;
    imgBridge = cv_bridge::CvImage( header, sensor_msgs::image_encodings::BGR8, panoBGR); 
    
    sensor_msgs::Image imgMsg;
    imgBridge.toImageMsg(imgMsg);

    pub_panoramic.publish( imgMsg ); 
}   

int main(int argc, char** argv)
{
	std::cout << " >>>>> INIT PANO MAKER NODE <<<<<" << std::endl; 
	
	ros::init(argc, argv, "pano_maker_node"); 
	ros::NodeHandle n;
    node = &n;
	ros::Rate loop(60); 

    panoMaker = PanoMaker();  
    
    cli_rgbdRobot       = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

    //srv_getPanoramic    = n.advertiseService("/vision/pano_maker/get_panoramic", cb_srv_getPanoramic);
    
    sub_takeImage       = n.subscribe("/vision/pano_maker/take_image"   , 1, cb_sub_takeImage); 
    sub_clearImages     = n.subscribe("/vision/pano_maker/clear_images" , 1, cb_sub_clearImages); 
    sub_makePanoramic   = n.subscribe("/vision/pano_maker/make_panoramic"    , 1, cb_sub_makePanoramic);  
    pub_panoramic       = n.advertise< sensor_msgs::Image >( "/vision/pano_maker/panoramic", 1);

	while(ros::ok)
	{
		ros::spinOnce();
		loop.sleep();

        if( cv::waitKey(1) == 'q' )
            break;
    }
 
    cv::destroyAllWindows();
    return 0; 
}
