#include <iostream>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "vision_msgs/GetPanoramic.h"
#include "PanoMaker.hpp"

#include <justina_tools/JustinaTools.h>

ros::NodeHandle* node;

ros::ServiceClient cli_rgbdRobot;
ros::ServiceServer srv_getPanoramic;
ros::Subscriber sub_takeImage;
ros::Subscriber sub_clearImages;
ros::Subscriber sub_makePanoramic; 
ros::Publisher pub_panoramicImage; 
ros::Publisher pub_panoramicCloud;
ros::Publisher pub_noImages;

bool dbMode = false;
PanoMaker panoMaker; 
        
bool GetImagesFromJustina(cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cli_rgbdRobot.call(srv))
        return false;

    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}
 
sensor_msgs::Image Mat2SensorImage(cv::Mat ima)
{
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time::now();

    std::string encoding; 
    if( ima.type() == CV_8UC3) 
        encoding = sensor_msgs::image_encodings::BGR8;

    cv_bridge::CvImage imgBridge;
    imgBridge = cv_bridge::CvImage( header, encoding, ima); 
    
    sensor_msgs::Image imgMsg;
    imgBridge.toImageMsg(imgMsg);

    return imgMsg; 
}

bool cb_srv_getPanoramic(vision_msgs::GetPanoramic::Request &req, vision_msgs::GetPanoramic::Response &resp)
{
    float hdPanMin = req.head_pan_min;
    float hdPanMax = req.head_pan_max;
    float hdTiltMin = req.head_tilt_min;
    float hdTiltMax = req.head_tilt_max; 
    std::cout << "Info: PanoMaker (cb_srv_getPanoramic) - pan=[" << hdPanMin << "," << hdPanMax << "] tilt=[" << hdTiltMax << "," << hdTiltMin << "]" << std::endl; 

    if( hdPanMin == hdPanMax && hdTiltMin == hdTiltMax  )
    {
        // use the images from list. 
        cv::Mat panoBGR; 
        cv::Mat panoXYZ; 
        if( !panoMaker.MakePanoramic(panoBGR, panoXYZ) )
            return false;  

        if( dbMode )
        {
            imshow( "panoBGR", panoBGR );
            imshow( "panoXYZ", panoXYZ ); 
        }
        
        resp.panoramic_image = Mat2SensorImage( panoBGR ); 
        //resp.panoramic_cloud = Mat2SensorImage( panoXYZ );

        panoMaker.ClearImages();
        std_msgs::Int8 no_image;
        no_image.data = 0;
        pub_noImages.publish(no_image); 

        return true; 
    }

    return false; 
}

void cb_sub_takeImage(const std_msgs::Empty::ConstPtr& msg)
{
    cv::Mat imaBGR; 
    cv::Mat imaXYZ;
    if( !GetImagesFromJustina( imaBGR, imaXYZ) )
    {
        std::cout << "Info PanoMaker (cv_sub_takeImage) : Cant get images from Justina" << std::endl; 
        return; 
    }

    if( dbMode )
        cv::imshow("addImage", imaBGR); 

    panoMaker.AddImage( imaBGR , imaXYZ); 
    std_msgs::Int8 no_image;
    no_image.data = panoMaker.GetNoImages();
    pub_noImages.publish(no_image); 
    return; 
} 

void cb_sub_clearImages(const std_msgs::Empty::ConstPtr& msg)
{
    panoMaker.ClearImages();
    std_msgs::Int8 no_image;
    no_image.data = 0;
    pub_noImages.publish(no_image); 
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
    pub_panoramicImage.publish( Mat2SensorImage( panoBGR ) ); 
    //pub_panoramicCloud.publish( Mat2RosIma( panoXYZ ) ); 

    panoMaker.ClearImages();
    std_msgs::Int8 no_image;
    no_image.data = 0;
    pub_noImages.publish(no_image);
}

void GetParams(int argc, char** argv)
{ 
    for( int i=0; i<argc; i++)
    {
        std::string params( argv[i] );

        if( params == "--d" )
        {
            dbMode = true;
            std::cout << "PanoMaker (DebugMode ON)" << std::endl;
        }
    }
}

int main(int argc, char** argv)
{
	std::cout << " >>>>> INIT PANO MAKER NODE <<<<<" << std::endl; 
    GetParams(argc, argv); 

	
	ros::init(argc, argv, "pano_maker_node"); 
	ros::NodeHandle n;
    node = &n;
	ros::Rate loop(60); 

    panoMaker = PanoMaker();  
    panoMaker.configdir = ros::package::getPath("pano_maker") ;
    
    std::cout << panoMaker.configdir << std::endl;
    
    cli_rgbdRobot       = n.serviceClient   <point_cloud_manager::GetRgbd>  ("/hardware/point_cloud_man/get_rgbd_wrt_robot");

    srv_getPanoramic    = n.advertiseService("/vision/pano_maker/get_panoramic", cb_srv_getPanoramic);
    
    sub_takeImage       = n.subscribe("/vision/pano_maker/take_image"       , 1, cb_sub_takeImage); 
    sub_clearImages     = n.subscribe("/vision/pano_maker/clear_images"     , 1, cb_sub_clearImages); 
    sub_makePanoramic   = n.subscribe("/vision/pano_maker/make_panoramic"   , 1, cb_sub_makePanoramic);  

    pub_panoramicImage  = n.advertise   < sensor_msgs::Image >          ("/vision/pano_maker/panoramic_image", 1);
    pub_panoramicCloud  = n.advertise   < sensor_msgs::PointCloud2 >    ("/vision/pano_maker/panoramic_cloud", 1);
    pub_noImages        = n.advertise   < std_msgs::Int8 >              ("/vision/pano_maker/no_images", 1);

	while(ros::ok)
	{
		ros::spinOnce();
		loop.sleep();

        if( cv::waitKey(1) == 'q' )
            break;
    }
 
    sub_takeImage.shutdown();
    sub_clearImages.shutdown();
    sub_makePanoramic.shutdown();
    pub_panoramicImage.shutdown();
    pub_panoramicCloud.shutdown();

    cv::destroyAllWindows();
    return 0; 
}
